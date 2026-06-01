#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
import sys
import time
from typing import Callable, Iterable, Protocol

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
SCRIPT_ROOT = REPO_ROOT / "scripts"
for path in (SRC_ROOT, SCRIPT_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

DEFAULT_PORT = "/dev/c30d"
DEFAULT_BAUD = 115200
DEFAULT_CAPTURE_DURATION_S = 5.0
DEFAULT_READ_SIZE = 256
INSPECTED_DIRS = (
    Path("data/c30d_captures"),
    Path("data/c30d_live"),
    Path("data/c30d_analysis"),
    Path("data/c30d_diagnostics"),
)
REPORT_ROOT = Path("data/c30d_diagnostics")
ZERO_NEUTRAL_FRAME_HEX = "7b 00 00 00 00 00 00 00 00 7b 7d"
KNOWN_TEST_FACTS = (
    "zero frame: safe",
    "target_x=0.05 single pulse: no movement",
    "target_x=0.05 stream: no movement",
    "reserved byte stream variants 00/00, 00/01, 01/00, 01/01: no movement",
    "target_x=0.10 deadband probe: no movement",
    "angular_z probe target_z=0.05: no steering/yaw/movement",
)
REQUIRED_LIVE_CONFIRMATIONS = (
    "wheels_lifted",
    "robot_restrained",
    "motor_switch_reviewed",
    "manual_power_cutoff_ready",
)


class SerialHandle(Protocol):
    def read(self, size: int) -> bytes: ...

    def write(self, data: bytes) -> int: ...

    def flush(self) -> None: ...

    def close(self) -> None: ...


class SerialFactory(Protocol):
    def __call__(self, port: str, baud: int) -> SerialHandle: ...


@dataclass(frozen=True)
class ByteDifference:
    position: int
    left_values: tuple[int, ...]
    right_values: tuple[int, ...]


@dataclass(frozen=True)
class CaptureSummary:
    path: Path
    kind: str
    byte_count: int
    frame_count: int = 0
    valid_checksum_count: int = 0
    invalid_checksum_count: int = 0
    partial_frame_count: int = 0
    rejected_resync_count: int = 0
    battery_min_mV: int | None = None
    battery_max_mV: int | None = None
    frame_rate_hz: float | None = None
    changing_byte_positions: tuple[int, ...] = ()
    status_byte_values: tuple[int, ...] = ()
    phases: tuple[str, ...] = ()
    apparent_write_effect: bool = False
    notes: tuple[str, ...] = ()


@dataclass(frozen=True)
class StateComparison:
    left_label: str
    right_label: str
    byte_differences: tuple[ByteDifference, ...]
    field_differences: tuple[str, ...]


@dataclass
class DiagnosisState:
    report_dir: Path
    inspected_paths: list[Path] = field(default_factory=list)
    captures: list[CaptureSummary] = field(default_factory=list)
    comparisons: list[StateComparison] = field(default_factory=list)
    commands_run: list[str] = field(default_factory=list)
    captures_collected: list[Path] = field(default_factory=list)
    confirmed_facts: list[str] = field(default_factory=lambda: list(KNOWN_TEST_FACTS))
    hypotheses: list[str] = field(default_factory=list)
    untested_assumptions: list[str] = field(default_factory=list)
    conclusion: str = ""
    recommended_next_step: str = ""
    live_captures_required: bool = False
    decision_trace: list[str] = field(default_factory=list)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Autonomous C30D diagnosis runner. Offline analysis is read-only. Guided live mode "
            "prompts before any write and only sends a known-safe zero frame when explicitly approved."
        )
    )
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--offline-only", action="store_true")
    modes.add_argument("--guided-live", action="store_true")
    modes.add_argument("--continue-until-exhausted", action="store_true")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--capture-duration", type=float, default=DEFAULT_CAPTURE_DURATION_S)
    parser.add_argument("--non-interactive", action="store_true")
    return parser


def timestamped_report_dir(clock: Callable[[], datetime] = datetime.now) -> Path:
    return REPORT_ROOT / clock().strftime("%Y%m%d_%H%M%S")


def open_serial_handle(port: str, baud: int) -> SerialHandle:
    import serial

    return serial.Serial(port=port, baudrate=baud, timeout=0.02, write_timeout=1.0)


def iter_existing_files(base_dirs: Iterable[Path]) -> list[Path]:
    paths: list[Path] = []
    for directory in base_dirs:
        if not directory.exists():
            continue
        paths.extend(path for path in directory.rglob("*") if path.is_file())
    return sorted(paths)


def _parse_bool(value: str | bool | None) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "y"}


def _candidate_int(row: dict[str, str], *names: str) -> int | None:
    for name in names:
        if name in row and row[name] not in ("", None):
            try:
                return int(float(row[name]))
            except ValueError:
                return None
    return None


def _candidate_float(row: dict[str, str], *names: str) -> float | None:
    for name in names:
        if name in row and row[name] not in ("", None):
            try:
                return float(row[name])
            except ValueError:
                return None
    return None


def analyze_binary_capture(path: Path) -> CaptureSummary:
    from ackermann_robot.drivers.c30d_feedback import parse_feedback_candidates
    from ackermann_robot.drivers.c30d_frames import (
        changing_byte_positions,
        extract_frames_with_stats,
        filter_frames_by_checksum,
    )

    data = path.read_bytes()
    extraction = extract_frames_with_stats(data)
    frames = extraction.frames
    valid_frames = filter_frames_by_checksum(frames, require_valid=True)
    candidates = parse_feedback_candidates(frames) if frames else []
    batteries = [candidate.candidate_battery_mV for candidate in candidates]
    status_values = sorted({frame[21] for frame in valid_frames if len(frame) > 21})
    return CaptureSummary(
        path=path,
        kind="binary_capture",
        byte_count=len(data),
        frame_count=len(frames),
        valid_checksum_count=len(valid_frames),
        invalid_checksum_count=len(frames) - len(valid_frames),
        partial_frame_count=extraction.partial_frame_count,
        rejected_resync_count=extraction.rejected_resync_count,
        battery_min_mV=min(batteries, default=None),
        battery_max_mV=max(batteries, default=None),
        changing_byte_positions=tuple(changing_byte_positions(valid_frames or frames)),
        status_byte_values=tuple(status_values),
    )


def analyze_csv_capture(path: Path) -> CaptureSummary:
    rows: list[dict[str, str]] = []
    with path.open(newline="", encoding="utf-8") as input_file:
        reader = csv.DictReader(input_file)
        rows = list(reader)

    phases = tuple(sorted({row.get("phase", "") for row in rows if row.get("phase")}))
    batteries = [
        value
        for row in rows
        if (value := _candidate_int(row, "candidate_battery_mV", "candidate_battery_mV_min"))
        is not None
    ]
    checksums = [
        _parse_bool(row.get("checksum_valid"))
        for row in rows
        if "checksum_valid" in row and row.get("checksum_valid") != ""
    ]
    timestamps = [
        value
        for row in rows
        if (value := _candidate_float(row, "monotonic_timestamp", "timestamp_s")) is not None
    ]
    frame_rate = None
    if len(timestamps) > 1 and max(timestamps) > min(timestamps):
        frame_rate = (len(timestamps) - 1) / (max(timestamps) - min(timestamps))

    apparent_write_effect = False
    if phases:
        baseline = [
            _candidate_int(row, "forward_candidate", "candidate_forward_motion")
            for row in rows
            if row.get("phase") == "baseline"
        ]
        active = [
            _candidate_int(row, "forward_candidate", "candidate_forward_motion")
            for row in rows
            if row.get("phase") not in {"baseline", ""}
        ]
        baseline_abs = max((abs(value) for value in baseline if value is not None), default=0)
        active_abs = max((abs(value) for value in active if value is not None), default=0)
        apparent_write_effect = active_abs > baseline_abs + 1

    return CaptureSummary(
        path=path,
        kind="csv_capture",
        byte_count=path.stat().st_size,
        frame_count=len(rows),
        valid_checksum_count=sum(1 for value in checksums if value),
        invalid_checksum_count=sum(1 for value in checksums if not value),
        battery_min_mV=min(batteries, default=None),
        battery_max_mV=max(batteries, default=None),
        frame_rate_hz=frame_rate,
        phases=phases,
        apparent_write_effect=apparent_write_effect,
    )


def analyze_text_artifact(path: Path) -> CaptureSummary:
    text = path.read_text(encoding="utf-8", errors="replace")
    notes: list[str] = []
    lower = text.lower()
    for token in ("no movement", "zero", "checksum", "target_x", "target_z", "reserved"):
        if token in lower:
            notes.append(f"mentions_{token.replace('_', '-')}")
    return CaptureSummary(
        path=path,
        kind="text_artifact",
        byte_count=path.stat().st_size,
        notes=tuple(notes),
    )


def analyze_path(path: Path) -> CaptureSummary:
    suffix = path.suffix.lower()
    if suffix == ".bin":
        return analyze_binary_capture(path)
    if suffix == ".csv":
        return analyze_csv_capture(path)
    return analyze_text_artifact(path)


def verify_host_command_builder() -> tuple[bool, list[str]]:
    from ackermann_robot.drivers.c30d_checksum import xor_checksum
    from ackermann_robot.drivers.c30d_host_command_frame import (
        build_ackermann_host_command_frame,
        build_ackermann_host_command_frame_from_floats,
    )

    notes: list[str] = []
    zero = build_ackermann_host_command_frame(0, 0, 0, 0, 0)
    candidate = build_ackermann_host_command_frame_from_floats(0, 1, target_x=0.05, target_z=0.0)
    checks = {
        "zero_frame_matches_known_safe": zero.hex(" ") == ZERO_NEUTRAL_FRAME_HEX,
        "frame_length_11": len(candidate.frame) == 11,
        "start_byte_0x7b": candidate.frame[0] == 0x7B,
        "end_byte_0x7d": candidate.frame[10] == 0x7D,
        "target_x_scaled_by_1000_in_bytes_3_4": candidate.frame[3:5]
        == (50).to_bytes(2, "big", signed=True),
        "target_y_zero_in_bytes_5_6": candidate.frame[5:7] == b"\x00\x00",
        "target_z_zero_in_bytes_7_8": candidate.frame[7:9] == b"\x00\x00",
        "checksum_xor_bytes_0_through_8": candidate.frame[9] == xor_checksum(candidate.frame[:9]),
    }
    for name, passed in checks.items():
        notes.append(f"{name}: {'pass' if passed else 'fail'}")
    return all(checks.values()), notes


def detect_byte_differences(
    left_frames: Iterable[bytes],
    right_frames: Iterable[bytes],
) -> tuple[ByteDifference, ...]:
    left = list(left_frames)
    right = list(right_frames)
    if not left or not right:
        return ()
    max_length = max(max(len(frame) for frame in left), max(len(frame) for frame in right))
    differences: list[ByteDifference] = []
    for position in range(max_length):
        left_values = sorted({frame[position] for frame in left if position < len(frame)})
        right_values = sorted({frame[position] for frame in right if position < len(frame)})
        if left_values != right_values:
            differences.append(
                ByteDifference(
                    position=position,
                    left_values=tuple(left_values),
                    right_values=tuple(right_values),
                )
            )
    return tuple(differences)


def compare_capture_groups(label_to_frames: dict[str, list[bytes]]) -> list[StateComparison]:
    from ackermann_robot.drivers.c30d_feedback import parse_feedback_candidates

    comparisons: list[StateComparison] = []
    labels = sorted(label_to_frames)
    for index, left_label in enumerate(labels):
        for right_label in labels[index + 1 :]:
            left_frames = label_to_frames[left_label]
            right_frames = label_to_frames[right_label]
            byte_differences = detect_byte_differences(left_frames, right_frames)
            field_differences: list[str] = []
            if left_frames and right_frames:
                left_candidates = parse_feedback_candidates(left_frames)
                right_candidates = parse_feedback_candidates(right_frames)
                left_battery = {candidate.candidate_battery_mV for candidate in left_candidates}
                right_battery = {candidate.candidate_battery_mV for candidate in right_candidates}
                if left_battery != right_battery:
                    field_differences.append("candidate_battery_mV")
                left_forward = {candidate.candidate_forward_motion for candidate in left_candidates}
                right_forward = {
                    candidate.candidate_forward_motion for candidate in right_candidates
                }
                if left_forward != right_forward:
                    field_differences.append("candidate_forward_motion")
                left_yaw = {candidate.candidate_yaw_motion for candidate in left_candidates}
                right_yaw = {candidate.candidate_yaw_motion for candidate in right_candidates}
                if left_yaw != right_yaw:
                    field_differences.append("candidate_yaw_motion")
            comparisons.append(
                StateComparison(
                    left_label=left_label,
                    right_label=right_label,
                    byte_differences=byte_differences,
                    field_differences=tuple(field_differences),
                )
            )
    return comparisons


def run_offline_analysis(report_dir: Path) -> DiagnosisState:
    from ackermann_robot.drivers.c30d_frames import extract_frames, filter_frames_by_checksum

    state = DiagnosisState(report_dir=report_dir)
    state.commands_run.append("python scripts/c30d_autonomous_diagnosis_runner.py --offline-only")
    paths = iter_existing_files(INSPECTED_DIRS)
    state.inspected_paths.extend(paths)
    for path in paths:
        try:
            state.captures.append(analyze_path(path))
        except (OSError, ValueError, csv.Error) as exc:
            state.captures.append(
                CaptureSummary(
                    path=path,
                    kind="unreadable_artifact",
                    byte_count=path.stat().st_size if path.exists() else 0,
                    notes=(f"analysis_error: {exc}",),
                )
            )

    builder_ok, builder_notes = verify_host_command_builder()
    if builder_ok:
        state.confirmed_facts.append("Python host command builder matches the 11-byte layout.")
    else:
        state.hypotheses.append("Python host command builder does not match the expected layout.")
    state.confirmed_facts.extend(builder_notes)

    label_to_frames: dict[str, list[bytes]] = {}
    for summary in state.captures:
        if summary.kind == "binary_capture":
            frames = filter_frames_by_checksum(extract_frames(summary.path.read_bytes()), True)
            if frames:
                label_to_frames[summary.path.stem] = frames
    state.comparisons.extend(compare_capture_groups(label_to_frames))

    total_frames = sum(summary.frame_count for summary in state.captures)
    total_invalid = sum(summary.invalid_checksum_count for summary in state.captures)
    live_artifacts = [
        summary
        for summary in state.captures
        if "c30d_live" in summary.path.parts or summary.phases or summary.apparent_write_effect
    ]
    write_effects = [summary for summary in state.captures if summary.apparent_write_effect]
    if write_effects:
        state.hypotheses.append("At least one CSV shows a feedback change after a write phase.")
    else:
        state.confirmed_facts.append(
            "No existing CSV shows a clear feedback change after write phases."
        )

    state.live_captures_required = not live_artifacts
    if state.live_captures_required:
        state.untested_assumptions.append(
            "No guided-live OFF/ON/USER/RESET/zero capture set is available yet."
        )
    if total_frames == 0:
        state.untested_assumptions.append("No parseable C30D feedback frames were found offline.")
    if total_invalid > 0:
        state.hypotheses.append(
            "Some captures include invalid feedback checksums; live writes require zero."
        )

    state.conclusion = f"Offline analysis inspected {len(paths)} artifacts and parsed {total_frames} feedback rows/frames."
    state.recommended_next_step = choose_recommendation(state)
    return state


def choose_recommendation(state: DiagnosisState) -> str:
    if state.live_captures_required:
        return "continue C30D protocol work: collect guided-live state captures first"
    if any(summary.apparent_write_effect for summary in state.captures):
        return "continue C30D protocol work: analyze the active feedback-changing write path"
    if any(summary.invalid_checksum_count for summary in state.captures):
        return (
            "investigate firmware/download port read-only or feedback integrity before live writes"
        )
    return "bypass C30D actuation using separate MCU/motor drivers"


def prompt_enter(message: str, input_fn: Callable[[str], str] = input) -> None:
    input_fn(f"{message}\nPress Enter when ready, or Ctrl-C to abort: ")


def require_typed_yes(
    reason: str,
    input_fn: Callable[[str], str] = input,
    required: str = "YES",
) -> bool:
    response = input_fn(f"{reason}\nType {required} to continue: ")
    return response == required


def require_live_safety_confirmation(input_fn: Callable[[str], str] = input) -> bool:
    print("Live C30D write safety checklist:")
    print("- wheels lifted")
    print("- robot restrained")
    print("- motor switch reviewed")
    print("- manual power cutoff ready")
    for name in REQUIRED_LIVE_CONFIRMATIONS:
        if not require_typed_yes(f"Confirm {name.replace('_', ' ')}.", input_fn=input_fn):
            return False
    return require_typed_yes("Final confirmation for this live write.", input_fn=input_fn)


def capture_passive_feedback(
    label: str,
    output_dir: Path,
    *,
    port: str,
    baud: int,
    duration_s: float,
    serial_factory: SerialFactory = open_serial_handle,
    sleep_fn: Callable[[float], None] = time.sleep,
    clock: Callable[[], float] = time.monotonic,
) -> Path:
    handle = serial_factory(port, baud)
    data = bytearray()
    deadline = clock() + duration_s
    try:
        while clock() < deadline:
            chunk = handle.read(DEFAULT_READ_SIZE)
            if chunk:
                data.extend(chunk)
            sleep_fn(0.005)
    finally:
        handle.close()
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / f"{label}.bin"
    output_path.write_bytes(bytes(data))
    return output_path


def run_readiness_for_live_zero(duration_s: float = 5.0):
    import c30d_first_write_readiness as readiness

    preflight = readiness.run_readonly_preflight(
        duration_s,
        readiness.PREFLIGHT_MODE_C30D_ONLY,
        readiness.DEFAULT_C30D_WARMUP_DURATION_S,
    )
    threshold = readiness.load_warning_battery_threshold()
    confirmations = {
        "wheels_lifted": True,
        "robot_restrained": True,
        "manual_power_cutoff_ready": True,
        "motor_enable_switch_reviewed": True,
        "i_understand_this_is_not_a_motor_test": True,
    }
    report = readiness.evaluate_readiness(confirmations, preflight, threshold)
    readiness.print_report(report)
    return report


def write_zero_frame_after_confirmation(
    *,
    port: str,
    baud: int,
    input_fn: Callable[[str], str] = input,
    serial_factory: SerialFactory = open_serial_handle,
) -> bool:
    from send_c30d_zero_frame_once import build_zero_frame, validate_zero_frame

    if not require_live_safety_confirmation(input_fn=input_fn):
        print("zero_frame_write: refused_by_user_confirmation")
        return False
    report = run_readiness_for_live_zero()
    preflight = report.preflight
    if (
        not report.readiness_allowed
        or preflight.counted_invalid_checksum_count not in (None, 0)
        or preflight.invalid_checksum_count not in (None, 0)
    ):
        print("zero_frame_write: refused_by_readiness")
        return False
    frame = build_zero_frame()
    validation = validate_zero_frame(frame)
    if not validation.valid:
        print(f"zero_frame_write: refused_invalid_zero_frame {', '.join(validation.reasons)}")
        return False
    handle = serial_factory(port, baud)
    try:
        handle.write(frame)
        handle.flush()
    finally:
        handle.close()
    print(f"zero_frame_write: sent {frame.hex(' ')}")
    return True


def run_guided_live(
    report_dir: Path,
    *,
    port: str,
    baud: int,
    capture_duration_s: float,
    input_fn: Callable[[str], str] = input,
    serial_factory: SerialFactory = open_serial_handle,
) -> DiagnosisState:
    state = DiagnosisState(report_dir=report_dir)
    state.commands_run.append("python scripts/c30d_autonomous_diagnosis_runner.py --guided-live")
    capture_dir = report_dir / "captures"

    steps = (
        ("motor_switch_off", "Set motor switch OFF."),
        ("motor_switch_on", "Set motor switch ON."),
        ("user_button_pressed_released", "Press and release USER once."),
    )
    for label, prompt in steps:
        prompt_enter(prompt, input_fn=input_fn)
        path = capture_passive_feedback(
            label,
            capture_dir,
            port=port,
            baud=baud,
            duration_s=capture_duration_s,
            serial_factory=serial_factory,
        )
        state.captures_collected.append(path)
        state.captures.append(analyze_binary_capture(path))

    if require_typed_yes("Optionally press RESET, then capture after reset.", input_fn=input_fn):
        path = capture_passive_feedback(
            "after_reset",
            capture_dir,
            port=port,
            baud=baud,
            duration_s=capture_duration_s,
            serial_factory=serial_factory,
        )
        state.captures_collected.append(path)
        state.captures.append(analyze_binary_capture(path))

    if require_typed_yes(
        "Optionally send known-safe zero frame and capture after it.", input_fn=input_fn
    ):
        sent = write_zero_frame_after_confirmation(
            port=port,
            baud=baud,
            input_fn=input_fn,
            serial_factory=serial_factory,
        )
        if sent:
            path = capture_passive_feedback(
                "after_zero_frame",
                capture_dir,
                port=port,
                baud=baud,
                duration_s=capture_duration_s,
                serial_factory=serial_factory,
            )
            state.captures_collected.append(path)
            state.captures.append(analyze_binary_capture(path))

    from ackermann_robot.drivers.c30d_frames import extract_frames, filter_frames_by_checksum

    label_to_frames = {}
    for path in state.captures_collected:
        frames = filter_frames_by_checksum(extract_frames(path.read_bytes()), True)
        if frames:
            label_to_frames[path.stem] = frames
    state.comparisons.extend(compare_capture_groups(label_to_frames))
    state.conclusion = (
        f"Guided-live collected {len(state.captures_collected)} passive capture files."
    )
    state.recommended_next_step = choose_recommendation(state)
    return state


def all_branches_exhausted(state: DiagnosisState) -> bool:
    if state.live_captures_required:
        return False
    if any(summary.apparent_write_effect for summary in state.captures):
        return False
    if any(comparison.byte_differences for comparison in state.comparisons):
        return False
    return True


def run_continue_until_exhausted(
    report_dir: Path,
    *,
    port: str,
    baud: int,
    capture_duration_s: float,
    non_interactive: bool,
    input_fn: Callable[[str], str] = input,
    serial_factory: SerialFactory = open_serial_handle,
) -> DiagnosisState:
    state = run_offline_analysis(report_dir)
    state.commands_run[0] = (
        "python scripts/c30d_autonomous_diagnosis_runner.py --continue-until-exhausted"
    )
    state.decision_trace.append("A. status/mode-byte discovery: offline comparison completed")
    state.decision_trace.append(
        "B. active UART/control-path verification: existing write data reviewed"
    )
    state.decision_trace.append("C. command layout verification: Python builder checked")

    if state.live_captures_required:
        state.decision_trace.append(
            "Existing data insufficient; guided-live captures are required."
        )
        if non_interactive:
            state.conclusion = "Stopped before live captures because --non-interactive was set."
            state.recommended_next_step = (
                "continue C30D protocol work: run --guided-live on the robot"
            )
            return state
        if require_typed_yes("Guided-live passive captures are required.", input_fn=input_fn):
            live_state = run_guided_live(
                report_dir,
                port=port,
                baud=baud,
                capture_duration_s=capture_duration_s,
                input_fn=input_fn,
                serial_factory=serial_factory,
            )
            state.captures.extend(live_state.captures)
            state.comparisons.extend(live_state.comparisons)
            state.captures_collected.extend(live_state.captures_collected)
            state.live_captures_required = False

    if any(summary.apparent_write_effect for summary in state.captures):
        state.decision_trace.append(
            "D. low-risk live write re-test: evidence supports protocol work, but no automatic probe run."
        )
        state.recommended_next_step = "continue C30D protocol work"
    elif all_branches_exhausted(state):
        state.decision_trace.append("E. no remaining software/protocol path with current evidence.")
        state.recommended_next_step = "bypass C30D actuation using separate MCU/motor drivers"
        state.conclusion = (
            "Reasonable non-flashing software/protocol branches are exhausted by available data."
        )
    else:
        state.decision_trace.append("E. stop before unsafe or unsupported live probe.")
        state.recommended_next_step = choose_recommendation(state)
    return state


def _format_values(values: tuple[int, ...]) -> str:
    if not values:
        return "none"
    return ", ".join(f"0x{value:02x}" for value in values)


def _format_capture(summary: CaptureSummary) -> str:
    parts = [
        f"- `{summary.path}` ({summary.kind})",
        f"  - bytes: {summary.byte_count}",
        f"  - frames/rows: {summary.frame_count}",
        f"  - checksums valid/invalid: {summary.valid_checksum_count}/{summary.invalid_checksum_count}",
    ]
    if summary.battery_min_mV is not None:
        parts.append(f"  - battery mV min/max: {summary.battery_min_mV}/{summary.battery_max_mV}")
    if summary.frame_rate_hz is not None:
        parts.append(f"  - frame rate Hz: {summary.frame_rate_hz:.2f}")
    if summary.changing_byte_positions:
        parts.append(
            "  - changing byte positions: "
            + ", ".join(str(position) for position in summary.changing_byte_positions)
        )
    if summary.status_byte_values:
        parts.append(f"  - byte 21 values: {_format_values(summary.status_byte_values)}")
    if summary.phases:
        parts.append(f"  - phases: {', '.join(summary.phases)}")
    if summary.apparent_write_effect:
        parts.append("  - apparent write feedback effect: yes")
    if summary.notes:
        parts.append(f"  - notes: {', '.join(summary.notes)}")
    return "\n".join(parts)


def render_report(state: DiagnosisState) -> str:
    status_candidates = sorted(
        {
            value
            for summary in state.captures
            for value in summary.status_byte_values
            if summary.invalid_checksum_count == 0 or summary.valid_checksum_count > 0
        }
    )
    switch_diffs = [
        comparison
        for comparison in state.comparisons
        if {"motor_switch_off", "motor_switch_on"}
        <= {comparison.left_label, comparison.right_label}
    ]
    user_diffs = [
        comparison
        for comparison in state.comparisons
        if "user_button_pressed_released" in {comparison.left_label, comparison.right_label}
    ]
    zero_diffs = [
        comparison
        for comparison in state.comparisons
        if "after_zero_frame" in {comparison.left_label, comparison.right_label}
    ]
    lines = [
        "# C30D Autonomous Diagnosis Report",
        "",
        "## Confirmed facts",
        *(f"- {fact}" for fact in state.confirmed_facts),
        "",
        "## Hypotheses",
        *(
            f"- {item}"
            for item in (state.hypotheses or ["No active hypothesis from current data."])
        ),
        "",
        "## Untested assumptions",
        *(
            f"- {item}"
            for item in (state.untested_assumptions or ["No untested assumption recorded."])
        ),
        "",
        "## All commands run",
        *(f"- `{command}`" for command in state.commands_run),
        "",
        "## All captures collected",
        *(
            f"- `{path}`"
            for path in (state.captures_collected or ["No new live captures collected."])
        ),
        "",
        "## Inspected artifacts",
        *(f"- `{path}`" for path in state.inspected_paths),
        "",
        "## Battery/frame-rate/checksum summaries",
        *(_format_capture(summary) for summary in state.captures),
        "",
        "## Status/mode byte candidates",
        f"- Candidate byte 21 values: {_format_values(tuple(status_candidates))}",
        "",
        "## Motor switch OFF vs ON differences",
        *render_comparison_lines(switch_diffs),
        "",
        "## USER button differences",
        *render_comparison_lines(user_diffs),
        "",
        "## Zero-frame effect",
        *render_comparison_lines(zero_diffs),
        "",
        "## All live command tests and results",
        "- This runner did not execute motion probes.",
        "- Known prior live command results are listed in confirmed facts.",
        "",
        "## Decision trace",
        *(f"- {item}" for item in (state.decision_trace or ["Offline-only analysis completed."])),
        "",
        "## Conclusion",
        state.conclusion or "No conclusion recorded.",
        "",
        "## Recommended next step",
        f"- {state.recommended_next_step or choose_recommendation(state)}",
        "",
    ]
    return "\n".join(lines)


def render_comparison_lines(comparisons: list[StateComparison]) -> list[str]:
    if not comparisons:
        return ["- No matching comparison available."]
    lines: list[str] = []
    for comparison in comparisons:
        lines.append(f"- {comparison.left_label} vs {comparison.right_label}")
        if comparison.byte_differences:
            diffs = ", ".join(
                f"byte {diff.position}: {_format_values(diff.left_values)} -> "
                f"{_format_values(diff.right_values)}"
                for diff in comparison.byte_differences[:12]
            )
            if len(comparison.byte_differences) > 12:
                diffs += f", ... {len(comparison.byte_differences) - 12} more"
            lines.append(f"  - byte differences: {diffs}")
        else:
            lines.append("  - byte differences: none")
        lines.append(
            "  - field differences: "
            + (", ".join(comparison.field_differences) if comparison.field_differences else "none")
        )
    return lines


def write_report(state: DiagnosisState) -> Path:
    state.report_dir.mkdir(parents=True, exist_ok=True)
    report_path = state.report_dir / "c30d_autonomous_diagnosis_report.md"
    report_path.write_text(render_report(state), encoding="utf-8")
    return report_path


def main(
    argv: list[str] | None = None,
    *,
    input_fn: Callable[[str], str] = input,
    serial_factory: SerialFactory = open_serial_handle,
) -> int:
    args = build_parser().parse_args(argv)
    report_dir = timestamped_report_dir()
    if args.offline_only:
        state = run_offline_analysis(report_dir)
    elif args.guided_live:
        state = run_guided_live(
            report_dir,
            port=args.port,
            baud=args.baud,
            capture_duration_s=args.capture_duration,
            input_fn=input_fn,
            serial_factory=serial_factory,
        )
    else:
        state = run_continue_until_exhausted(
            report_dir,
            port=args.port,
            baud=args.baud,
            capture_duration_s=args.capture_duration,
            non_interactive=args.non_interactive,
            input_fn=input_fn,
            serial_factory=serial_factory,
        )
    report_path = write_report(state)
    print(f"report_path: {report_path}")
    print(f"conclusion: {state.conclusion}")
    print(f"recommended_next_step: {state.recommended_next_step}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
