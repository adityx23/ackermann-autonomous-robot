from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


def load_search_script():
    script_path = (
        Path(__file__).resolve().parents[1] / "scripts" / "search_c30d_protocol_references.py"
    )
    spec = importlib.util.spec_from_file_location(script_path.stem, script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[script_path.stem] = module
    spec.loader.exec_module(module)
    return module


def test_line_matches_is_case_insensitive_and_returns_keywords():
    module = load_search_script()

    matches = module.line_matches("C30D motor checksum 0x7b")

    assert matches == ["c30d", "motor", "checksum", "0x7B"]


def test_search_file_returns_keyword_matches(tmp_path: Path):
    module = load_search_script()
    path = tmp_path / "demo.py"
    path.write_text(
        "\n".join(
            [
                "controller = 'C30D'",
                "serial.write(b'example')",
                "no match here",
            ]
        ),
        encoding="utf-8",
    )

    matches = module.search_file(path)

    assert [(match.line_number, match.keyword) for match in matches] == [
        (1, "c30d"),
        (2, "serial.write"),
    ]
    assert module.format_match(matches[0]).endswith("[c30d] controller = 'C30D'")


def test_search_file_skips_binary_file(tmp_path: Path):
    module = load_search_script()
    path = tmp_path / "binary.bin"
    path.write_bytes(b"\x00c30d")

    assert module.search_file(path) == []


def test_iter_candidate_files_skips_git_and_missing_roots(tmp_path: Path):
    module = load_search_script()
    root = tmp_path / "root"
    root.mkdir()
    keep = root / "keep.txt"
    keep.write_text("c30d", encoding="utf-8")
    git_dir = root / ".git"
    git_dir.mkdir()
    ignored = git_dir / "config"
    ignored.write_text("motor", encoding="utf-8")

    files = module.iter_candidate_files((root, tmp_path / "missing"))

    assert files == [keep]


def test_search_roots_combines_matches(tmp_path: Path):
    module = load_search_script()
    first = tmp_path / "first.txt"
    second = tmp_path / "second.txt"
    first.write_text("speed\n", encoding="utf-8")
    second.write_text("servo\n", encoding="utf-8")

    matches = module.search_roots((tmp_path,))

    assert [match.keyword for match in matches] == ["speed", "servo"]


def test_default_keywords_include_wheeltec_board_search_terms():
    module = load_search_script()

    for keyword in (
        "WHEELTEC",
        "轮趣",
        "轮趣科技",
        "底层主控",
        "通信协议",
        "源码",
        "串口",
        "C10B",
        "R550",
        "STM32F407VET6",
        "ROS bottom controller",
        "cmd_vel",
        "serial protocol",
    ):
        assert keyword in module.DEFAULT_KEYWORDS
