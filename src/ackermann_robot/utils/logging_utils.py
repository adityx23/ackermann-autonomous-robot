from __future__ import annotations

import logging
from datetime import datetime
from pathlib import Path


def setup_logging(
    log_root: str | Path = "logs",
    *,
    console: bool = True,
    level: int = logging.INFO,
) -> Path:
    """Configure process logging and return the timestamped run folder."""
    run_folder = _create_run_folder(Path(log_root))
    log_file = run_folder / "robot.log"

    formatter = logging.Formatter(
        fmt="%(asctime)s %(levelname)s %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    root_logger = logging.getLogger()
    root_logger.setLevel(level)
    _remove_ackermann_handlers(root_logger)

    file_handler = logging.FileHandler(log_file, encoding="utf-8")
    file_handler.setFormatter(formatter)
    file_handler.setLevel(level)
    file_handler._ackermann_robot_handler = True  # type: ignore[attr-defined]
    root_logger.addHandler(file_handler)

    if console:
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        console_handler.setLevel(level)
        console_handler._ackermann_robot_handler = True  # type: ignore[attr-defined]
        root_logger.addHandler(console_handler)

    logging.getLogger(__name__).info("logging initialized in %s", run_folder)
    return run_folder


def _create_run_folder(log_root: Path) -> Path:
    log_root.mkdir(parents=True, exist_ok=True)
    base_name = datetime.now().strftime("run_%Y%m%d_%H%M%S")

    for suffix in ["", *[f"_{index:02d}" for index in range(1, 100)]]:
        run_folder = log_root / f"{base_name}{suffix}"
        try:
            run_folder.mkdir()
            return run_folder
        except FileExistsError:
            continue

    raise RuntimeError(f"could not create a unique run folder under {log_root}")


def _remove_ackermann_handlers(logger: logging.Logger) -> None:
    for handler in list(logger.handlers):
        if getattr(handler, "_ackermann_robot_handler", False):
            logger.removeHandler(handler)
            handler.close()
