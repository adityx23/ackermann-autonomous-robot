import logging
import re

from ackermann_robot.utils.logging_utils import setup_logging


def test_setup_logging_creates_run_folder_and_log_file(tmp_path):
    run_folder = setup_logging(log_root=tmp_path, console=False)

    assert run_folder.parent == tmp_path
    assert re.match(r"run_\d{8}_\d{6}(?:_\d{2})?$", run_folder.name)
    assert (run_folder / "robot.log").exists()


def test_setup_logging_writes_to_run_log_file(tmp_path):
    run_folder = setup_logging(log_root=tmp_path, console=False)
    logging.getLogger("ackermann_robot.test").info("small status message")

    for handler in logging.getLogger().handlers:
        handler.flush()

    log_text = (run_folder / "robot.log").read_text(encoding="utf-8")
    assert "small status message" in log_text


def test_setup_logging_uses_new_run_folder_each_call(tmp_path):
    first = setup_logging(log_root=tmp_path, console=False)
    second = setup_logging(log_root=tmp_path, console=False)

    assert first != second
