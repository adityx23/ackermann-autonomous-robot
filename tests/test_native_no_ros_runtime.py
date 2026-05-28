from __future__ import annotations

import ast
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOTS = (REPO_ROOT / "src", REPO_ROOT / "scripts")
FORBIDDEN_IMPORTS = {"rospy", "rclpy"}
FORBIDDEN_PRODUCTION_COMMANDS = {"roslaunch", "rostopic", "ros2"}


def iter_python_source_files() -> list[Path]:
    files: list[Path] = []
    for root in SOURCE_ROOTS:
        files.extend(path for path in root.rglob("*.py") if path.is_file())
    return sorted(files)


def dotted_name(node: ast.AST) -> str | None:
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        parent = dotted_name(node.value)
        if parent is None:
            return node.attr
        return f"{parent}.{node.attr}"
    return None


def literal_strings(node: ast.AST) -> list[str]:
    strings: list[str] = []
    for child in ast.walk(node):
        if isinstance(child, ast.Constant) and isinstance(child.value, str):
            strings.append(child.value)
    return strings


def test_no_source_file_imports_rospy_or_rclpy():
    offenders: list[str] = []
    for path in iter_python_source_files():
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    root_name = alias.name.split(".")[0]
                    if root_name in FORBIDDEN_IMPORTS:
                        offenders.append(f"{path}:{alias.name}")
            elif isinstance(node, ast.ImportFrom) and node.module is not None:
                root_name = node.module.split(".")[0]
                if root_name in FORBIDDEN_IMPORTS:
                    offenders.append(f"{path}:{node.module}")

    assert offenders == []


def test_no_production_script_calls_roslaunch_rostopic_or_ros2():
    offenders: list[str] = []
    for path in sorted((REPO_ROOT / "scripts").rglob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            function_name = dotted_name(node.func) or ""
            if function_name in FORBIDDEN_PRODUCTION_COMMANDS:
                offenders.append(f"{path}:{function_name}")
            for value in literal_strings(node):
                first_token = value.split()[0] if value.split() else ""
                if first_token in FORBIDDEN_PRODUCTION_COMMANDS:
                    offenders.append(f"{path}:{value}")

    assert offenders == []
