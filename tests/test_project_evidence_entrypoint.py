import json
import os
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def fake_midstream(tmp_path: Path) -> Path:
    package = tmp_path / "project_knowledge"
    package.mkdir()
    (package / "__init__.py").write_text("", encoding="utf-8")
    (package / "cli.py").write_text(
        "import json, sys\nprint(json.dumps({'args': sys.argv[1:]}))\n",
        encoding="utf-8",
    )
    return tmp_path


def run_entrypoint(name: str, arguments: list[str], midstream: Path):
    environment = {**os.environ, "EPISODE_DATA_LAB_ROOT": str(midstream), "PYTHONPATH": ""}
    return subprocess.run(
        [str(ROOT / "bin" / name), *arguments], cwd=ROOT, env=environment,
        capture_output=True, text=True, check=False,
    )


def test_project_evidence_forwards_query(tmp_path):
    result = run_entrypoint("project-evidence", ["query", "--mode", "fact", "--query", "gate"], fake_midstream(tmp_path))
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)["args"] == ["query", "--mode", "fact", "--query", "gate"]


def test_project_evidence_defaults_impact_to_upstream(tmp_path):
    result = run_entrypoint("project-evidence", ["impact", "--base", "A", "--head", "B"], fake_midstream(tmp_path))
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)["args"] == [
        "impact", "--repository", "ros2-arm-teleoperation-suite", "--base", "A", "--head", "B",
    ]


def test_ask_project_preserves_legacy_query_shape(tmp_path):
    result = run_entrypoint("ask-project", ["三仓", "职责"], fake_midstream(tmp_path))
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)["args"] == [
        "query", "--mode", "auto", "--no-llm", "--query", "三仓 职责",
    ]


def test_entrypoint_reports_missing_core(tmp_path):
    result = run_entrypoint("project-evidence", ["query", "--query", "gate"], tmp_path / "missing")
    assert result.returncode == 2
    assert "Set EPISODE_DATA_LAB_ROOT" in result.stderr
