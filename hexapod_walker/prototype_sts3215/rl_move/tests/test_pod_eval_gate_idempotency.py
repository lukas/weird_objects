"""Gate/owncfg idempotency must key on report.json, not bare dir
existence (pod_eval.py).

Bug found 08-28 (unified1-mix triage, chasing a stalled-forever
prestage): a partial/interrupted copy-back — e.g. a hand `kubectl cp`
that grabbed a still-computing remote artifact dir (videos present,
`report.json` not yet written) — left a directory that `local_out
.exists()` happily called "already on controller", writing the
prestage sentinel and releasing a verdict cycle to judge an incomplete
panel (confirmed live: two runs' gate/owncfg dirs, manually copied
mid-flight, were silently accepted as synced and the false
`_prestage.synced` sentinel got written). Every other pass in this
file (session/mixedsession/joygate) already keys on its own
completion file for exactly this reason; `core_pass_synced` brings
gate/owncfg in line and this test pins it directly.
"""
import importlib.util
import pathlib
import sys

_ORCH = pathlib.Path(__file__).resolve().parents[1] / "orchestrator"
sys.path.insert(0, str(_ORCH))  # pod_eval imports its sibling ``tracks``
_P = _ORCH / "pod_eval.py"
_spec = importlib.util.spec_from_file_location("pod_eval", _P)
pod_eval = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(pod_eval)

core_pass_synced = pod_eval.core_pass_synced


def test_missing_dir_not_synced(tmp_path):
    assert core_pass_synced(tmp_path / "nope_gate") is False


def test_partial_copy_no_report_json_not_synced(tmp_path):
    # the exact shape a mid-flight `kubectl cp` leaves: videos/pngs
    # present, report.json not written yet because the remote eval
    # hasn't finished.
    d = tmp_path / "run_gate"
    d.mkdir()
    (d / "walk_sto_5.mp4").write_bytes(b"\x00")
    (d / "walk_sto_5.png").write_bytes(b"\x00")
    assert core_pass_synced(d) is False


def test_complete_pass_with_report_json_is_synced(tmp_path):
    d = tmp_path / "run_gate"
    d.mkdir()
    (d / "report.json").write_text("{}")
    assert core_pass_synced(d) is True
