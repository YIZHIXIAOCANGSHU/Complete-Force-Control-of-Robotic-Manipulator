from __future__ import annotations

import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))


def test_four_mode_entrypoints_import():
    import modes.control_real.main as control_real
    import modes.control_sim.main as control_sim
    import modes.param_id_real.main as param_id_real
    import modes.param_id_sim.main as param_id_sim

    assert callable(control_sim.main)
    assert callable(control_real.main)
    assert callable(param_id_sim.main)
    assert callable(param_id_real.main)
