from __future__ import annotations


def test_four_mode_entrypoints_import():
    import robot_control.modes.control_real.main as control_real
    import robot_control.modes.control_sim.main as control_sim
    import robot_control.modes.param_id_real.main as param_id_real
    import robot_control.modes.param_id_sim.main as param_id_sim

    assert callable(control_sim.main)
    assert callable(control_real.main)
    assert callable(param_id_sim.main)
    assert callable(param_id_real.main)
