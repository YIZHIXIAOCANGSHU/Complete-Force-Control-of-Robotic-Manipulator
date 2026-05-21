from __future__ import annotations


def test_identification_mode_entrypoints_import():
    import robot_control.modes.payload_id_sim.main as payload_id_sim
    import robot_control.modes.param_id_real.main as param_id_real
    import robot_control.modes.param_id_sim.main as param_id_sim

    assert callable(payload_id_sim.main)
    assert callable(param_id_sim.main)
    assert callable(param_id_real.main)
