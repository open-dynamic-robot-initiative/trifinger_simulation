from trifinger_simulation.sim_finger import int_to_rgba


def test_int_to_rgba():
    assert int_to_rgba(0x000000) == (0.0, 0.0, 0.0, 1.0)
    assert int_to_rgba(0xFFFFFF) == (1.0, 1.0, 1.0, 1.0)
    assert int_to_rgba(0x006C66) == (0, 108 / 255, 102 / 255, 1.0)

    assert int_to_rgba(0x006C66, alpha=42) == (
        0,
        108 / 255,
        102 / 255,
        42 / 255,
    )
