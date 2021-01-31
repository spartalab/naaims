import aimsim.util as util


def test_Coord():
    assert util.Coord(1.1, 2.2) == (1.1, 2.2)


def test_SpeedUpdate():
    assert util.SpeedUpdate(1.1, 2.2) == (1.1, 2.2)
