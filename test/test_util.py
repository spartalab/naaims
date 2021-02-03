from aimsim.util import *


def test_Coord():
    assert Coord(1.1, 2.2) == (1.1, 2.2)


def test_SpeedUpdate():
    assert SpeedUpdate(1.1, 2.2) == (1.1, 2.2)
