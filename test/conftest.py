from importlib import reload

from pytest import fixture

import aimsim.shared as SHARED


@fixture(scope="session")
def read_config():
    try:
        SHARED.SETTINGS.read()
    except RuntimeError:
        pass


@fixture
def clean_config():
    reload(SHARED)
