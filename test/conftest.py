from pytest import fixture

import aimsim.shared as SHARED


@fixture(scope="session")
def read_shared():
    try:
        SHARED.SETTINGS.read()
    except RuntimeError:
        pass
