import pytest
from irobot.db_manager import DBManager

@pytest.fixture
def db_path(tmp_path):
    """
    Provides a fresh SQLite file path and resets the DBManager singleton
    before and after each test that uses it.
    """
    # teardown old instance if exists
    inst = DBManager._instance
    if inst:
        try:
            inst.conn.close()
        except:
            pass
    DBManager._instance = None

    path = tmp_path / "test.db"
    yield str(path)

    # teardown again
    inst = DBManager._instance
    if inst:
        try:
            inst.conn.close()
        except:
            pass
    DBManager._instance = None
