import pytest
from irobot.db_manager import DBManager

def test_subscription_crud(db_path):
    db = DBManager(db_path)
    # no subscribers initially
    assert db.get_subscribers("topic1") == []

    # add one
    db.add_subscription("topic1", "127.0.0.1", 5555)
    subs = db.get_subscribers("topic1")
    assert subs == [("127.0.0.1", 5555)]

    # duplicate add is a no-op
    db.add_subscription("topic1", "127.0.0.1", 5555)
    assert db.get_subscribers("topic1") == [("127.0.0.1", 5555)]

    # remove it
    db.remove_subscription("topic1", "127.0.0.1", 5555)
    assert db.get_subscribers("topic1") == []

def test_message_crud(db_path):
    db = DBManager(db_path)
    # no messages initially
    assert db.get_messages("foo") == []

    # add two messages
    db.add_message("foo", "hello")
    db.add_message("foo", "world")

    msgs = db.get_messages("foo")
    # payload order must match insertion
    assert [payload for _, payload, _ in msgs] == ["hello", "world"]

def test_singleton_behavior(db_path):
    db1 = DBManager(db_path)
    db2 = DBManager(db_path)
    assert db1 is db2
