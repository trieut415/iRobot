import socket
import time
import threading
import pytest

from irobot.network import NodeServer, Publisher, Subscriber
from irobot.db_manager import DBManager

def test_nodeserver_receives_message():
    received = []
    event = threading.Event()

    def cb(msg):
        received.append(msg)
        event.set()

    server = NodeServer(host="127.0.0.1", port=0, callback=cb)
    server.start()

    # give the thread a moment to spin up
    time.sleep(0.1)

    with socket.create_connection((server.host, server.port), timeout=1) as sock:
        sock.sendall(b"ping")

    # wait for our callback
    assert event.wait(timeout=1), "Server callback never fired"
    assert received == ["ping"]

def test_publisher_no_subscribers(db_path, caplog):
    pub = Publisher(db_path=db_path)
    caplog.set_level("INFO")

    # should not crash even if nobody is subscribed
    pub.publish("nobody", "nothing")

    # must at least store in history
    assert "Stored message on 'nobody': nothing" in caplog.text

def test_pub_sub_full_flow(db_path):
    # manual DBManager so we can pre-seed history
    db = DBManager(db_path)
    db.add_message("chat", "old1")
    db.add_message("chat", "old2")

    # collect both history and new
    received = []
    new_event = threading.Event()

    def cb(msg):
        received.append(msg)
        # flag only on new ones
        if msg == "brand-new":
            new_event.set()

    # subscriber will start a server & replay history
    sub = Subscriber("chat", cb, db_path=db_path)
    # give replay a moment
    time.sleep(0.1)
    assert received == ["old1", "old2"]

    # now publish
    pub = Publisher(db_path=db_path)
    pub.publish("chat", "brand-new")

    # wait for live message
    assert new_event.wait(timeout=1), "Did not receive live publish"
    assert received[-1] == "brand-new"
