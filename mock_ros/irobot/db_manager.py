import sqlite3
import threading
import logging

class DBManager:
    """
    Thread-safe singleton for SQLite CRUD.
    Tables:
      - subscribers(topic, host, port)
      - messages(id, topic, payload, timestamp)
    """
    _instance = None
    _lock = threading.Lock()

    def __new__(cls, db_path="irobot.db"):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
                cls._instance._initialized = False
        return cls._instance

    def __init__(self, db_path="irobot.db"):
        if getattr(self, "_initialized", False):
            return
        self.db_path = db_path
        try:
            self.conn = sqlite3.connect(self.db_path, check_same_thread=False)
            self.cur  = self.conn.cursor()
            self._init_tables()
            self._initialized = True
        except sqlite3.Error as e:
            logging.error(f"DB connection failed: {e}")
            raise

    def _init_tables(self):
        try:
            self.cur.execute("""
                CREATE TABLE IF NOT EXISTS subscribers (
                  topic TEXT NOT NULL,
                  host  TEXT NOT NULL,
                  port  INTEGER NOT NULL,
                  PRIMARY KEY(topic, host, port)
                )""")
            self.cur.execute("""
                CREATE TABLE IF NOT EXISTS messages (
                  id        INTEGER PRIMARY KEY AUTOINCREMENT,
                  topic     TEXT NOT NULL,
                  payload   TEXT NOT NULL,
                  timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
                )""")
            self.conn.commit()
        except sqlite3.Error as e:
            logging.error(f"Failed to init tables: {e}")
            raise

    def add_subscription(self, topic, host, port):
        try:
            self.cur.execute(
                "INSERT OR IGNORE INTO subscribers (topic, host, port) VALUES (?, ?, ?)",
                (topic, host, port)
            )
            self.conn.commit()
            logging.info(f"Subscribed {host}:{port} to '{topic}'")
        except sqlite3.Error as e:
            logging.error(f"add_subscription error: {e}")

    def remove_subscription(self, topic, host, port):
        try:
            self.cur.execute(
                "DELETE FROM subscribers WHERE topic=? AND host=? AND port=?",
                (topic, host, port)
            )
            self.conn.commit()
            logging.info(f"Unsubscribed {host}:{port} from '{topic}'")
        except sqlite3.Error as e:
            logging.error(f"remove_subscription error: {e}")

    def get_subscribers(self, topic):
        try:
            self.cur.execute(
                "SELECT host, port FROM subscribers WHERE topic=?",
                (topic,)
            )
            return self.cur.fetchall()
        except sqlite3.Error as e:
            logging.error(f"get_subscribers error: {e}")
            return []

    def add_message(self, topic, payload):
        try:
            self.cur.execute(
                "INSERT INTO messages (topic, payload) VALUES (?, ?)",
                (topic, payload)
            )
            self.conn.commit()
            logging.info(f"Stored message on '{topic}': {payload}")
        except sqlite3.Error as e:
            logging.error(f"add_message error: {e}")

    def get_messages(self, topic):
        try:
            self.cur.execute(
                "SELECT id, payload, timestamp FROM messages WHERE topic=? ORDER BY id",
                (topic,)
            )
            return self.cur.fetchall()
        except sqlite3.Error as e:
            logging.error(f"get_messages error: {e}")
            return []
