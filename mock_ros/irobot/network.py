import socket
import threading
import logging
from .db_manager import DBManager

class NodeServer(threading.Thread):
    """
    A simple TCP server that invokes `callback(message)` on each recv().
    """
    def __init__(self, host="127.0.0.1", port=0, callback=None):
        super().__init__(daemon=True)
        self.host     = host
        self.port     = port
        self.callback = callback
        self.sock     = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.bind((self.host, self.port))
            self.sock.listen()
            self.port = self.sock.getsockname()[1]
            logging.info(f"NodeServer listening on {self.host}:{self.port}")
        except socket.error as e:
            logging.error(f"Server socket error: {e}")
            raise

    def run(self):
        while True:
            try:
                conn, addr = self.sock.accept()
                threading.Thread(target=self._handle, args=(conn,addr), daemon=True).start()
            except Exception as e:
                logging.error(f"Accept failed: {e}")

    def _handle(self, conn, addr):
        try:
            data = conn.recv(4096)
            if data:
                msg = data.decode("utf-8")
                logging.info(f"Received from {addr}: {msg}")
                if self.callback:
                    self.callback(msg)
        except Exception as e:
            logging.error(f"Handle client error: {e}")
        finally:
            conn.close()

class Publisher:
    def __init__(self, db_path="irobot.db"):
        self.db = DBManager(db_path)

    def publish(self, topic, message):
        logging.info(f"Publishing [{topic}]: {message}")
        # store in history
        self.db.add_message(topic, message)
        # send to subscribers
        for host, port in self.db.get_subscribers(topic):
            try:
                with socket.create_connection((host, port), timeout=5) as sock:
                    sock.sendall(message.encode("utf-8"))
                    logging.info(f"→ sent to {host}:{port}")
            except Exception as e:
                logging.error(f"Publish to {host}:{port} failed: {e}")

class Subscriber:
    def __init__(self, topic, callback, db_path="irobot.db"):
        self.topic    = topic
        self.callback = callback
        self.db       = DBManager(db_path)

        # start server to receive messages
        self.server = NodeServer(callback=self._on_message)
        self.server.start()

        # register ourselves
        self.db.add_subscription(self.topic, self.server.host, self.server.port)

        # replay history
        for _id, payload, ts in self.db.get_messages(self.topic):
            self.callback(payload)

    def _on_message(self, message):
        try:
            self.callback(message)
        except Exception as e:
            logging.error(f"Subscriber callback error: {e}")
