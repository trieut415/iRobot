import argparse
import logging
import threading
from irobot.network import Subscriber

def main(cli_args=None):
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")
    p = argparse.ArgumentParser(prog="listener")
    p.add_argument("--topic", default="chatter", help="Topic to subscribe")
    args = p.parse_args(cli_args)

    def callback(msg):
        print(f"[{args.topic}] {msg}")

    sub = Subscriber(args.topic, callback)

    # block forever until Ctrl-C
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        logging.info("Listener shutdown.")
