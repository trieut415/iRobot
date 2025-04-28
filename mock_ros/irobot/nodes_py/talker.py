import time
import argparse
import logging
from irobot.network import Publisher

def main(cli_args=None):
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")
    p = argparse.ArgumentParser(prog="talker")
    p.add_argument("--topic",   default="chatter", help="Topic name")
    p.add_argument("--message", default="Hello, ROS!", help="Payload")
    p.add_argument("--rate",    type=float, default=1.0, help="Hz publish rate")
    args = p.parse_args(cli_args)

    pub = Publisher()
    interval = 1.0 / args.rate if args.rate > 0 else 1.0

    try:
        while True:
            pub.publish(args.topic, args.message)
            time.sleep(interval)
    except KeyboardInterrupt:
        logging.info("Talker shutdown.")
