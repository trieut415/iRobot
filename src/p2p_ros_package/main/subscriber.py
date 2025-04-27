import os
import rclpy
from std_msgs.msg import String
from p2p_ros_package.common.networking import P2PNode
from p2p_ros_package.common.db import init_db, create_message


def main(args=None):
    rclpy.init(args=args)
    node = P2PNode('subscriber_node')

    os.makedirs('data', exist_ok=True)
    conn = init_db()

    def callback(msg: String):
        node.get_logger().info(f"Received: '{msg.data}'")
        create_message(conn, 'chatter', msg.data)

    node.create_subscription(String, 'chatter', callback, qos_profile=10)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        conn.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()