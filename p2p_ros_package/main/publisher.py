import os
import rclpy
from std_msgs.msg import String
from p2p_ros_package.common.networking import P2PNode
from p2p_ros_package.common.db import init_db, create_message


def main(args=None):
    rclpy.init(args=args)
    node = P2PNode('publisher_node')

    # Ensure data directory exists
    os.makedirs('data', exist_ok=True)
    conn = init_db()

    pub = node.create_publisher(String, 'chatter', qos_profile=10)

    def timer_callback():
        msg = String()
        msg.data = f"Hello ROS2 at {node.get_clock().now().to_msg().sec}"
        node.get_logger().info(f"Publishing: '{msg.data}'")
        pub.publish(msg)
        create_message(conn, 'chatter', msg.data)

    node.create_timer(1.0, timer_callback)

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