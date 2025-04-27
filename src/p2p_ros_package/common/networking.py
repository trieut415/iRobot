import rclpy
from rclpy.node import Node

class P2PNode(Node):
    def __init__(self, name: str):
        super().__init__(name)
        self.publishers = {}
        self.subscriptions = {}

    def create_publisher(self, msg_type, topic: str, qos_profile=10):
        pub = super().create_publisher(msg_type, topic, qos_profile)
        self.publishers[topic] = pub
        self.get_logger().info(f"[Networking] Publisher created on topic: '{topic}'")
        return pub

    def create_subscription(self, msg_type, topic: str, callback, qos_profile=10):
        sub = super().create_subscription(msg_type, topic, callback, qos_profile)
        self.subscriptions[topic] = sub
        self.get_logger().info(f"[Networking] Subscribed to topic: '{topic}'")
        return sub