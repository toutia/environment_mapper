import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from rtabmap_ros.srv import AddLabel

class ObjectLabelingNode(Node):
    def __init__(self):
        super().__init__('object_labeling_node')
        self.object_sub = self.create_subscription(
            Detection2DArray,
            '/detections',  # Replace with the actual topic name
            self.object_callback,
            10)
        
        self.label_client = self.create_client(AddLabel, '/rtabmap/add_label')
        while not self.label_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /rtabmap/add_label service...')
        
        self.pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.get_logger().info("Object Labeling Node Initialized")

    def object_callback(self, msg):
        for detection in msg.detections:
            # Extract object information
            label = f"Object_{int(detection.results[0].id)}"
            x = detection.bbox.center.x
            y = detection.bbox.center.y

            # Add label in RTAB-Map
            self.add_label_to_map(label, x, y)

            # Publish as a goal for path planning
            self.publish_goal(label, x, y)

    def add_label_to_map(self, label, x, y):
        request = AddLabel.Request()
        request.node_label = label
        request.node_id = 0  # Use 0 for the closest node in RTAB-Map

        future = self.label_client.call_async(request)
        future.add_done_callback(
            lambda future: self.get_logger().info(
                f"Labeled object as '{label}'" if future.result() else f"Failed to label '{label}'"
            ))

    def publish_goal(self, label, x, y):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = 1.0  # Default orientation

        self.pose_pub.publish(goal_pose)
        self.get_logger().info(f"Published goal for '{label}' at ({x}, {y})")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectLabelingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
