import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys
import os

sys.path.append(os.path.dirname(__file__))
from detection_pipeline import *


class ObjectDetector(Node):

    def __init__(self):
        super().__init__("object_detector")

        self.pipeline = ObjectDetectionPipeline()
        self.detections = self.create_publisher(String, "/detections", 10)
        self.pipeline_start_service = self.create_service(
            Empty, "start_pipeline", self.start_pipeline
        )
        self.pipeline_stop_service = self.create_service(
            Empty, "stop_pipeline", self.stop_pipeline
        )
        self.input_images = self.create_subscription(
            Image, "/merged/rgbd_image", self.on_new_frame, 10
        )
        self.bridge = CvBridge()
        self.pipeline.start()

    def on_new_frame(self, msg):

        # Convert ROS Image message to OpenCV (NumPy array)
        color_frame = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="bgr8"
        )  # Use 'rgb8' or other encoding if necessary
        depth_frame = None
        self.get_logger().info(
            f"Received image with shape: {color_frame.shape} and dtype: {color_frame.dtype}"
        )
        # self.get_logger().info(f"Received depth with shape: {depth_frame.shape} and dtype: {depth_frame.dtype} ")

        # feeding the pipeline with the images
        source = self.pipeline.get_by_name("rs-source")

        gst_buffer = Gst.Buffer.new_allocate(None, color_frame.nbytes, None)
        gst_buffer.fill(0, color_frame.tobytes())

        with self.pipeline.depth_lock:
            self.pipeline.depth_buffer = np.asanyarray(depth_frame)

        # Push buffer into the pipeline
        source.emit("push-buffer", gst_buffer)

    def start_pipeline(self, request, response):
        # start the pipeline here
        self.pipeline.start()
        self.get_logger().info("Starting object detection pipeline")
        return response

    def stop_pipeline(self, request, response):
        # start the pipeline here
        self.pipeline.stop()
        self.get_logger().info("Stopping object detection pipeline")
        return response


def main(args=None):

    rclpy.init(args=args)
    object_detector = ObjectDetector()
    rclpy.spin(object_detector)
    object_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
