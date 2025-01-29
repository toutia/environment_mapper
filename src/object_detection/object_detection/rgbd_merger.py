import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rtabmap_msgs.msg import RGBDImages
from cv_bridge import CvBridge
import cv2
import numpy as np



class RGBDImageMerger(Node):
    def __init__(self):
        super().__init__('rgbd_image_merger')

        self.bridge = CvBridge()

        # Subscribers
        self.image1_sub = self.create_subscription(
            RGBDImages, '/cameras/rgbd_images', self.images_callback, 10
        )
        # Publisher
        self.merged_image_pub = self.create_publisher(Image, '/merged/rgbd_image', 10)


    def images_callback(self, msg):
        try:
            self.get_logger().info('ok')
            color_image1 = self.bridge.imgmsg_to_cv2(msg.rgbd_images[0].rgb, desired_encoding='bgr8')
            color_image2 = self.bridge.imgmsg_to_cv2(msg.rgbd_images[1].rgb, desired_encoding='bgr8')
             # # # Debugging: Validate combined frame

               




            if  color_image1 is None  or  color_image2 is None  :
                return


            # Apply a slight rotation to color_frame2 and depth_frame2
            h, w, _ = color_image2.shape
            rotation_matrix = cv2.getRotationMatrix2D((w // 2, h // 2), angle=10, scale=1.0)
                
            # Rotate the second color frame with proper interpolation
            rotated_color2 = cv2.warpAffine(
                color_image2,
                rotation_matrix,
                (w, h),
                flags=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue=(0, 0, 0)  # Black border
            )

            color_image1_rgb = cv2.cvtColor(color_image1, cv2.COLOR_BGR2RGB)
            color_image2_rgb = cv2.cvtColor(color_image2, cv2.COLOR_BGR2RGB)
            rotated_color2_rgb = cv2.cvtColor(rotated_color2, cv2.COLOR_BGR2RGB)



            # Combine the color frames vertically
            combined_color_frame = np.vstack((color_image1_rgb, color_image2_rgb))

            merged_frame = self.bridge.cv2_to_imgmsg(combined_color_frame, encoding="bgr8")


            self.merged_image_pub.publish( merged_frame )

         
            



     
        except Exception as e:
            self.get_logger().error(f"Error processing images: {e}")





def main(args=None):
    rclpy.init(args=args)
    node = RGBDImageMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
