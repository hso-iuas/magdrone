import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
import scipy
from scipy.spatial.transform import Rotation as R
import cv2
import cv_bridge
import importlib.metadata
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import sys
import numpy as np
from tf2_ros import TransformBroadcaster
import time

# Node to process camera feed
class CameraProcessorMarker(Node):

    def __init__(self):
        super().__init__('camera_processor_node')
        self.declare_parameter('aruco_marker_side_length', 0.)

        # Get the parameter value from launch file
        self.aruco_marker_side_length = self.get_parameter(
            'aruco_marker_side_length').get_parameter_value().double_value

        self.get_logger().info(f"Marker Side Length: {self.aruco_marker_side_length}")

        try:
            cv_bridge_ver = importlib.metadata.version('cv-bridge')
        except importlib.metadata.PackageNotFoundError:
            cv_bridge_ver = "Unknown (Package not found in metadata)"

        print(f"cv-bridge:             {cv_bridge_ver}")
        print(sys.path)
        print(f"scipy:                 {scipy.__version__}")
        print(f"opencv-python:         {cv2.__version__}")

        self.bridge = CvBridge()
        self.subscription_rect_raw = self.create_subscription(
            Image,
            '/camera/camera/infra1/image_rect_raw',
            self.listener_rect_raw,
            10)

        self.subscription_cma_info = self.create_subscription(
            CameraInfo,
            'camera/camera/infra1/camera_info',
            self.listener_info,
            10)

        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.compressed_img_pub_ = self.create_publisher(
            CompressedImage,
            '/magdrone/camera/processed_image/compressed',
            qos_policy)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.cam_info = None

    def listener_rect_raw(self, msg):
        cv_ir = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        if self.cam_info:
            self.processIR(msg, cv_ir, self.cam_info)

    def listener_info(self, msg):
        self.cam_info = msg

    def processIR(self, original_msg, cv_ir, ir_info):
        transforms_list = []

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

        # Detect the markers
        corners, ids, rejected = detector.detectMarkers(cv_ir)

        D = np.array(ir_info.d)
        K = np.array(ir_info.k).reshape(3, 3)

        # Convert to BGR for visualization
        cv_viz = cv2.cvtColor(cv_ir, cv2.COLOR_GRAY2BGR)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(cv_viz, corners, ids)

            rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.aruco_marker_side_length,
                K,
                D)

            for i in range(len(ids)):
                t = TransformStamped()

                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'camera_optical_frame'
                t.child_frame_id = 'marker_' + str(ids[i][0])

                t.transform.translation.x = tvecs[i][0][0]
                t.transform.translation.y = tvecs[i][0][1]
                t.transform.translation.z = tvecs[i][0][2]

                rotation = R.from_rotvec(rvecs[i][0])
                quat = rotation.as_quat()
                t.transform.rotation.x = quat[0]
                t.transform.rotation.y = quat[1]
                t.transform.rotation.z = quat[2]
                t.transform.rotation.w = quat[3]
                transforms_list.append(t)

                # Draw on visualization image
                cv2.drawFrameAxes(
                    cv_viz,
                    K,
                    D,
                    rvecs[i],
                    tvecs[i],
                    0.05
                )

            #publish all found markers
            for curr in transforms_list:
                self.tf_broadcaster.sendTransform(curr)

        # Publish compressed visualization image
        msg_out = CompressedImage()
        msg_out.header.stamp = original_msg.header.stamp
        msg_out.header.frame_id = original_msg.header.frame_id
        msg_out.format = "jpeg"
        ret, buffer = cv2.imencode('.jpg', cv_viz, [int(cv2.IMWRITE_JPEG_QUALITY), 10])
        msg_out.data = np.array(buffer).tobytes()
        self.compressed_img_pub_.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    camera_show = CameraProcessorMarker()
    rclpy.spin(camera_show)
    camera_show.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()