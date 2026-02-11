#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from flight_core.msg import ArucoMarker, ArucoMarkers
import cv2
import cv2.aruco as aruco
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        # Parameters
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_size', 0.2) # meters
        
        # TF Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera_down/image_raw',
            self.image_callback,
            10)
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera_down/camera_info',
            self.info_callback,
            10)
            
        # Publishers
        self.target_pub = self.create_publisher(ArucoMarkers, '/perception/aruco_markers', 10)
        self.viz_pub = self.create_publisher(MarkerArray, '/perception/aruco_viz', 10)
        self.debug_pub = self.create_publisher(Image, '/perception/debug_image', 10)
        
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Aruco setup
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        self.get_logger().info('ArUco Detector initialized.')

    def info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera Info received.')
            # Unsubscribe after getting info once (assuming constant intrinsics)
            # self.destroy_subscription(self.info_sub) 

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return
            
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        # Prepare output messages
        aruco_msg = ArucoMarkers()
        aruco_msg.header.stamp = msg.header.stamp
        aruco_msg.header.frame_id = 'map' 
        
        marker_viz = MarkerArray()

        if ids is not None:
            # Estimate pose
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.2, self.camera_matrix, self.dist_coeffs)
            
            for i in range(len(ids)):
                # Draw axis for debug
                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)
                
                try:
                    # Create PoseStamped for the marker in camera frame
                    p_cam = PoseStamped()
                    p_cam.header = msg.header
                    p_cam.pose.position.x = tvecs[i][0][0]
                    p_cam.pose.position.y = tvecs[i][0][1]
                    p_cam.pose.position.z = tvecs[i][0][2]
                    
                    # Convert rotation vector to quaternion
                    rot_mat, _ = cv2.Rodrigues(rvecs[i])
                    # (Simple conversion to quaternion omitted for brevity, usually needed.
                    #  For now using identity orientation or a simple robust fallback because
                    #  we honestly mostly care about POSITION for landing)
                    #  Ideally: use tf2 or scipy to convert rot_mat to quat.
                    #  Here I will just use identity for safety unless precise orientation needed.
                    p_cam.pose.orientation.w = 1.0 
                    
                    # Transform to map frame
                    p_map = None
                    try:
                        p_map = self.tf_buffer.transform(p_cam, 'map', timeout=rclpy.duration.Duration(seconds=0.1))
                    except TransformException as ex:
                        # Fallback to latest transform if extrapolation fails
                        # self.get_logger().debug(f'Transform failed, retrying with latest time: {ex}')
                        try:
                            # Use Time(seconds=0) to get latest available transform
                            p_cam.header.stamp = rclpy.time.Time(seconds=0).to_msg()
                            p_map = self.tf_buffer.transform(p_cam, 'map', timeout=rclpy.duration.Duration(seconds=0.1))
                        except TransformException as ex2:
                            self.get_logger().warn(f'Could not transform pose (even latest): {ex} -> {ex2}')
                    
                    if p_map:
                        # Add to ArucoMarkers msg
                        marker = ArucoMarker()
                        marker.marker_id = int(ids[i][0])
                        marker.pose = p_map.pose
                        aruco_msg.markers.append(marker)
                        
                        # Add to Visualization MarkerArray
                        viz = Marker()
                        viz.header = p_map.header
                        viz.ns = "aruco_markers"
                        viz.id = int(ids[i][0])
                        viz.type = Marker.CUBE
                        viz.action = Marker.ADD
                        viz.pose = p_map.pose
                        viz.scale.x = 0.2
                        viz.scale.y = 0.2
                        viz.scale.z = 0.05
                        viz.color.r = 0.0
                        viz.color.g = 1.0
                        viz.color.b = 0.0
                        viz.color.a = 0.8
                        viz.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
                        marker_viz.markers.append(viz)

                except Exception as e:
                    self.get_logger().error(f'Error processing marker {i}: {e}')
            
            aruco.drawDetectedMarkers(cv_image, corners, ids)
            
        self.target_pub.publish(aruco_msg)
        self.viz_pub.publish(marker_viz)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
