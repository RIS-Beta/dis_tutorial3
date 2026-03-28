#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped, Vector3Stamped

import tf2_geometry_msgs as tfg
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer, Time
from tf2_ros.transform_listener import TransformListener

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

from rclpy.duration import Duration

class detect_rings(Node):

    def __init__(self):
        super().__init__('detect_rings')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('device', ''),
        ])

        marker_topic = "/rings_marker"

        self.detection_color = (0,255,0)
        self.device = self.get_parameter('device').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scan = None
        self.new_marker_id = 0

        self.min_depth = 0.2  # Minimalna globina (m)
        self.max_depth = 3.0  # Maksimalna globina (m)

        #thersholdings
        self.ration_thr = 1.5
        self.center_thr = 5.0 # Malo povečano za boljšo toleranco v simulatorju

        #self.rgb_image_sub = self.create_subscription(Image, "/oakd/rgb/preview/image_raw", self.rgb_callback, qos_profile_sensor_data)
        self.depth_image_sub = self.create_subscription(Image, "/oakd/rgb/preview/depth", self.depth_callback, qos_profile_sensor_data)
        self.pointcloud_sub = self.create_subscription(PointCloud2, "/oakd/rgb/preview/depth/points", self.pointcloud_callback, qos_profile_sensor_data)

        self.marker_pub = self.create_publisher(Marker, marker_topic, qos_profile_sensor_data)

        self.rings = []
        self.markers = []

        self.get_logger().info(f"Node has been initialized! Will publish ring markers to {marker_topic}.")


    def depth_callback(self, data):
        try:
            depth = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

            #removing nan and inf values
            depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)

            #mask to ignore zero values (invalid depth)
            mask = (depth > 0) & (depth < self.max_depth) 

            #if depth image exists, we will normalize it for better visualization
            if np.any(mask):
                depth_norm = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
                depth_uint8 = depth_norm.astype(np.uint8)
                depth_uint8[~mask] = 0  # SEt invalid depth to black just in case
                blurred_depth = cv2.GaussianBlur(depth_uint8, (9, 9), 2)
                circles = cv2.HoughCircles(
                    blurred_depth,
                    cv2.HOUGH_GRADIENT,
                    dp=1.2,
                    minDist=50,
                    param1=100,
                    param2=35,     # accrurace of detection, higher = more strict
                    minRadius=4,
                    maxRadius=50,  # Limit to avoid detecting very large objects
                )

                depth_color = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_JET)

                if circles is not None:
                    circles = np.round(circles[0, :]).astype(int)
                    for x, y, radius in circles:
                        self.get_logger().info(f"Ring detected (Hough): {x}, {y}")
                        cv2.circle(depth_color, (x, y), radius, (0, 255, 0), 2)
                        cv2.circle(depth_color, (x, y) , 5, (0, 0, 255), -1)
                        self.rings.append((x, y, radius))


            else:
                # invalid image, just show it as black
                depth_norm = np.zeros_like(depth)
                depth_color = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_JET)

            cv2.imshow("Depth", depth_color)    
            cv2.waitKey(1)

        except Exception as e:
            print(f"Depth callback error: {e}")

    def pointcloud_callback(self, data):
        if not self.rings:
            return

        try:
            a = pc2.read_points_numpy(data, field_names=("x", "y", "z"))
            a = a.reshape((data.height, data.width, 3))
        

            for x, y, radius in self.rings:
                if x >= data.width or y >= data.height or x < 0 or y < 0:
                    continue
               
                mask = np.zeros((data.height, data.width), dtype=np.uint8)
                cv2.circle(mask, (x, y), radius, 1, thickness=2)
                cv2.circle(mask, (x, y), radius, 1, thickness=-2)
                valid_points = a[mask==1]
                valid_points = valid_points[~np.isnan(valid_points).any(axis=1)]
                valid_points = valid_points[~np.isinf(valid_points).any(axis=1)]

                if valid_points.shape[0] == 0:
                    continue

                # avrage the valid points to get a more stable position of the ring
                center_point = np.mean(valid_points, axis=0)

                self.get_logger().info(f"Center point: {center_point}")

                if center_point[2] < self.min_depth or center_point[2] > self.max_depth:
                    continue

                #calulating normal vector of the ring plane
                normal = [-float(center_point[0]), -float(center_point[1]), float(center_point[2])]
                normal_length = np.linalg.norm(normal)
                if normal_length == 0:
                    continue
                normal = normal / normal_length 
                
                #creating point
                point_in_robot_frame = PointStamped()
                point_in_robot_frame.header = data.header # original header
                point_in_robot_frame.point.x = float(center_point[0])
                point_in_robot_frame.point.y = float(center_point[1])
                point_in_robot_frame.point.z = float(center_point[2])


                norm_vec = Vector3Stamped()
                norm_vec.header = data.header
                norm_vec.vector.x = float(normal[0])
                norm_vec.vector.y = float(normal[1])
                norm_vec.vector.z = float(normal[2])

                time_now = rclpy.time.Time()
                timeout = Duration(seconds=0.2)
                try:

                    trans = self.tf_buffer.lookup_transform("map", data.header.frame_id, data.header.stamp, timeout)
                    point_in_map_frame = tfg.do_transform_point(point_in_robot_frame, trans)
                    normal_in_map = self.tf_buffer.transform(norm_vec, "map")

                    # creating marker on map
                    marker_in_map_frame = self.create_marker(point_in_map_frame, self.new_marker_id, 0.0)
                    if marker_in_map_frame.pose.position.z > 3.0 or marker_in_map_frame.pose.position.z < 0.2:
                        self.get_logger().info("Marker preskočen (hardcoded filter)")
                        continue

                    p0 = Point()
                    p0.x = point_in_map_frame.point.x
                    p0.y = point_in_map_frame.point.y
                    p0.z = point_in_map_frame.point.z

                    #0.5 long vector it is just for better represention in rviz
                    p1 = Point()
                    p1.x = normal_in_map.vector.x * 0.5 + p0.x
                    p1.y = normal_in_map.vector.y * 0.5 + p0.y
                    p1.z = normal_in_map.vector.z * 0.5 + p0.z

                    marker_in_map_frame.points = [p0, p1]  # Set the points for the line marker
                    marker_in_map_frame.type = Marker.SPHERE  # Change the marker type to ARROW

                    # Objavi marker
                    self.marker_pub.publish(marker_in_map_frame)
                    self.new_marker_id += 1
                    
                    self.markers.append(marker_in_map_frame)
                    

                    self.get_logger().info(f"Ring marker objavljen na: {point_in_map_frame.point.x:.2f}, {point_in_map_frame.point.y:.2f}, {point_in_map_frame.point.z:.2f}")

                except TransformException as te:
                    self.get_logger().warn(f"Could not get transform: {te}")

        except Exception as e:
            self.get_logger().error(f"Pointcloud error: {e}")

        # Počisti detekcije po obdelavi
        self.rings = []

    def create_marker(self, point_stamped, marker_id, lifetime=30.0):
        marker = Marker()
        marker.header = point_stamped.header
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.id = marker_id

        scale = 0.15
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.pose.position.x = point_stamped.point.x
        marker.pose.position.y = point_stamped.point.y
        marker.pose.position.z = point_stamped.point.z

        marker.lifetime = Duration(seconds=lifetime).to_msg()
        return marker

def main():
    rclpy.init(args=None)
    node = detect_rings()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()