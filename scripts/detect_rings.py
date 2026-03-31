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

        self.rgb_image_sub = self.create_subscription(Image, "/oakd/rgb/preview/image_raw", self.rgb_callback, qos_profile_sensor_data)
        self.depth_image_sub = self.create_subscription(Image, "/oakd/rgb/preview/depth", self.depth_callback, qos_profile_sensor_data)
        self.pointcloud_sub = self.create_subscription(PointCloud2, "/oakd/rgb/preview/depth/points", self.pointcloud_callback, qos_profile_sensor_data)

        self.marker_pub = self.create_publisher(Marker, marker_topic, qos_profile_sensor_data)

        self.rings = []
        self.markers = []

        self.last_rgb_image = None
        self.get_logger().info(f"Node has been initialized! Will publish ring markers to {marker_topic}.")

    def rgb_callback(self, data):
        try:
            self.last_rgb_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
            # For ring detection, we will rely more on depth data, so we won't do much processing here.
            # However, you can add any color-based filtering or preprocessing if needed.

        except CvBridgeError as e:
            print(f"RGB callback error: {e}")


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
                    maxRadius=20,  # Limit to avoid detecting very large objects
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
                cv2.circle(mask, (x, y), radius, 1, thickness=5)
                cv2.circle(mask, (x, y), radius, 1, thickness=-5)
                valid_points = a[mask==1]
                valid_points = valid_points[~np.isnan(valid_points).any(axis=1)]
                valid_points = valid_points[~np.isinf(valid_points).any(axis=1)]

                if valid_points.shape[0] == 0:
                    continue

                # avrage the valid points to get a more stable position of the ring
                center_point = np.mean(valid_points, axis=0)
                depth = valid_points[:, 2]
                closest_depth = np.min(depth)

                self.get_logger().info(f"Center point: {center_point}")

                if center_point[2] < self.min_depth or center_point[2] > self.max_depth:
                    continue


                #TODO: GET colour from hsv and then get the rgb it is better (wont be bland and will be more accurate) but for now we will just get the average color from the rgb image in the area of the ring and then get the closest color name from that
                #calculating the color of the ring
                if self.last_rgb_image is not None:
                    ring_color = self.last_rgb_image[mask==1]

                    if len(ring_color) > 0:
                        avg_color = np.mean(ring_color, axis=0)
                        avg_rgb_color = avg_color[::-1]  # Convert BGR to RGB for color naming
                        color_name = self.get_color_name(avg_rgb_color)  # Convert BGR to RGB for color naming
                        self.get_logger().info(f"Ring color: {color_name}")
                    else:
                        self.get_logger().info("Could not determine ring color (no valid pixels in RGB image)")
                else:
                    self.get_logger().info("No RGB image available for color detection")

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
                #detpth is calulated
                point_in_robot_frame.point.z = float(closest_depth)


                norm_vec = Vector3Stamped()
                norm_vec.header = data.header
                norm_vec.vector.x = float(normal[0])
                norm_vec.vector.y = float(normal[1])
                norm_vec.vector.z = float(normal[2])

                time_now = rclpy.time.Time()
                timeout = Duration(seconds=0.1)
                try:

                    trans = self.tf_buffer.lookup_transform("map", data.header.frame_id, time_now, timeout)
                    point_in_map_frame = tfg.do_transform_point(point_in_robot_frame, trans)
                    normal_in_map = self.tf_buffer.transform(norm_vec, "map")

                    # creating marker on map
                    marker_in_map_frame = self.create_marker(point_in_map_frame, self.new_marker_id, 0.0, color=avg_rgb_color/255.0)
                    if marker_in_map_frame.pose.position.z > self.max_depth or marker_in_map_frame.pose.position.z < self.min_depth:
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

    def get_color_name(self, rgb_color):
        # Simple color classification based on average RGB values
        colors = {
            (255, 0, 0): "Red",
            (0, 255, 0): "Green",
            (0, 0, 255): "Blue",
            (255, 255, 0): "Yellow",
            (255, 255, 255): "White",
            (0, 0, 0): "Black",
        }

        #which colour is the closest to the average color of the ring
        closest_color = min(colors.keys(), key=lambda c: np.linalg.norm(np.array(c) - np.array(rgb_color)))

        return colors[closest_color]

    def create_marker(self, point_stamped, marker_id, lifetime=30.0, color=(0.0, 0.0, 1.0, 1.0)):
        marker = Marker()
        marker.header = point_stamped.header
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.id = marker_id

        scale = 0.15
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
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