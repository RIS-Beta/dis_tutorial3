#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped

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
        new_marker_topic = "/new_rings_marker"

        self.detection_color = (0,255,0)
        self.device = self.get_parameter('device').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scan = None
        self.new_marker_id = 0

        #thersholdings
        self.ration_thr = 1.5
        self.center_thr = 5.0 # Malo povečano za boljšo toleranco v simulatorju

        self.rgb_image_sub = self.create_subscription(Image, "/oakd/rgb/preview/image_raw", self.rgb_callback, qos_profile_sensor_data)
        self.pointcloud_sub = self.create_subscription(PointCloud2, "/oakd/rgb/preview/depth/points", self.pointcloud_callback, qos_profile_sensor_data)

        self.marker_pub = self.create_publisher(Marker, marker_topic, qos_profile_sensor_data)
        self.marker_new_pub = self.create_publisher(Marker, new_marker_topic, qos_profile_sensor_data)

        self.rings = []
        self.markers = []

        self.get_logger().info(f"Node has been initialized! Will publish ring markers to {marker_topic}.")

    def rgb_callback(self, data):
        self.rings = []
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #color can not be detected gray image
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (9, 9), 2)
            
            #hugh cricles
            circles = cv2.HoughCircles(
                blurred,
                cv2.HOUGH_GRADIENT,
                dp=1.2,
                minDist=50,
                param1=100,
                param2=35,     # Višja številka = bolj stroga detekcija
                minRadius=4,
                maxRadius=80,  # Omejitev, da ne prepozna prevelikih objektov
            )

            if circles is not None:
                circles = np.round(circles[0, :]).astype(int)
                for x, y, radius in circles:
                    self.get_logger().info(f"Ring detected (Hough): {x}, {y}")
                    cv2.circle(cv_image, (x, y), radius, (0, 255, 0), 2)
                    cv2.circle(cv_image, (x, y) , 5, (0, 0, 255), -1)
                    self.rings.append((x, y, radius))
           
            cv2.imshow("Ring detection Window", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def pointcloud_callback(self, data):
        if not self.rings:
            return

        try:
            # Preberemo numpy točke ENKRAT pred zanko (optimizacija)
            a = pc2.read_points_numpy(data, field_names=("x", "y", "z"))
            a = a.reshape((data.height, data.width, 3))

            for x, y, radius in self.rings:
                # Preveri meje slike
                if x >= data.width or y >= data.height or x < 0 or y < 0:
                    continue

                edge_x = int(x + (radius * 0.8)) # 80% radija, da smo sigurno na materialu
                edge_x = min(edge_x, data.width - 1)
                
                d = a[y, edge_x, :]
                
                # Če je desni rob NaN, poskusi še levo
                if np.isnan(d[0]) or np.isinf(d[0]):
                    edge_x_left = max(int(x - (radius * 0.8)), 0)
                    d = a[y, edge_x_left, :]
                    
                    if np.isnan(d[0]) or np.isinf(d[0]):
                        continue

                center_point = a[y, x, :]
                center_z = center_point[2]
                edge_z = d[2]

                # Če center NI nan, preverimo razliko v globini
                # Če je razlika med centrom in robom majhna (< 10cm), je to stena/nalepka
                if not np.isnan(center_z) and not np.isinf(center_z):
                    if abs(center_z - edge_z) < 0.1:
                        # self.get_logger().info("Preskakovanje: Ni votel (verjetno nalepka).")
                        continue

                # 3. Priprava točke za transformacijo
                point_in_cam_frame = PointStamped()
                point_in_cam_frame.header = data.header # Uporabi originalen header (frame + stamp)
                point_in_cam_frame.point.x = float(d[0])
                point_in_cam_frame.point.y = float(d[1])
                point_in_cam_frame.point.z = float(d[2])

                timeout = Duration(seconds=0.2)
                try:
                    trans = self.tf_buffer.lookup_transform(
                        "map", 
                        data.header.frame_id, 
                        Time(),
                        timeout
                    )

                    point_in_map_frame = tfg.do_transform_point(point_in_cam_frame, trans)
                    # Ustvari marker v mapi
                    marker_in_map_frame = self.create_marker(point_in_map_frame, self.new_marker_id, 0.0)

                    # Objavi marker
                    self.marker_new_pub.publish(marker_in_map_frame)
                    self.new_marker_id += 1
                    
                    self.marker_pub.publish(marker_in_map_frame)
                    self.markers.append(marker_in_map_frame)
                    

                    #TODO: clusetering
                    #TODO:color recognition


                    self.get_logger().info(f"Ring marker objavljen na: {point_in_map_frame.point.x:.2f}")

                except TransformException as te:
                    self.get_logger().warn(f"Could not get transform: {te}")

        except Exception as e:
            self.get_logger().error(f"Pointcloud error: {e}")

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