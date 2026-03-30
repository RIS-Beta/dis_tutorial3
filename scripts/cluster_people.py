#!/usr/bin/env python3

import json
import math
import os
import random
import time
import numpy as np
import threading
from copy import deepcopy

from dis_tutorial3.srv import Speech
import rclpy
from geometry_msgs.msg import PointStamped, Pose, PoseArray, PoseStamped, Point
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.duration import Duration
from robot_commander import RobotCommander, TaskResult
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
'''
Cluster is a class that will be used to cluster people and rings for better interaction.
'''
class Cluster:

    #global id counter for clusters
    id_counter = 0

    def __init__(self, type, position, normal):
        self.id = self.id_counter
        Cluster.id_counter += 1
        self.type = type # "people" or "ring"
        self.center_position = position # [y,x,z] 
        self.status = "NOT_INTERACTED" # "NOT_INTERACTED", "READY", "INTERACTED"
        self.normal = normal # normal vector for calcluating the pose for robot postion to interact
        self.count = 1 #how many markers are in the cluster for better clustering

    def update(self, new_marker, new_normal):
        #getting new center
        self.center_position = [(self.center_position[0]*self.count + new_marker.pose.position.y)/(self.count+1),
                                (self.center_position[1]*self.count + new_marker.pose.position.x)/(self.count+1),
                                (self.center_position[2]*self.count + new_marker.pose.position.z)/(self.count+1)]
        
        #normal vector for calculating the pose for robot postion to interact
        sum_normal = [self.normal[0]*self.count + new_normal[0],
                       self.normal[1]*self.count + new_normal[1],
                       self.normal[2]*self.count + new_normal[2]]
        norm = np.linalg.norm(sum_normal)
        if norm != 0:
            self.normal = [sum_normal[0]/norm, sum_normal[1]/norm, sum_normal[2]/norm]
        
        self.count += 1

class cluster_people(Node):
    def __init__(self):
        super().__init__('cluster_people')

        #subscription to markers of people and rings
        self.people_marker_sub = self.create_subscription(Marker, "/new_people_marker", self.people_marker_callback, qos_profile_sensor_data)

        #publisher for map goals
        self.marker_pub = self.create_publisher(MarkerArray, "/people_clusters", QoSReliabilityPolicy.BEST_EFFORT) 

        self.people_cluster = []
        self.rings_cluster = []

        self.cluster_distance_thr = 0.5 #threshold for clustering markers into clusters
        self.cluster_thr_people = 150 #threshold for number of markers in cluster to consider it as detected object for people

    def calculating_normal_vector(self, marker):
        if len(marker.points) >= 2:
            p0 = marker.points[0]
            p1 = marker.points[1]
            normal_vector = [p1.x - p0.x, p1.y - p0.y, p1.z - p0.z]

            norm = np.linalg.norm(normal_vector)
            if norm != 0:   
                return [normal_vector[0]/norm, normal_vector[1]/norm, normal_vector[2]/norm]
        else:
            return [0,0,0]

    def clustering(self, marker, type):
        #cluster people markers and update people_cluster
        position = marker.pose.position
        closeToCluster = False

        #getting the normal vector for calculating the pose for robot postion to interact
        normal_vector = self.calculating_normal_vector(marker)

        #searching for close cluster for updating
        for cluster in self.people_cluster:
            if math.sqrt((position.x - cluster.center_position[1])**2 + (position.y - cluster.center_position[0])**2) < self.cluster_distance_thr:
                cluster.update(marker, normal_vector)
                closeToCluster = True
                self.get_logger().info(f"Updated cluster {cluster.id} for people with new marker, count of markers in cluster is now {cluster.count}")
                if cluster.count >= self.cluster_thr_people and cluster.status == "NOT_INTERACTED":
                    cluster.status = "READY" #ready means its hnot yet been interacted with but its ready for interaction because it has enough markers in cluster
                    self.post_goal_marker(cluster) #post marker for visualization in rviz when cluster is ready for interaction
                if cluster.count >= self.cluster_thr_people and cluster.status != "NOT_INTERACTED":
                    self.post_goal_marker(cluster) #post marker for visualization in rviz when cluster is ready for interaction
                    self.get_logger().info(f"Cluster {cluster.id} for people is already ready for interaction, count of markers in cluster is now {cluster.count}")
                break

        #we create new cluster if marker is not close to any existing cluster and add it to detected objects for interaction
        if not closeToCluster:
            new_cluster = Cluster(type, [position.y, position.x, position.z], normal_vector)
            self.people_cluster.append(new_cluster)
        return


    def people_marker_callback(self, msg):
        marker = msg
        self.get_logger().info(f"Received people marker")
        #cluster people markers and update people_cluster
        self.clustering(marker, "people")
        return

    def post_goal_marker(self, cluster):
        #create marker for cluster and publish it to map_goals_marker topic for visualization in rviz
        self.get_logger().info(f"Cluster {cluster.id} of type {cluster.type} is ready for interaction with count of markers in cluster {cluster.count}, posting marker for visualization in rviz")
        point_stamped = PointStamped()
        point_stamped.header.frame_id = "map"
        point_stamped.point.x = cluster.center_position[1]
        point_stamped.point.y = cluster.center_position[0]
        point_stamped.point.z = cluster.center_position[2]

        markerText = self.create_marker(point_stamped, cluster.id, lifetime=0.0, color=(0.0, 0.0, 0.0, 1.0), scale=0.3)
        marker = self.create_marker(point_stamped, cluster.id, lifetime=0.0, color=(1.0, 0.0, 0.0, 0.5), scale=0.7)
        normal_marker = self.create_normal_arrow_marker(point_stamped, cluster.id + 2000, cluster.normal, arrow_length=0.8, color=(1.0, 0.0, 0.0, 0.9))
        markerText.type = Marker.TEXT_VIEW_FACING
        markerText.id = cluster.id + 1000 

        markerText.text = f"{cluster.count}"

        markerArray = MarkerArray()
        markerArray.markers.append(marker)
        markerArray.markers.append(markerText)
        markerArray.markers.append(normal_marker)
        
        self.marker_pub.publish(markerArray)
        self.get_logger().info(f"Published marker for cluster {cluster.id} of type {cluster.type} with text {marker.text} to /map_goals_marker topic for visualization in rviz")


    def create_marker(self, point_stamped, marker_id, lifetime=30.0, color=(1.0, 0.0, 0.0, 1.0), scale=0.5):
        """You can see the description of the Marker message here: https://docs.ros2.org/galactic/api/visualization_msgs/msg/Marker.html"""
        marker = Marker()

        marker.header = point_stamped.header

        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.id = marker_id

        # Set the scale of the marker
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        # Set the color
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        # Set the pose of the marker
        marker.pose.position.x = point_stamped.point.x
        marker.pose.position.y = point_stamped.point.y
        marker.pose.position.z = point_stamped.point.z


        marker.lifetime = Duration(seconds=lifetime).to_msg()

        return marker

    def create_normal_arrow_marker(self, point_stamped, marker_id, normal_vector, arrow_length=0.8, lifetime=0.0, color=(0.0, 0.0, 1.0, 1.0)):
        marker = Marker()
        marker.header = point_stamped.header
        marker.ns = "cluster_normals"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        start = Point()
        start.x = point_stamped.point.x
        start.y = point_stamped.point.y
        start.z = point_stamped.point.z

        end = Point()
        end.x = start.x + normal_vector[0] * arrow_length
        end.y = start.y + normal_vector[1] * arrow_length
        end.z = start.z + normal_vector[2] * arrow_length

        marker.points = [start, end]

        marker.scale.x = 0.06
        marker.scale.y = 0.12
        marker.scale.z = 0.12

        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        marker.lifetime = Duration(seconds=lifetime).to_msg()
        return marker


def main():
	print('Cluster creator node started')

	rclpy.init(args=None)
	node = cluster_people()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
