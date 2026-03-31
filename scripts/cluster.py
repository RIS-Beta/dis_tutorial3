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
from dis_tutorial3.msg import ClusterMsg, ClusterArray
from dis_tutorial3.srv import PeopleCluster

class Cluster:

    #global id counter for clusters
    id_counter = 0

    def __init__(self, type, position, normal, count=1, status = "NOT_INTERACTED", avg_color=None):
        self.id = self.id_counter
        Cluster.id_counter += 1
        self.type = type # "people" or "rings"
        self.center_position = position # [y,x,z] 
        self.status = status # "NOT_INTERACTED", "READY", "INTERACTED"
        self.normal = normal # normal vector for calcluating the pose for robot postion to interact
        self.count = count #how many markers are in the cluster for better clustering
        self.avg_color = avg_color #average color of the cluster for rings

    def update(self, new_marker, new_normal, color=None):
        prev_count = self.count

        #getting new center
        self.center_position = [(self.center_position[0]*prev_count + new_marker.pose.position.y)/(prev_count+1),
                                (self.center_position[1]*prev_count + new_marker.pose.position.x)/(prev_count+1),
                                (self.center_position[2]*prev_count + new_marker.pose.position.z)/(prev_count+1)]
        
        #normal vector for calculating the pose for robot postion to interact
        sum_normal = [self.normal[0]*prev_count + new_normal[0],
                       self.normal[1]*prev_count + new_normal[1],
                       self.normal[2]*prev_count + new_normal[2]]
        norm = np.linalg.norm(sum_normal)
        if norm != 0:
            self.normal = [sum_normal[0]/norm, sum_normal[1]/norm, sum_normal[2]/norm]
        
        self.count = prev_count + 1

        if self.type in ("ring", "rings") and color is not None:
            if hasattr(color, "x"):
                color_values = [color.x, color.y, color.z]
            else:
                color_values = [color[0], color[1], color[2]]

            if self.avg_color is not None:
                if hasattr(self.avg_color, "x"):
                    avg_color_values = [self.avg_color.x, self.avg_color.y, self.avg_color.z]
                else:
                    avg_color_values = [self.avg_color[0], self.avg_color[1], self.avg_color[2]]

                self.avg_color = [(avg_color_values[0]*prev_count + color_values[0])/(prev_count+1),
                                  (avg_color_values[1]*prev_count + color_values[1])/(prev_count+1),
                                  (avg_color_values[2]*prev_count + color_values[2])/(prev_count+1)]
            else:
                self.avg_color = color_values