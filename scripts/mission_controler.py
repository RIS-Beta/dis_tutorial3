#!/usr/bin/env python3

import json
import math
import os
import random
import numpy as np
import threading
from copy import deepcopy

from dis_tutorial3.srv import Speech
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.duration import Duration

from robot_commander import RobotCommander, TaskResult

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

'''

MissionControler is the main node which will have 3 states explore, evaluate and interact. 
It will subscribe to markers of people and rings and cluster them for better interaction. 
It will also have a client for the voice commander to talk to people. 
The main loop will be in the states_loop function which will call the appropriate function for each state. 

'''
class MissionControler(Node):
    def __init__(self):
        super().__init__('mission_controler')
        self.get_logger().info("Mission controler node initialized")

        #robot commander for giving goal poses to robot
        self.robot_commander = RobotCommander(self)

        #TODO: we will fix this so the node will jiust read some file or smth not imported
        #waypoints for navigation
        self.waypoints = [
            {
                'position': {'x': 0.9992520366512115, 'y': 5.099285747708961, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.06709634580279415, 'w': 0.9977465010612224},
            },
            {
                'position': {'x': -0.9625339498989449, 'y': 3.266840370555999, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': -0.9895548398428475, 'w': 0.14415692471607602},
            },
            {
                'position': {'x': -3.7034241420503897, 'y': 3.838940628120296, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': -0.9973686480486497, 'w': 0.07249675778687495},
            },
        ]

        self.current_waypoint_index = 0
        
        #thresholds
        self.cluster_thr_people = 10 #number od markers in cluster for it to be valid
        self.cluster_thr_rings = 5 #number od markers in cluster for it to be valid
        self.cluster_distance_thr = 0.5 #distance in meters for markers to be in the same cluster
     
        #states of robot
        self.states = ["EXPLORE", "EVALUATE", "INTERACT"]
        self.current_state = self.states[0] 

        #subscription to markers of people and rings
        self.people_marker_sub = self.create_subscription(MarkerArray, "/people_marker", self.people_marker_callback, 10)
        self.rings_marker_sub = self.create_subscription(MarkerArray, "/rings_marker", self.rings_marker_callback, 10)
        
        #placeholder for markers
        self.people_markers = []
        self.rings_markers = []

        #voice commander client for talking to people
        self.voice_commander_client = self.create_client(Speech, 'speech')
        while not self.voice_commander_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for speech service to be available...')
        
        #clusters of people and rings for better interaction
        #cluster have also states interected not intercated
        self.people_cluster = []
        self.rings_cluster = []

        #array or clusters ready for interaction
        self.detected_objects = []

        #targeted object for interaction
        self.target_object = None

        #counters for intractions for stopping condition
        self.people_interaction_count = 0
        self.rings_interaction_count = 0

        self.get_logger().info("Mission controler node setup complete, starting main loop")

        #timer for changing states and doing main loop
        #TODO: nto sure if it is really needed you can just automatically switch states after finishing the task in each state, but we will see
        self.timer = self.create_timer(0.5, self.states_loop)

    def states_loop(self):
        if self.current_state == "EXPLORE":
            #exploring function
            self.state_explore()
        elif self.current_state == "EVALUATE":
            #evaluating function
            self.state_evaluate()
        elif self.current_state == "INTERACT":
            #interacting function
            self.state_interact()

        self.get_logger().info(f"Current state: {self.current_state}")
        
    #moving the robot to the given pose() 
    def move_to(self, pose):
        self.robot_commander.goToPose(pose)

        #wait until the robot reaches the goal
        while not self.robot_commander.isTaskComplete():
            rclpy.spin_once(self)
        
        result = self.robot_commander.getTaskResult()

        #feedback about the result of navigation
        if result == TaskResult.SUCCESS:
            self.get_logger().info("Robot reached the goal successfully")
        else:            
            self.get_logger().warn("Robot failed to reach the goal")

    def rotate(self, pose, angle = 2*math.pi):
        # Implementation for rotating the robot
        self.get_logger().info(f"Rotating robot by {math.degrees(angle)} degrees")
        
        #spin() has time_allowence if we want to rotate for given time
        self.robot_commander.spin(angle)

        while not self.robot_commander.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        result = self.robot_commander.getTaskResult()
        if result == TaskResult.SUCCESS:
            self.get_logger().info("Robot rotated successfully")
        else:
            self.get_logger().warn("Robot failed to rotate")

    #robot moves to next waypoint and then checks for detected objects
    def state_explore(self):
        #TODO: fixe so 360 roation beacoems mroe optimal (just fix this brute force)
        #navigate to waypoints
        if self.current_waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_waypoint_index]
            pose = Pose()
            pose.position.x = waypoint['position']['x']
            pose.position.y = waypoint['position']['y']
            pose.position.z = waypoint['position']['z']
            pose.orientation.x = waypoint['orientation']['x']
            pose.orientation.y = waypoint['orientation']['y']
            pose.orientation.z = waypoint['orientation']['z']
            pose.orientation.w = waypoint['orientation']['w']

            self.move_to(pose)

            #rotating 360 degrees to look around
            self.rotate(pose)

            self.get_logger().info(f"Finished exploring waypoint {self.current_waypoint_index}")
            self.current_waypoint_index += 1
            self.current_state = self.states[1] #switch to evaluate state after exploring each waypoint
        else:
            objects_for_interaction = [objects for objects in self.detected_objects if objects.status == "READY"] #ignoring those who are interacted

            if not objects_for_interaction:
                self.get_logger().info("Finished exploring all waypoints, no objects detected for interaction")
                self.timer.cancel() #stop the timer to prevent switching to evaluate state
                return
            else:
                self.get_logger().info("Finished exploring all waypoints, switching to evaluate state")
                self.current_state = self.states[1] #switch to evaluate state after finishing exploring all waypoints
        return


    def closest_point(self, objects_for_interaction):
        closest_object = None
        min_distance = float('inf')

        #getting the current position of the robot
        current_pose = self.robot_commander.current_pose
        if current_pose is None:
            self.get_logger().warn("Current robot pose is unknown, cannot evaluate closest object")
            return None

        #TODO: find a faster way to search for closest (but there arnet that many objects so it should be fine)
        closest_object_index = -1
        for i, obj in enumerate(objects_for_interaction):
            distance = math.sqrt((current_pose.position.x - obj.center_position[1])**2 + (current_pose.position.y - obj.center_position[0])**2)
            if distance < min_distance:
                min_distance = distance
                closest_object = obj
                closest_object_index = i
        self.get_logger().info(f"Closest object is {closest_object.type} with id {closest_object.id} at distance {min_distance:.2f} meters")
        return closest_object

    #evaluate the situation
    def state_evaluate(self):
        #getting all detected objects
        objects_for_interaction = [objects for objects in self.detected_objects if objects.status == "READY"] #ignoring those who are interacted
        self.get_logger().info(f"Evaluating detected objects for interaction, found {len(objects_for_interaction)} objects ready for interaction")

        if len(objects_for_interaction) == 0:
            self.current_state = self.states[0] #switch to explore state if no objects are ready for interaction
            self.get_logger().info(f"No objects ready for interaction, switching back to {{self.states[0]}} state")
            return
        
        closest_object = self.closest_point(objects_for_interaction)
        if closest_object is None:
            self.current_state = self.states[0] #switch to explore state if we cannot find the closest object
            self.get_logger().info(f"Could not determine closest object, switching back to {{self.states[0]}} state")
            return
        
        self.target_object = closest_object
        self.get_logger().info(f"Targeting object {self.target_object.type} with id {self.target_object.id} for interaction")
        self.current_state = self.states[2] #switching state to interact after evaluating the situation
        return
    
    #interacting with the targeted object 
    def state_interact(self):
        #interact with the person (e.g. say something, ask for help, etc.)
        #after interaction, switch back to explore state
        #when array of intrest is empty switch back to explore state
        #if all person and rings detected stop 

        #getting the pose
        pose = Pose()

        #TODO: perhaps we change this based on the type of object
        distance_to_object = 0.5 #distcne of normal from the object

        pose.position.x = self.target_object.center_position[1] + self.target_object.normal[0]*distance_to_object
        pose.position.y = self.target_object.center_position[0] + self.target_object.normal[1]*distance_to_object
        pose.position.z = self.target_object.center_position[2]

        orientation = math.atan2(-self.target_object.normal[0], -self.target_object.normal[1])   
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = math.sin(orientation/2)
        pose.orientation.w = math.cos(orientation/2)

        #moving to target object
        self.move_to(pose)

        self.get_logger().info(f"Interacting with object {self.target_object.type} with id {self.target_object.id}")
        
        #functionallity for interaction with the object
        self.speech_future = None

        self.people_interaction()

        #TODO: add ring interaction
        #uncomment when ring interaction is implemented
        #if self.target_object.type == "people":
        #    self.people_interaction(self.target_object)
        #elif self.target_object.type == "ring":
        #    self.ring_interaction(self.target_object)

        while not self.speech_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        self.detected_objects[self.detected_objects.index(self.target_object)].status = "INTERACTED" #marking the object as interacted
        self.target_object = None
        self.current_state = self.states[1] #switching back to evaluate
        self.get_logger().info(f"Finished interaction, switching back to {self.states[1]} state")
        return

    def people_interaction(self):
        greetings = 'Hello human!, Nice face!, Nice to see you!, Hows it going?, What a nice day!, Hi there!, Greetings!, Salutations!, Hey!, Good to see you!'

        greeting = random.choice(greetings.split(',')).strip()
        request = Speech.Request()
        request.text = greeting

        self.get_logger().info(greeting)
        self.speech_future = self.voice_commander_client.call_async(request)
        return

    #TODO: implement ring interaction (e.g. say something about the ring, ask for help, etc.)
    def ring_interaction(self, cluster):
        return

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
        if type == "people":
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
                    if cluster.count >= self.cluster_thr_people and cluster.status == "NOT_INTERACTED":
                        self.detected_objects.append(cluster)
                        cluster.status = "READY" #ready means its hnot yet been interacted with but its ready for interaction because it has enough markers in cluster
                    break

            #we create new cluster if marker is not close to any existing cluster and add it to detected objects for interaction
            if not closeToCluster:
                new_cluster = Cluster(type, [position.y, position.x, position.z], normal_vector)
                self.people_cluster.append(new_cluster)
            pass
        elif type == "rings":
            #cluster rings markers and update rings_cluster
            position = marker.pose.position
            closeToCluster = False

            #getting the normal vector for calculating the pose for robot postion to interact
            normal_vector = self.calculating_normal_vector(marker)

            for cluster in self.rings_cluster:
                if math.sqrt((position.x - cluster.center_position[1])**2 + (position.y - cluster.center_position[0])**2) < self.cluster_distance_thr:
                    cluster.update(marker, normal_vector)
                    closeToCluster = True
                    if cluster.count >= self.cluster_thr_rings and cluster.status == "NOT_INTERACTED":
                        self.detected_objects.append(cluster)
                        cluster.status = "READY" #ready means its hnot yet been interacted with but its ready for interaction because it has enough markers in cluster
                    break

            if not closeToCluster:
                new_cluster = Cluster(type, [position.y, position.x, position.z], normal_vector)
                self.rings_cluster.append(new_cluster)
            pass
        return


    def people_marker_callback(self, msg):
        self.people_markers = msg.markers
        self.get_logger().info(f"Received {len(self.people_markers)} people markers")
        #cluster people markers and update people_cluster
        for marker in self.people_markers:
            self.clustering(marker, "people")
        return

    def rings_marker_callback(self, msg):
        self.rings_markers = msg.markers
        self.get_logger().info(f"Received {len(self.rings_markers)} rings markers")
        #cluster rings markers and update rings_cluster
        for marker in self.rings_markers:
            self.clustering(marker, "rings")
        return
    
    def create_marker(self, point_stamped, marker_id, lifetime=30.0):
        """You can see the description of the Marker message here: https://docs.ros2.org/galactic/api/visualization_msgs/msg/Marker.html"""
        marker = Marker()

        marker.header = point_stamped.header

        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.id = marker_id

        # Set the scale of the marker
        scale = 0.1
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        # Set the color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position.x = point_stamped.point.x
        marker.pose.position.y = point_stamped.point.y
        marker.pose.position.z = point_stamped.point.z

        marker.lifetime = Duration(seconds=lifetime).to_msg()

        return marker
    
def main():
    rclpy.init(args=None)
    node = MissionControler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()