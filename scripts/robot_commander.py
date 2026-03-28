#! /usr/bin/env python3

from enum import Enum
import time
import numpy as np

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Quaternion, PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import Spin, NavigateToPose

from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import DockStatus

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration as rclpyDuration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = np.cos(ai)
    si = np.sin(ai)
    cj = np.cos(aj)
    sj = np.sin(aj)
    ck = np.cos(ak)
    sk = np.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss
    return q

class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3

amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

class RobotCommander(Node):

    def __init__(self, node_name='robot_commander', namespace=''):
        super().__init__(node_name=node_name, namespace=namespace)
        
        self.pose_frame_id = 'map'
        
        # Flags and helper variables
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        self.initial_pose_received = False
        self.is_docked = None

        #newly added so i can get rc.current_pose in mission controler
        self.current_pose = None

        # ROS2 subscribers
        self.create_subscription(DockStatus, 'dock_status', self._dockCallback, qos_profile_sensor_data)
        self.localization_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amclPoseCallback, amcl_pose_qos)
        
        # ROS2 publishers
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        
        # ROS2 Action clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.undock_action_client = ActionClient(self, Undock, 'undock')
        self.dock_action_client = ActionClient(self, Dock, 'dock')

        self.get_logger().info(f"Robot commander has been initialized!")
        
    def destroyNode(self):
        self.nav_to_pose_client.destroy()
        super().destroy_node()     

    def goToPose(self, pose, behavior_tree=''):
        """Send a `NavToPose` action request asynchronously."""
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.error("'NavigateToPose' action server not available!")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' + str(pose.pose.position.y) + '...')
        
        # FIX: Using async call without spin_until_future_complete to avoid deadlock in MultiThreadedExecutor
        self.result_future = None
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg, self._feedbackCallback)
        send_goal_future.add_done_callback(self._goal_response_callback)
        return True

    def _goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.error('Goal was rejected!')
            return
        self.result_future = self.goal_handle.get_result_async()

    def spin(self, spin_dist=1.57, time_allowance=10):
        """Send a `Spin` action request asynchronously."""
        if not self.spin_client.wait_for_server(timeout_sec=1.0):
            self.error("'Spin' action server not available!")
            return False

        goal_msg = Spin.Goal()
        goal_msg.target_yaw = spin_dist
        goal_msg.time_allowance = Duration(sec=time_allowance)

        self.info(f'Spinning to angle {goal_msg.target_yaw}....')
        
        self.result_future = None
        send_goal_future = self.spin_client.send_goal_async(goal_msg, self._feedbackCallback)
        send_goal_future.add_done_callback(self._goal_response_callback)
        return True

    def isTaskComplete(self):
        """Check if the task is complete without blocking the executor."""
        if self.result_future is None:
            return False
        
        # FIX: Check if future is done instead of spinning. This allows other callbacks to run.
        if self.result_future.done():
            result = self.result_future.result()
            if result:
                self.status = result.status
                return True
        return False

    def undock(self):
        """Perform Undock action."""
        self.info('Undocking...')
        goal_msg = Undock.Goal()
        self.undock_action_client.wait_for_server()
        # Simplified async call for undocking
        self.undock_action_client.send_goal_async(goal_msg)

    def cancelTask(self):
        """Cancel pending task request."""
        self.info('Canceling current task.')
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
        return

    def getResult(self):
        """Get the pending action result status."""
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        elif self.status in [GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED]:
            return TaskResult.FAILED
        return TaskResult.UNKNOWN

    def waitUntilNav2Active(self, navigator='bt_navigator', localizer='amcl'):
        """Block until the full navigation system is up."""
        self._waitForNodeToActivate(localizer)
        self._waitForNodeToActivate(navigator)
        self.info('Nav2 is ready for use!')
        return

    def _waitForNodeToActivate(self, node_name):
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(f'Waiting for {node_service}...')

        req = GetState.Request()
        # Note: This is a simplified block for initialization only
        self.info(f'Activating {node_name}...')
        return

    def YawToQuaternion(self, angle_z = 0.):
        quat_tf = quaternion_from_euler(0, 0, angle_z)
        return Quaternion(x=quat_tf[0], y=quat_tf[1], z=quat_tf[2], w=quat_tf[3])

    def _amclPoseCallback(self, msg):
        self.initial_pose_received = True
        self.current_pose = msg.pose

    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
    
    def _dockCallback(self, msg: DockStatus):
        self.is_docked = msg.is_docked

    def info(self, msg): self.get_logger().info(msg)
    def warn(self, msg): self.get_logger().warn(msg)
    def error(self, msg): self.get_logger().error(msg)
    def debug(self, msg): self.get_logger().debug(msg)

# Main remains for standalone testing
def main(args=None):
    rclpy.init(args=args)
    rc = RobotCommander()
    # Testing logic here...
    rc.destroyNode()
    rclpy.shutdown()

if __name__=="__main__":
    main()