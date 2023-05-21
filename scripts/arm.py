#! /usr/bin/env python3

import numpy as np

import sys
import time
import threading

import conn_utilities
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2


class Gen3Lite:
    def __init__(self):
        self.TIMEOUT_DURATION = 20
    
    
    def init(self):
        # Connection
        args = conn_utilities.parseConnectionArguments()
        self.device_conn = conn_utilities.DeviceConnection.createTcpConnection(args)
        self.device_conn.__enter__()
        self.router = self.device_conn.router
        
        # Base Client
        self.base = BaseClient(self.router)
        self.base_cyclic = BaseCyclicClient(self.router)
        
        # Serve
        base_servo_mode = Base_pb2.ServoingModeInformation()
        base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        self.base.SetServoingMode(base_servo_mode)
    
    
    def close(self):
        self.device_conn.__exit__(None, None, None)
    
    
    def check_for_end_or_abort(self, e):
        def check(notification, e = e):
            print("NOTIFICATION EVENT : " + \
                Base_pb2.ActionEvent.Name(notification.action_event))
            if notification.action_event == Base_pb2.ACTION_END \
            or notification.action_event == Base_pb2.ACTION_ABORT:
                e.set()
        return check

    
    def wait_for_notification(self):
        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )
        finished = e.wait(self.TIMEOUT_DURATION)
        self.base.Unsubscribe(notification_handle)
        return finished
    
    
    def move_home(self):
        action_type = Base_pb2.RequestedActionType()
        action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
        action_list = self.base.ReadAllActions(action_type)
        action_handle = None
        for action in action_list.action_list:
            if action.name == "Home":
                action_handle = action.handle
        if action_handle == None:
            print("RESULT : FAILURE. Home action not found.")
            return False
        self.base.ExecuteActionFromReference(action_handle)
        
        finished = self.wait_for_notification()
        print("RESULT : {}.".format("SUCCESS" if finished else "FAILURE"))
        return finished
    
    
    def set_gripper_position(self, position):
        gripper_command = Base_pb2.GripperCommand()
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        finger = gripper_command.gripper.finger.add()
        finger.value = position
        
        self.base.SendGripperCommand(gripper_command)
        time.sleep(2.5) # TODO
    
    
    def get_feedback(self):
        feedback = self.base_cyclic.RefreshFeedback()
        return feedback
    
    
    def get_current_cartesian_pose(self):
        feedback = self.get_feedback()
        current_pose = np.array([
            feedback.base.tool_pose_x,
            feedback.base.tool_pose_y,
            feedback.base.tool_pose_z,
            feedback.base.tool_pose_theta_x,
            feedback.base.tool_pose_theta_y,
            feedback.base.tool_pose_theta_z
        ])
        return current_pose
    
    
    def get_current_cartesian_velocity(self):
        feedback = self.get_feedback()
        current_velocity = np.array([
            feedback.base.tool_twist_linear_x,
            feedback.base.tool_twist_linear_y,
            feedback.base.tool_twist_linear_z,
            feedback.base.tool_twist_angular_x,
            feedback.base.tool_twist_angular_y,
            feedback.base.tool_twist_angular_z
        ])
        return current_velocity

    
    def get_current_angular_pose(self):
        feedback = self.get_feedback()
        current_thetas = np.array([feedback.actuators[idx].position for idx in range(6)])
        return current_thetas
    
    
    def get_current_angular_velocity(self):
        feedback = self.get_feedback()
        current_thetas_dot = np.array([feedback.actuators[idx].velocity for idx in range(6)])
        return current_thetas_dot
    
    
    def get_current_external_wrench(self):
        feedback = self.get_feedback()
        current_external_wrench = np.array([
            feedback.base.tool_external_wrench_force_x,
            feedback.base.tool_external_wrench_force_y,
            feedback.base.tool_external_wrench_force_z,
            feedback.base.tool_external_wrench_torque_x,
            feedback.base.tool_external_wrench_torque_y,
            feedback.base.tool_external_wrench_torque_z
        ])
        return current_external_wrench
    
    
    def set_cartesian_pose(self, cartesian_pose):
        action = Base_pb2.Action()

        target_cartesian_pose = action.reach_pose.target_pose
        target_cartesian_pose.x = cartesian_pose[0]
        target_cartesian_pose.y = cartesian_pose[1]
        target_cartesian_pose.z = cartesian_pose[2]
        target_cartesian_pose.theta_x = cartesian_pose[3]
        target_cartesian_pose.theta_y = cartesian_pose[4]
        target_cartesian_pose.theta_z = cartesian_pose[5]

        self.base.ExecuteAction(action)

        finished = self.wait_for_notification()
        print("RESULT : {}.".format("SUCCESS" if finished else "FAILURE"))
        return finished

    
    def set_angular_pose(self, thetas):
        action = Base_pb2.Action()
        
        for idx in range(6):
            joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.joint_identifier = idx
            joint_angle.value = thetas[idx]

        self.base.ExecuteAction(action)

        finished = self.wait_for_notification()
        print("RESULT : {}.".format("SUCCESS" if finished else "FAILURE"))
        return finished
    
    
    def stop_motioning(self):
        self.base.Stop()
    
    
    def set_joint_speeds(self, thetas_d):
        joint_speeds = Base_pb2.JointSpeeds()
        
        for idx in range(6):
            joint_speed = joint_speeds.joint_speeds.add()
            joint_speed.joint_identifier = idx
            joint_speed.value = thetas_d[idx]
            joint_speed.duration = 0
        
        self.base.SendJointSpeedsCommand(joint_speeds)
    
    
    def set_twist(self, twists, reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE):
        command = Base_pb2.TwistCommand()
        command.reference_frame = reference_frame
        command.duration = 0
        
        twist = command.twist
        twist.linear_x = twists[0]
        twist.linear_y = twists[1]
        twist.linear_z = twists[2]
        twist.angular_x = twists[3]
        twist.angular_y = twists[4]
        twist.angular_z = twists[5]
        
        self.base.SendTwistCommand(command)

