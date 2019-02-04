#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
from geometry_msgs.msg import Vector3, Quaternion
import rosnode
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import math
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose
from std_msgs.msg import UInt8


class JoyWrapper(object):
    def __init__(self):
        self._sub_joy = rospy.Subscriber("joy", Joy, self._callback_joy, queue_size=1)

        self._grip_updated = False
        self._pose_updated = False
        self._name_updated = False
        self._preset_updated = False
        self._do_shutdown = False # ジョイスティックによる終了操作フラグ

        # /rane_x7_examples/launch/joystick_example.launch でキー割り当てを変更する
        self._BUTTON_SHUTDOWN_1 = rospy.get_param("~button_shutdown_1")
        self._BUTTON_SHUTDOWN_2 = rospy.get_param("~button_shutdown_2")
        self._BUTTON_HOME       = rospy.get_param("~button_home")
        self._BUTTON_POSI_ENABLE = rospy.get_param("~button_posi_enable")
        self._BUTTON_GRIP_ENABLE = rospy.get_param("~button_grip_enable")
        self._BUTTON_RPY_ENABLE = rospy.get_param("~button_rpy_enable")
        self._BUTTON_PRESET_PID = rospy.get_param("~button_preset_pid")
        self._AXIS_POSITION_X   = rospy.get_param("~axis_position_x")
        self._AXIS_POSITION_Y   = rospy.get_param("~axis_position_y")
        self._AXIS_POSITION_Z   = rospy.get_param("~axis_position_z")

        # 目標姿勢
        self._target_gripper_joint_values = [0.0, 0.0]
        self._target_arm_pose = Pose()
        self._target_arm_rpy = Vector3()
        self._target_name = "vertical"


    def set_target_gripper(self, joint_values):
        self._target_gripper_joint_values = joint_values

    def set_target_arm(self, pose):
        self._target_arm_pose = pose
        self._target_arm_rpy = self._orientation_to_rpy(pose.pose.orientation)


    def get_target_gripper(self):
        return self._target_gripper_joint_values

    def get_target_arm(self):
        return self._target_arm_pose

    def get_target_name(self):
        return self._target_name

    def do_shutdown(self):
        return self._do_shutdown

    def get_and_reset_grip_update_flag(self):
        flag = self._grip_updated
        self._grip_updated = False
        return flag

    def get_and_reset_pose_update_flag(self):
        flag = self._pose_updated
        self._pose_updated = False
        return flag

    def get_and_reset_name_update_flag(self):
        flag = self._name_updated
        self._name_updated = False
        return flag

    def get_and_reset_preset_update_flag(self):
        flag = self._preset_updated
        self._preset_updated = False
        return flag

    def _orientation_to_rpy(self, orientation):
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        e = euler_from_quaternion((x, y, z, w))
        return Vector3(e[0], e[1], e[2])

    def _rpy_to_orientation(self, rpy):
        q = quaternion_from_euler(rpy.x, rpy.y, rpy.z)
        return Quaternion(q[0], q[1], q[2], q[3])


    def _callback_joy(self, msg):

        # shutdown1, 2を同時押しでexampleを終了する
        if msg.buttons[self._BUTTON_SHUTDOWN_1] and msg.buttons[self._BUTTON_SHUTDOWN_2]:
            self._do_shutdown = True
            return

        # grip_enableが押されている時のみ、gripperの開閉を実行する
        if msg.buttons[self._BUTTON_GRIP_ENABLE]:
            grip_value = math.fabs(msg.axes[self._AXIS_POSITION_Z])

            if not grip_value:
                grip_value = 0.01

            self._target_gripper_joint_values = [grip_value, grip_value]
            self._grip_updated = True
        
        # posi_enableが押されている時のみ、armの位置を変更する
        if msg.buttons[self._BUTTON_POSI_ENABLE]:
            # ジョイスティックの仕様に合わせて、符号を変えてください
            self._target_arm_pose.pose.position.x -= msg.axes[self._AXIS_POSITION_X] * 0.1
            self._target_arm_pose.pose.position.y += msg.axes[self._AXIS_POSITION_Y] * 0.1
            self._target_arm_pose.pose.position.z += msg.axes[self._AXIS_POSITION_Z] * 0.1
            self._pose_updated = True

        # rpy_enableが押されている時のみ、armの角度を変更する
        if msg.buttons[self._BUTTON_RPY_ENABLE]:
            self._target_arm_rpy.x -= msg.axes[self._AXIS_POSITION_X] * math.pi * 0.1
            self._target_arm_rpy.y += msg.axes[self._AXIS_POSITION_Y] * math.pi * 0.1
            self._target_arm_rpy.z += msg.axes[self._AXIS_POSITION_Z] * math.pi * 0.1
            self._target_arm_pose.pose.orientation = self._rpy_to_orientation(self._target_arm_rpy)
            self._pose_updated = True

        if msg.buttons[self._BUTTON_HOME]:
            self._target_name = "home"
            self._name_updated = True

        # ご操作を防ぐため、presetフラグはボタン２つを同時押しでTrueにする
        if msg.buttons[self._BUTTON_PRESET_PID]:
            if msg.buttons[self._BUTTON_SHUTDOWN_2]:
                self._preset_updated = True


def preset_pid_gain(pid_gain_no):
    # サーボモータのPIDゲインをプリセットする
    # プリセットの内容はcrane_x7_control/scripts/preset_reconfigure.pyに書かれている
    rospy.loginfo("PID Gain Preset. No." + str(pid_gain_no))
    preset_no = UInt8()
    preset_no.data = pid_gain_no
    pub_preset.publish(preset_no)
    rospy.sleep(1) # PIDゲインがセットされるまで待つ


def main():
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.5)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    PRESET_DEFAULT  = 0
    PRESET_FREE     = 1
    pid_gain_no = PRESET_DEFAULT

    # rvizが起動するまで待つ
    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)

    # 何かを掴んでいた時のためにハンドを開く
    gripper.set_joint_value_target([0.9, 0.9])
    gripper.go()

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()

    # 現在のアーム姿勢を、目標姿勢にセットする
    joy_wrapper.set_target_arm(arm.get_current_pose())
    joy_wrapper.set_target_gripper(gripper.get_current_joint_values())

    while joy_wrapper.do_shutdown() == False:
        # グリッパーの角度を変更する
        if joy_wrapper.get_and_reset_grip_update_flag():
            gripper.set_joint_value_target(joy_wrapper.get_target_gripper())
            gripper.go()

        # アームの姿勢を変更する
        if joy_wrapper.get_and_reset_pose_update_flag():
            arm.set_pose_target(joy_wrapper.get_target_arm())
            if arm.go() == False:
                # 現在のアーム姿勢を、目標姿勢にセットする
                joy_wrapper.set_target_arm(arm.get_current_pose())

        # アームの姿勢をpose_group (home or vertical)に変更する
        if joy_wrapper.get_and_reset_name_update_flag():
            arm.set_named_target(joy_wrapper.get_target_name())
            arm.go()
            # 現在のアーム姿勢を、目標姿勢にセットする
            joy_wrapper.set_target_arm(arm.get_current_pose())

        # アームのPIDゲインをプリセットする
        if joy_wrapper.get_and_reset_preset_update_flag():
            # PIDゲインを0:default, 1:Free に切り替える
            if pid_gain_no == PRESET_DEFAULT:
                pid_gain_no = PRESET_FREE
            else:
                pid_gain_no = PRESET_DEFAULT
                # PIDゲインをdefaultに戻すと、目標姿勢に向かって急に動き出す
                # 安全のため、現在のアームの姿勢を目標姿勢に変更する
                arm.set_pose_target(arm.get_current_pose())
                arm.go()

            preset_pid_gain(pid_gain_no)
            # 現在のアーム姿勢を、目標姿勢にセットする
            joy_wrapper.set_target_arm(arm.get_current_pose())


    rospy.loginfo("Shutdown...")

    # SRDFに定義されている"vertical"の姿勢にする
    arm.set_named_target("vertical")
    arm.go()


if __name__ == '__main__':
    rospy.init_node("joystick_example")

    joy_wrapper = JoyWrapper()

    # PIDゲインプリセット用のPublisher
    pub_preset = rospy.Publisher("preset_gain_no", UInt8, queue_size=1)

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
