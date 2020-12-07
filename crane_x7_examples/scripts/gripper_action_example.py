#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler


def main():
    rospy.init_node("crane_x7_pick_and_place_controller")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    def move_max_velocity(value = 0.5):
        arm.set_max_velocity_scaling_factor(value)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)

    print("Group names:")
    print(robot.get_group_names())

    print("Current state:")
    print(robot.get_current_state())



    stop_time = 2.0  # 停止する時間を指定

    force_hold_stick = 0.5 # 棒を握る力を指定
    
    te_x_position_vertical = 0.193040 # x
    te_y_position_vertical = -0.233386 # y
    te_z_position_vertical = 0.085469  # z
    stick_angle_vertical = 1.3 # 棒

    
    # アーム初期ポーズを表示
    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)

    # ハンドを開く/ 閉じる
    def move_gripper(pou):
        gripper.set_joint_value_target([pou, pou])
        gripper.go()

    # アームを移動する
    def move_arm(pos_x, pos_y, pos_z):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x
        target_pose.position.y = pos_y
        target_pose.position.z = pos_z
        q = quaternion_from_euler(-3.14/2.0, 3.14, -3.14/2.0) 
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  
        arm.go() 
        
    def preparation_vertical():
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = te_x_position_vertical
        target_pose.position.y = te_y_position_vertical
        target_pose.position.z = te_z_position_vertical
        q = quaternion_from_euler(-3.14/2.0, 3.14, -3.14/2.0)  
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  
        arm.go()  
        
    def p1_vertical():
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = te_x_position_vertical
        target_pose.position.y = te_y_position_vertical
        target_pose.position.z = te_z_position_vertical
        q = quaternion_from_euler(0, 3.14, -0)  
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  
        arm.go()  
        
    def p2_vertical():
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = te_x_position_vertical
        target_pose.position.y = te_y_position_vertical
        target_pose.position.z = te_z_position_vertical
        q = quaternion_from_euler(0, 3.14, -3.14/2.0)  
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  
        arm.go()  
        
    def p3_vertical():
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = te_x_position_vertical
        target_pose.position.y = te_y_position_vertical
        target_pose.position.z = te_z_position_vertical
        q = quaternion_from_euler(0, 3.14, 0) 
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose) 
        arm.go() 
        
    def p4_vertical():
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = te_x_position_vertical
        target_pose.position.y = te_y_position_vertical
        target_pose.position.z = te_z_position_vertical
        q = quaternion_from_euler(0, 3.14, -3.14/2.0) 
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  
        arm.go() 
        
        
    preparation_vertical()
    preparation_vertical()
    p1_vertical()
    preparation_vertical()
    p2_vertical()
    preparation_vertical()
    p3_vertical()
    preparation_vertical()
    p4_vertical()
    preparation_vertical()
    p1_vertical()
    preparation_vertical()
 
    move_max_velocity()
    arm.set_named_target("home")
    arm.go()



if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
