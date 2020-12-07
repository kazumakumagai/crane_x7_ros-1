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

    ###
    ### 変数宣言
    ###

    stop_time = 2.0  # 停止する時間を指定

    force_hold_stick = 0.5 # 棒を握る力を指定
    
    
    te_x_position_vertical = 0.043040　#x座標
    te_y_position_vertical = 0.303386 # y座標
    te_z_position_vertical = 0.085469  # z座標
    stick_angle_vertical = 1.3 # 棒の角度を指定

   
 """
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
        q = quaternion_from_euler(3.14 * 9 / 10, 3.14 / 2, -3.14)  
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

   
    def hit_tambourine_vertical():
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = te_x_position_vertical
        target_pose.position.y = te_y_position_vertical
        target_pose.position.z = te_z_position_vertical
        q = quaternion_from_euler(3.14 * 9 / 10, stick_angle_vertical, -3.14)  
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  
        arm.go()  # 実行

　　 def move_arm(pos_x, pos_y, pos_z):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x
        target_pose.position.y = pos_y
        target_pose.position.z = pos_z
        q = quaternion_from_euler(-3.14/2.0, 3.14, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    # コップ上部をつかむ位置へ移動１
    def move_arm_upper(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x
        target_pose.position.y = pos_y
        target_pose.position.z = 0.25
        q = quaternion_from_euler(0, 3.14, 0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    # コップ上部をつかむ位置へ移動２
    def move_arm_upper_catch(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x
        target_pose.position.y = pos_y
        target_pose.position.z = weak_position
        q = quaternion_from_euler(0, 3.14, 0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    # コップ下部をつかむ位置へ移動１
    def move_arm_lower(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x
        target_pose.position.y = pos_y
        target_pose.position.z = 0.25
        q = quaternion_from_euler(0, 3.14, 0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    # コップ下部をつかむ位置へ移動２
    def move_arm_lower_catch(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x
        target_pose.position.y = pos_y
        target_pose.position.z = strong_position
        q = quaternion_from_euler(0, 3.14, 0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    # コップの上を持ち上へ移動
    def move_arm_upper_up(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x
        target_pose.position.y = pos_y
        target_pose.position.z = 0.25
        q = quaternion_from_euler(0, 3.14, 0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    
    def move_arm_lower_up(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x
        target_pose.position.y = pos_y
        target_pose.position.z = 0.25
        q = quaternion_from_euler(0, 3.14, 0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行
    """
    
      #頂点に振り上げる
    move_arm(0.1, 0.0, 0.5)
    #あおり
    move_arm(0.22, -0.2, 0.2)
    #頂点に振り上げる
    move_arm(0.1, 0.0, 0.5)
    #あおり
    move_arm(0.22, 0.2, 0.2)
    #homeに戻る
    arm.set_named_target("home")
    arm.go()

    
    
    
    
    
    
    
    
        
    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()





    move_max_velocity()
    arm.set_named_target("home")
    arm.go()


if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
