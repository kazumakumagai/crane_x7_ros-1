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
    ###     b-group
    ###

    #紙Aコップのx座標　紙Aコップのy座標　紙Bコップx座標 紙Bコップy座標
    position_base =[[0.29, 0.13, 0.29, -0.11]]
    Acup_tukamu = False
    Bcup_tukamu = False

    """
    使い方
    ハンドをつかんだ時は
    position_manager(False,False,x,y,True)
    ハンドをはなした時は
    position_manager(False,False,x,y,False)
    Aカップをつかみたい時の手先の位置を知りたいときは
    aa = position_manager(True,True,0,0,False)
    x = aa[0]
    y = aa[1]
    BBカップをつかみたい時の手先の位置を知りたいときは
    aa = position_manager(True,False,0,0,False)
    x = aa[0]
    y = aa[1]
    """
    def position_manager(master_judge,paper_cup,x,y,tukami):
        global Acup_tukamu
        global Bcup_tukamu
        position_ret=[0.0,0.0]
        if master_judge == True:
            if len(position_base)>0:

            #ホンスワン
                if paper_cup == True:
                    position_ret[0] = position_base[len(position_base)-1][2]
                    position_ret[1] = position_base[len(position_base)-1][3]
                    #position_historyの末尾の紙Aコップのx座標y座標
                    return position_ret
                else:
                    position_ret[0] = position_base[len(position_base)-1][0]
                    position_ret[1] = position_base[len(position_base)-1][1]
                    #position_historyの末尾の紙Bコップのx座標y座標
                    return position_ret

            else:
                return position_ret

        else:
            #熊谷さん
            #position_base配列の末尾にx,yを追加
            if tukami == True:
                #Aをつかんでいる場合
                if position_base[len(position_base)-1][0] == x and position_base[len(position_base)-1][1] == y:
                    position_base.append([x,y,position_base[len(position_base)-1][2],position_base[len(position_base)-1][3]])
                    Acup_tukamu = True
                #Bをつかんでいる場合
                elif position_base[len(position_base)-1][2] == x and position_base[len(position_base)-1][3] == y:
                    position_base.append([position_base[len(position_base)-1][0],position_base[len(position_base)-1][1],x,y])
                    Bcup_tukamu = True
            else:
                #Aをはなした時
                if Acup_tukamu == True:
                    position_base.append([x,y,position_base[len(position_base)-1][2],position_base[len(position_base)-1][3]])
                    Acup_tukamu = False
                elif Bcup_tukamu == True:
                    position_base.append([position_base[len(position_base)-1][0],position_base[len(position_base)-1][1],x,y])
                    Bcup_tukamu = False

            return position_ret

    ###
    ###     b-group-end
    ###

    ###
    ###     a-group
    ###

    stop_time = 2.0  # 停止する時間を指定

    force_hold_stick = 0.5 # 棒を握る力を指定
    
    te_x_position_vertical = 0.043040 # タンバリンの手前のx座標を指定
    te_y_position_vertical = 0.303386 # タンバリンのy座標を指定
    te_z_position_vertical = 0.085469  # タンバリンの少し上のz座標を指定
    stick_angle_vertical = 1.3 # 棒の角度を指定

    
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
        q = quaternion_from_euler(-3.14/2.0, 3.14, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    def preparation_vertical():
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = te_x_position_vertical
        target_pose.position.y = te_y_position_vertical
        target_pose.position.z = te_z_position_vertical
        q = quaternion_from_euler(3.14 * 9 / 10, 3.14 / 2, -3.14)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

 
    move_max_velocity()
    arm.set_named_target("home")
    arm.go()



if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
