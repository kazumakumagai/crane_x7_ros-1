#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
引用元
紙コップ　→　https://github.com/GakuKuwano/crane_x7_ros/blob/master/crane_x7_gazebo/worlds/table2.world
アーム動作　→　https://github.com/GakuKuwano/crane_x7_ros/blob/master/crane_x7_examples/scripts/papercup_tower.py
"""

"""
流れ
準備
    コップAとそれに対応するボールA、コップBとそれに対応するボールBを用意する
    コップAとコップBは同y、ｚ座標に置く
    コップA中にはボールAを入れておき、ボールBはコップBの横においておく
        #ボールの確認
    1.	コップAの手前に移動
    2.	アーム先端を下に向ける
    3.	アームを垂直に下げる
    4.	コップAの下部を握る
    5.	垂直に持ち上げる
    6.	目標座標へ到着後1秒ほど停止
    7.	コップAを下げる
    8.	アームを全開に
    9.	垂直に持ち上げる
    10.	コップBの上に移動
    11.	アーム先端を下に向ける
    12.	アームを垂直に下げる
    13.	コップBの上部を握る
    14.	垂直に持ち上げる
    15.	目標座標へ到着後1秒ほど停止
    16.	横へ移動させボールBの上で停止
    17.	垂直に降下させボールBにコップBをかぶせる
    18.	コップBを横移動させ元の位置へ戻す
    19.	アームを全開に
    20.	垂直に持ち上げる
        #パフォーマンス（消す前の）
    1.	ホームへ戻る
    2.	アームを全開に
        #ボールを瞬間移動
    1.	コップBの上に移動
    2.	アーム先端を下に向ける
    3.	アームを垂直に下げる
    4.	コップBの下部を握る
    5.	垂直に持ち上げる
    6.	目標座標へ到着後1秒ほど停止
    7.	コップBを下げる
    8.	アームを全開に
    9.	垂直に持ち上げる
    10.	コップAの上に移動
    11.	アーム先端を下に向ける
    12.	アームを垂直に下げる
    13.	コップAの上部を握る
    14.	垂直に持ち上げる
    15.	目標座標へ到着後1秒ほど停止
    16.	コップAを下げ、アームを全開に
    17.	垂直に持ち上げる
        #ボールが瞬間移動したことを見せる
    1.	コップAの上に移動
    2.	コップAの上部を握る
    3.	アーム先端を下に向ける
    4.	垂直に持ち上げる
    5.	コップAを手前に移動
    6.	コップAを下す
    7.	アームを全開に
    8.	垂直に持ち上げる
    9.	コップBの上に移動
    10.	アーム先端を下に向ける
    11.	コップBの下部を握る
    12.	垂直に持ち上げる
    13.	コップBを手前に移動
    14.	コップBを下す
    15.	アームを全開に
    16.	垂直に持ち上げる
    17.	ホームへ戻る
"""

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
    position_base =[[0.30, 0.13, 0.30, -0.11]]
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
        q = quaternion_from_euler(-3.14/2.0, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    # コップ上部をつかむ位置へ移動１
    def move_arm_upper(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x - 0.13
        target_pose.position.y = pos_y
        target_pose.position.z = 0.075
        q = quaternion_from_euler(-3.14/2.0, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    # コップ上部をつかむ位置へ移動２
    def move_arm_upper_catch(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x - 0.005
        target_pose.position.y = pos_y
        target_pose.position.z = 0.075
        q = quaternion_from_euler(-3.14/2.0, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    # コップ下部をつかむ位置へ移動１
    def move_arm_lower(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x - 0.13
        target_pose.position.y = pos_y
        target_pose.position.z = -0.01
        q = quaternion_from_euler(-3.14/2.0 - 0.2, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    # コップ下部をつかむ位置へ移動２
    def move_arm_lower_catch(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x - 0.005
        target_pose.position.y = pos_y
        target_pose.position.z = -0.01
        q = quaternion_from_euler(-3.14/2.0 - 0.2, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    # コップの上を持ち上へ移動
    def move_arm_upper_up(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x - 0.005
        target_pose.position.y = pos_y
        target_pose.position.z = 0.075 + 0.1
        q = quaternion_from_euler(-3.14/2.0, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    # コップの下を持ち上へ移動
    def move_arm_lower_up(pos_x, pos_y):
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos_x - 0.005
        target_pose.position.y = pos_y
        target_pose.position.z = -0.01 + 0.1
        q = quaternion_from_euler(-3.14/2.0, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

    ###
    ###     a-group-end
    ###

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()
 
    #ハンドを開く
    move_gripper(1.3)
    
    #コップA下部を掴む-----1
    #流れ　1,2,3,4,5,6,7,8,9

    #aaにコップAの(x,y)を代入
    aa = position_manager(True, True, 0, 0, False)
    #グリッパを開く
    move_gripper(1.3)
    #アームをコップの下部と水平な部分へ移動
    move_arm_lower(aa[0], aa[1])
    #アームをコップの下部とくっつける
    move_arm_lower_catch(aa[0], aa[1])
    #グリッパを閉じ、コップをつかむ
    move_gripper(0.28)
    #アームを持ち上げる(空中で停止させるため複数回呼び出している)
    move_arm_lower_up(aa[0], aa[1])
    move_arm_lower_up(aa[0], aa[1])
    move_arm_lower_up(aa[0], aa[1])
    move_arm_lower_up(aa[0], aa[1])
    #アームを下げる
    move_arm_lower_catch(aa[0], aa[1])
    #グリッパを開き、コップを放す
    move_gripper(1.3)
    #アームを持ち上げコップから離れたらホームへ戻る
    move_arm_lower_up(aa[0], aa[1])
    move_max_velocity()
    arm.set_named_target("home")
    arm.go()


    #コップB上部を掴む-----2
    #流れ　10,11,12,13,14,15,16,17,18,19,20

    #aaにコップBの(x,y)を代入
    aa = position_manager(True, False, 0, 0, False)
    #グリッパを開く
    move_gripper(1.3)
    #アームをコップの下部と水平な部分へ移動
    move_arm_upper(aa[0], aa[1])
    #アームをコップの下部とくっつける
    move_arm_upper_catch(aa[0], aa[1])
    #グリッパを閉じ、コップをつかむ
    move_gripper(0.28)
    move_arm_upper_up(aa[0], aa[1])
    #アームを持ち上げる(空中で停止させるため複数回呼び出している)
    move_arm_upper_up(aa[0], aa[1])
    move_arm_upper_up(aa[0], aa[1])
    move_arm_upper_up(aa[0], aa[1])
    #アームをボールの真上へ(空中で停止させるため複数回呼び出している)
    move_arm_upper_up(aa[0], aa[1] + 0.1)
    move_arm_upper_up(aa[0], aa[1] + 0.1)
    move_arm_upper_up(aa[0], aa[1] + 0.1)
    move_arm_upper_up(aa[0], aa[1] + 0.1)
    #アームを下げカップをボールにかぶせる
    move_arm_upper_catch(aa[0], aa[1] + 0.1)
    #元の位置へ戻す
    move_arm_upper_catch(aa[0], aa[1])
    #グリッパを開き、コップを放す
    move_gripper(1.3)
    #アームを持ち上げコップから離れたらホームへ戻る
    move_arm_upper_up(aa[0], aa[1])
    move_max_velocity()
    arm.set_named_target("home")
    arm.go()


    #パフォーマンスするならここ
    #取り敢えず先輩のコードをそのまま流用

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


    #コップB下部を掴む-----4
    #瞬間移動　1,2,3,4,5,6,7,8,9

    #aaにコップBの(x,y)を代入
    aa = position_manager(True, False, 0, 0, False)
    #グリッパを開く
    move_gripper(1.3)
    #アームをコップの下部と水平な部分へ移動
    move_arm_lower(aa[0], aa[1])
    #アームをコップの下部とくっつける
    move_arm_lower_catch(aa[0], aa[1])
    #グリッパを閉じ、コップをつかむ
    move_gripper(0.28)
    #アームを持ち上げる(空中で停止させるため複数回呼び出している)
    move_arm_lower_up(aa[0], aa[1])
    move_arm_lower_up(aa[0], aa[1])
    move_arm_lower_up(aa[0], aa[1])
    move_arm_lower_up(aa[0], aa[1])
    #アームを下げる
    move_arm_lower_catch(aa[0], aa[1])
    #グリッパを開き、コップを放す
    move_gripper(1.3)
    #アームを持ち上げコップから離れたらホームへ戻る
    move_arm_lower_up(aa[0], aa[1])
    move_max_velocity()
    arm.set_named_target("home")
    arm.go()


    #コップA上部を掴む-----4
    #瞬間移動　10,11,12,13,14,15,16,17

    #aaにコップAの(x,y)を代入
    aa = position_manager(True, True, 0, 0, False)
    #グリッパを開く
    move_gripper(1.3)
    #アームをコップの上部と水平な部分へ移動
    move_arm_upper(aa[0], aa[1])
    #アームをコップの上部とくっつける
    move_arm_upper_catch(aa[0], aa[1])
    #グリッパを閉じ、コップをつかむ
    move_gripper(0.28)
    move_arm_upper_up(aa[0], aa[1])
    #アームを持ち上げる(空中で停止させるため複数回呼び出している)
    move_arm_upper_up(aa[0], aa[1])
    move_arm_upper_up(aa[0], aa[1])
    move_arm_upper_up(aa[0], aa[1])
    move_arm_upper_up(aa[0], aa[1])
    #アームを下げる
    move_arm_upper_catch(aa[0], aa[1])
    #グリッパを開き、コップを放す
    move_gripper(1.3)
    #アームを持ち上げコップから離れたらホームへ戻る
    move_arm_upper_up(aa[0], aa[1])
    move_max_velocity()
    arm.set_named_target("home")
    arm.go()


    #コップAの上部をつかむ-----5
    #中身の確認

    #aaにコップAの(x,y)を代入
    aa = position_manager(True, True, 0, 0, False)
    #グリッパを開く
    move_gripper(1.3)
    #アームをコップの上部と水平な部分へ移動
    move_arm_upper(aa[0], aa[1])
    #アームをコップの上部とくっつける
    move_arm_upper_catch(aa[0], aa[1])
    #グリッパを閉じ、コップをつかむ
    move_gripper(0.28)
    move_arm_upper_up(aa[0], aa[1])
    #アームを持ち上げる(空中で停止させるため複数回呼び出している)
    move_arm_upper_up(aa[0], aa[1])
    move_arm_upper_up(aa[0], aa[1])
    move_arm_upper_up(aa[0], aa[1])
    move_arm_upper_up(aa[0], aa[1])
    #アームを手前に移動
    move_arm_upper_up(aa[0] - 0.13, aa[1])
    #アームを下げる
    move_arm_upper(aa[0], aa[1])
    #グリッパを開き、コップを放す
    move_gripper(1.3)
    #アームを持ち上げコップから離れたらホームへ戻る
    move_arm_upper_up(aa[0], aa[1])
    move_max_velocity()
    arm.set_named_target("home")
    arm.go()


    #コップB下部を掴む-----6
    #中身の確認

    #aaにコップBの(x,y)を代入
    aa = position_manager(True, False, 0, 0, False)
    #グリッパを開く
    move_gripper(1.3)
    #アームをコップの下部と水平な部分へ移動
    move_arm_lower(aa[0], aa[1])
    #アームをコップの下部とくっつける
    move_arm_lower_catch(aa[0], aa[1])
    #グリッパを閉じ、コップをつかむ
    move_gripper(0.28)
    #アームを持ち上げる(空中で停止させるため複数回呼び出している)
    move_arm_lower_up(aa[0], aa[1])
    move_arm_lower_up(aa[0], aa[1])
    move_arm_lower_up(aa[0], aa[1])
    move_arm_lower_up(aa[0], aa[1])
    #アームを手前に移動
    move_arm_lower_up(aa[0] - 0.13, aa[1])
    #アームを下げる
    move_arm_lower(aa[0], aa[1])
    #グリッパを開き、コップを放す
    move_gripper(1.3)
    #アームを持ち上げコップから離れたらホームへ戻る
    move_arm_lower_up(aa[0], aa[1])
    move_max_velocity()
    arm.set_named_target("home")
    arm.go()


if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
