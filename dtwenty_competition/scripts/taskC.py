#! /usr/bin/env python
# coding=utf-8
import rospy
import tf
import copy
from robot_utils import *
import baxter_interface
from image_proc_msgs.srv import ImageProcSrv, ImageProcSrvRequest, ImageProcSrvResponse
from image_proc_msgs.msg import ImageProcMsg
from sensor_msgs.msg import Image
import baxter_external_devices
import baxter_dataflow
import threading
import time

from moveit_commander import conversions
from baxter_core_msgs.srv import (SolvePositionIK,
                                  SolvePositionIKRequest)

rospy.init_node("control_node", anonymous=True)
robot = RobotInit()

########################################################################################################################
# robot.camera_open('left_hand_camera', exposure=10)   #
# robot.camera_open('right_hand_camera', exposure=10)  #
########################################################################################################################
# 初始化机器人参数
tf_listener = tf.TransformListener()
right_frame = 'right_hand_camera'
left_frame = 'left_hand_camera'

service_name = "Image_Process_A"
client = rospy.ServiceProxy(service_name, ImageProcSrv)

offset = np.array([0.05, 0.07, 0.1])

robot.limb_left.set_joint_position_speed(0.6)
robot.limb_right.set_joint_position_speed(0.6)
########################################################################################################################
# initial position
left_limb_joints = {'left_w0': 0.47480141124049735, 'left_w1': 1.2314566949554813, 'left_w2': 0.15372088238146325,
                    'left_e0': -1.107928713595913, 'left_e1': 1.7039025215086192, 'left_s0': 0.5938655515377337,
                    'left_s1': -1.0688748478697863}
right_limb_joints = {'right_s0': -0.14544413158371697, 'right_s1': -1.2990578089692, 'right_w0': -0.12480716836529432,
                     'right_w1': 1.1338270746789818, 'right_w2': -0.45249113715377826, 'right_e0': 0.4325969271093139,
                     'right_e1': 1.7651020359794947}


########################################################################################################################


def getTfFromMatrix(matrix):
    scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(matrix)
    return trans, tf.transformations.quaternion_from_euler(*angles), angles


def lookupTransform(tf_listener, target, source):
    tf_listener.waitForTransform(target, source, rospy.Time(), rospy.Duration(4.0))

    trans, rot = tf_listener.lookupTransform(target, source, rospy.Time())
    euler = tf.transformations.euler_from_quaternion(rot)

    source_target = tf.transformations.compose_matrix(translate=trans, angles=euler)
    return source_target


# 定义速度
########################################################################################################################
# 定义速度
baxter_interface.Limb('left').set_joint_position_speed(0.6)
baxter_interface.Limb('right').set_joint_position_speed(0.6)

look_cup_height = 0.2  # check
pick_height = -0.08  # check#check

# 左臂初始位置
l_initial_position = [0.4, 0.54, look_cup_height, 0, 1, 0, 0]  # check
l_pick_position = [0.45, 0.42, 0.2, 0.48, 0.556, -0.472, 0.485]  # check
# 右臂初始位置
r_initial_position = [0.4, -0.54, look_cup_height, 0, 1, 0, 0]  # check
r_pick_position = [0.4, -0.565, 0.2, -0.435, 0.577, 0.344, 0.598]  # check
# 定义倾倒角度
turn = [1.4, 1.46, 1.56]

'''
# 左臂初始位置
l_initial_position = {'left_w0': 0.7735098122912198, 'left_w1': 2.095034261054504, 'left_w2': -0.3294223741983926,
                      'left_e0': -2.4417139191166073, 'left_e1': 2.0885148427059907, 'left_s0': 0.7125340759727747,
                      'left_s1': -0.31676703269833795}

# 右臂初始位置
r_initial_position = [0.789, -0.565, 0.157, -0.027, 0.705, -0.052, 0.706]
'''


# bottle observe height 67cm
########################################################################################################################
# 1 到达初始位置
def go_to_initial_pose_1():
    try:
        t1 = threading.Thread(target=baxter_interface.Limb('left').move_to_joint_positions, args=(left_limb_joints,))
        t2 = threading.Thread(target=baxter_interface.Limb('right').move_to_joint_positions, args=(right_limb_joints,))

        robot.gripper_control('left', 100)
        robot.gripper_control('right', 100)

        t1.start()
        t2.start()
        t1.join()
        t2.join()
    except Exception:
        rospy.loginfo("go_to_initial_pose_1")


########################################################################################################################
# 2 夹爪获取瓶子

# 6
def from_head_to_cup_above(head_method, classify='cup', don_move="no"):
    # 向服务器请求所需物体的相机坐标
    client.wait_for_service()
    request = ImageProcSrvRequest()
    request.method = head_method  # TODO_edit
    response = client.call(request)

    position = np.array(response.object[0].position)
    print("camera tf: ", position)
    position = np.asarray(robot.camera_to_base_tf(position))
    print("tf position: ", position)


    if (position[1] >= 0):
        gripper = 'left'
        frame = left_frame
        if (classify == 'cup'):
            hand_method = 9
        else:
            hand_method = 11  # TODO
    else:
        gripper = 'right'
        frame = right_frame
        if (classify == 'cup'):
            hand_method = 10
        else:
            hand_method = 12  # TODO

    if(don_move == 'yes'):
        return position

    # head_camera 把头部相机的坐标转换为基座坐标  offset 移动的补偿 tag 到达一个固定的高度
    robot.baxter_ik_move(gripper, position, offset=offset, tag='taskC')

    client.wait_for_service()
    request = ImageProcSrvRequest()
    request.method = hand_method  # TODO_edit
    response = client.call(request)

    # 判断返回的数据是否合法
    position = np.array(response.object[0].position)
    position = np.append(position, 1)
    base_marker = lookupTransform(tf_listener, frame, '/base')
    trans_baxter, rot, rot_euler = getTfFromMatrix(np.linalg.inv(base_marker))
    translation_matrix = robot.quaternion_to_trans_matrix(rot[0], rot[1], rot[2], rot[3],
                                                          trans_baxter[0], trans_baxter[1], trans_baxter[2])

    base_tf = np.dot(translation_matrix, position)[0:3]
    print("base_tf: ", base_tf)
    return base_tf


def gripper_to_get_the_cup_2(don_move):
    cup_base_tf = from_head_to_cup_above(6, 'cup', don_move)  # detect cup
    bottle_base_tf = from_head_to_cup_above(7, 'bottle', don_move)
    # bottle_base_tf = np.array([0.6,0.187,0.07])
    print("cup_base_tf", cup_base_tf)
    print("bottle_base_tf", bottle_base_tf)
    if (don_move == 'no'):
        return cup_base_tf, bottle_base_tf
    else:
        return bottle_base_tf



#######################################################################################################################


########################################################################################################################
# 3 到达预订位置
def to_initial_pose_1():
    # baxter_interface.Limb('left').move_to_joint_positions(l_initial_position)
    robot.baxter_ik_move('left', l_initial_position[0:3], l_initial_position[3:7])
    robot.baxter_ik_move('right', r_initial_position[0:3], r_initial_position[3:7])


########################################################################################################################
########################################################################################################################
# 判断裁判瓶子是否已经静止
def if_move(move_list):  # 判断是否静止 #这里的列表是
    if abs(move_list[-1] - move_list[-3]) < 0.05 or abs(move_list[-2] - move_list[-4]) < 0.05:
        ifmove = 0
    else:
        ifmove = 1
    return ifmove


# 判断裁判瓶子是否到达倾倒位置
def if_pour(pour_list):  # 判断是否是倾倒位置
    if abs(pour_list[-1] - pour_list[-3]) < 0.05 or abs(pour_list[-2] - pour_list[-4]) < 0.05:
        ifpour = 0
    else:
        ifpour = 1
    return ifpour


def pick_up_cup(limb, hand_cup_x, hand_cup_y, pick_pose):
    if (limb == 'right'):
        bx = 0
        by = 0.025
    else:
        bx = -0.01
        by = -0.085
    x = pick_pose[0]
    y = pick_pose[1]
    robot.baxter_ik_move(limb, pick_pose[0:3], pick_pose[3:7])  # 回到抓取的初始位置
    pick_pose[0:2] = [hand_cup_x + bx, hand_cup_y + by]
    robot.baxter_ik_move(limb, pick_pose[0:3], pick_pose[3:7])  # 回到杯子抓取位置
    pick_pose[2] = pick_height
    robot.baxter_ik_move(limb, pick_pose[0:3], pick_pose[3:7])  # down
    robot.gripper_control(limb, 27.5)
    pick_pose[2] = pick_pose[2] + 0.2
    robot.baxter_ik_move(limb, pick_pose[0:3], pick_pose[3:7])  # up
    pick_pose[0] = x
    pick_pose[1] = y
    robot.baxter_ik_move(limb, pick_pose[0:3], pick_pose[3:7])  # 回到抓取的初始位置


########################################################################################################################
########################################################################################################################

def taskC():
    go_to_initial_pose_1()
    don_move = "no"
    cup_base_tf, bottle_base_tf = gripper_to_get_the_cup_2(don_move)
    if cup_base_tf[1]>bottle_base_tf[1]:
        limb='left'
        limb1 = 'right'
        ini_joint = right_limb_joints
        pick_pose = l_pick_position
    else:
        limb='right'
        limb1 = 'left'
        ini_joint = left_limb_joints
        pick_pose = r_pick_position
    # ######Step2 pick the cup
    baxter_interface.Limb(limb1).move_to_joint_positions(ini_joint)
    pick_up_cup(limb, cup_base_tf[0], cup_base_tf[1], pick_pose)
    #######step3 开始三次倾倒
    ifmove = 1
    ifpour = 0
    #bias = [0.055, 0, 0.035]
    bias = [0.055, 0, 0.135]
    j = 0
    # 定义两个列表
    move_list = [0, 0]  # 每隔三秒读取的瓶子xy坐标
    pour_list = [0, 0]  # 静止的xy坐标
    don_move = "yes"
    while (ifmove):
        bottle_base_tf = gripper_to_get_the_cup_2(don_move)
        bottle_pos = [bottle_base_tf[0], bottle_base_tf[1], bottle_base_tf[2]]
        move_list = move_list + bottle_pos[0:2]
        ifmove = if_move(move_list)
        if ifmove == 1:
            continue
        else:
            pour_list = pour_list + bottle_pos[0:2]
            ifpour = if_pour(pour_list)
            if ifpour == 1:
                pick_pose[0:3] = [bottle_pos[0] + bias[0], bottle_pos[1] + bias[1], bottle_pos[2] + bias[2]]
                robot.baxter_ik_move(limb, pick_pose[0:3], pick_pose[3:7])
                angel = baxter_interface.Limb('right').joint_angles()
                angel['right_w2'] = angel['right_w2'] + turn[j]  # fall
                baxter_interface.Limb('right').move_to_joint_positions(angel)
                robot.baxter_ik_move(limb, pick_pose[0:3], pick_pose[3:7])
                j = j + 1
                if j < 3:
                    ifmove = 1
                    time.sleep(2)
                else:
                    ifmove = 0
            else:
                ifmove = 1
                time.sleep(2)

    # 结束



taskC()


########################################################################################################################
def task(rpose):  # check how to send photo and recieve the data, and set the time
    client.wait_for_service()
    request = ImageProcSrvRequest()
    request.method = 0
    print(request)
    try:
        response = client.call(request)
        if isinstance(response, ImageProcSrvResponse):
            # print response.object[0].position[0]
            position = np.asarray(response.object[0].position)
            offset = np.array([0.05, 0.07, 0.1])
            rposition = robot.baxter_ik_move('right', position, offset=offset)  # rposition[0]:x;rposition[1]:y
            # print(rposition)
            rpose[0] = rposition[0]
            rpose[1] = rposition[1]
    except Exception:
        rospy.loginfo("arg error")
    return rpose
