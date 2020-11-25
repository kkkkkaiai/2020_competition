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
baxter_interface.Limb('left').set_joint_position_speed(0.3)
baxter_interface.Limb('right').set_joint_position_speed(0.3)

# 左臂初始位置
l_initial_position = {'left_w0': 0.7735098122912198, 'left_w1': 2.095034261054504, 'left_w2': -0.3294223741983926,
                      'left_e0': -2.4417139191166073, 'left_e1': 2.0885148427059907, 'left_s0': 0.7125340759727747,
                      'left_s1': -0.31676703269833795}

# 右臂初始位置
r_initial_position = [0.789, -0.565, 0.157, -0.027, 0.705, -0.052, 0.706]


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
def from_head_to_cup_above(head_method, classify='cup'):
    # 向服务器请求所需物体的相机坐标
    client.wait_for_service()
    request = ImageProcSrvRequest()
    request.method = head_method  # TODO_edit
    response = client.call(request)

    position = np.array(response.object[0].position)
    position = robot.camera_to_base_tf(position)
    if(position[1] >= 0):
        gripper = 'left'
        frame = left_frame
        if(classify == 'cup'):
            hand_method = 9
        else:
            hand_method = 9 # TODO
    else:
        gripper = 'right'
        frame = right_frame
        if(classify == 'cup'):
            hand_method = 10
        else:
            hand_method = 10 # TODO

    gripper_tf = np.array([position[0], position[1], position[2]])
    print(position)
    print(robot.camera_to_base_tf(gripper_tf))
    # head_camera 把头部相机的坐标转换为基座坐标  offset 移动的补偿 tag 到达一个固定的高度
    robot.baxter_ik_move(gripper, gripper_tf, offset=offset, tag='taskC')

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

def gripper_to_get_the_cup_2():
    cup_base_tf = from_head_to_cup_above(6, 'cup')  # detect cup
    bottle_base_tf = from_head_to_cup_above(7, 'bottle')
    print("cup_base_tf", cup_base_tf)
    print("bottle_base_tf", bottle_base_tf)
#######################################################################################################################





########################################################################################################################
# 3 到达预订位置
def to_initial_pose_1():
    baxter_interface.Limb('left').move_to_joint_positions(l_initial_position)
    robot.baxter_ik_move('right', r_initial_position[0:3], r_initial_position[3:7])

########################################################################################################################


go_to_initial_pose_1()
gripper_to_get_the_cup_2()











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


