#! /usr/bin/env python

import tf
import copy
import time
import rospy
import threading

from robot_utils import *

import baxter_interface
import baxter_external_devices

from sensor_msgs.msg import Image
from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest)
from image_proc_msgs.srv import ImageProcSrv, ImageProcSrvRequest, ImageProcSrvResponse

from moveit_commander import conversions

########################################################################################################################
# robot initial
rospy.init_node("control_node", anonymous=True)
robot = RobotInit()

## just open once
########################################################
robot.camera_open('left_hand_camera', exposure=10)   #
robot.camera_open('right_hand_camera', exposure=10)  #
########################################################

right_frame = 'right_hand_camera'
left_frame = 'left_hand_camera'
service_name = "Image_Process_A"

tf_listener = tf.TransformListener()
client = rospy.ServiceProxy(service_name, ImageProcSrv)

offset = np.array([0.05, 0.07, 0.1])

robot.limb_left.set_joint_position_speed(1)
robot.limb_right.set_joint_position_speed(1)
#######################################################################################################################
# position for placing
put_place_left = np.array([0.8134, 0.4581, -0.1534, 0.0074, 0.99961, 0.0173, -0.0201])
put_place_left_1 = copy.copy(put_place_left)
put_place_left_1[0] -= 0.12
put_place_left_2 = copy.copy(put_place_left)
put_place_left_2[0] -= 0.12 * 2
put_place_left_3 = copy.copy(put_place_left)
put_place_left_3[0] -= 0.12 * 3

put_place_right = np.array([0.8201, -0.5234, -0.1523, 0.0074, 0.99961, 0.0173, -0.0201])
put_place_right_1 = copy.copy(put_place_right)
put_place_right_1[0] -= 0.12
put_place_right_2 = copy.copy(put_place_right)
put_place_right_2[0] -= 0.12 * 2
put_place_right_3 = copy.copy(put_place_right)
put_place_right_3[0] -= 0.12 * 3

put_place = np.array([put_place_left, put_place_left_1, put_place_left_2, put_place_left_3,
                      put_place_right, put_place_right_1, put_place_right_2, put_place_right_3])


# observation position
left_observation =  {'left_w0': -1.000538968898261, 'left_w1': 1.1593059804444015, 'left_w2': -0.13230584295511694,
                     'left_e0': -0.6247136758663347, 'left_e1': 1.6248691495676244, 'left_s0': -0.11083011192472114,
                     'left_s1': -0.38157772098649667}

right_observation = {'right_s0': -0.22204371904641365, 'right_s1': -0.21820876707670012, 'right_w0': 0.336325287743877,
                     'right_w1': 1.3871021274453854, 'right_w2': 0.5506991028508635, 'right_e0': 0.9108010928069644,
                     'right_e1': 1.268218616384266}

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


# all above is to initial robot
########################################################################################################################
def save_img():
    client.wait_for_service()
    request = ImageProcSrvRequest()
    request.method = -1
    # print(request)
    # try:
    response = client.call(request)
    if isinstance(response, ImageProcSrvResponse):
        print("saved")

# collect image
########################################################################################################################
class object:
    def __init__(self, position, grip_method, classify):
        self.position = position
        self.grip_method = grip_method
        self.classify = classify

    def print_object(self):
        print(self.position, self.grip_method, self.classify)


class object_sort:
    def __init__(self, response):
        self.object_list = []
        if response.is_success:
            length = len(response.grip_method)
            for i in range(length):
                response.object[i].position = robot.camera_to_base_tf(np.asarray(response.object[i].position))
                obj = object(response.object[i].position, response.grip_method[i], response.classify[i])
                self.object_list.append(obj)
        else:
            pass

    def cmp(self, object):
        return object.classify[0]

    def print_objs(self):
        for i in range(len(self.object_list)):
            print(self.object_list[i].print_object())

    def sort(self):
        self.object_list.sort(key=self.cmp)

    def assign_task(self):
        self.sort()
        divide_pos = 0
        for i in range(len(self.object_list)):
            if self.object_list[i].position[1] >= 0:
                divide_pos = i
                break
        return len(self.object_list) - divide_pos, divide_pos, \
               self.object_list[divide_pos:len(self.object_list)], self.object_list[0:divide_pos],

# assign task
########################################################################################################################
# 1 go to the default pose
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
# 2 recognize all the object and assign task
def multi_object_recog_2():
    client.wait_for_service()
    request = ImageProcSrvRequest()
    request.method = 3
    rospy.loginfo(request)

    try:
        response = client.call(request)
        if isinstance(response, ImageProcSrvResponse):

            obj_list = object_sort(response)
            obj_list.print_objs()

            left_task_num, right_task_num, left_gripper_task, right_gripper_task = obj_list.assign_task()
            # for i in range(left_task_num):
            #     left_gripper_task[i].print_object()
            #
            # for i in range(right_task_num):
            #     right_gripper_task[i].print_object()
            return left_task_num, right_task_num, left_gripper_task, right_gripper_task

    except Exception:
        rospy.loginfo("multi_object_recog_2")
########################################################################################################################
# _ use hand to detect and grip
def use_hand_to_grip(gripper): # gripper: 'left'/'right'
    client.wait_for_service()
    request = ImageProcSrvRequest()
    if(gripper == 'left'):
        request.method = 1
        frame = left_frame
        limb = robot.limb_left
        joint = 'left_w2'
    elif(gripper == 'right'):
        request.method = 2
        frame = right_frame
        limb = robot.limb_right
        joint = 'right_w2'

    response = client.call(request)
    try:
        if isinstance(response, ImageProcSrvResponse):
            theta = response.object[0].orientation[0]
            # print(theta)
            if theta == 0:
                theta = 180
            rotate_theta = robot.cal_gripper_theta(response.object[0].orientation[0])
            position = np.asarray(response.object[0].position) / 1000
            position = np.append(position, 1)
            base_marker = lookupTransform(tf_listener, frame, '/base')
            trans_baxter, rot, rot_euler = getTfFromMatrix(np.linalg.inv(base_marker))
            translation_matrix = robot.quaternion_to_trans_matrix(rot[0], rot[1], rot[2], rot[3],
                                                                  trans_baxter[0], trans_baxter[1], trans_baxter[2])

            base_tf = np.dot(translation_matrix, position)[0:3]
            # print(base_tf)
            robot.baxter_ik_move(gripper, base_tf, tag='solid')
            robot.set_j(limb, joint, rotate_theta, flag='pure')

            # get current end-joint'eular
            quaternion = np.asarray(limb.endpoint_pose()["orientation"])
            eular = np.asarray(
                tf.transformations.euler_from_quaternion([quaternion[0], quaternion[1], quaternion[2], quaternion[3]]))

            base_tf[2] = -0.178
            # print(base_tf)
            robot.baxter_ik_move(gripper, base_tf, rpy=eular)
            robot.gripper_control(gripper, 0)

            time.sleep(1)
            base_tf[2] = 0
            robot.baxter_ik_move(gripper, base_tf, rpy=eular)
    except Exception:
        rospy.loginfo("use_hand_to_grip")
########################################################################################################################
# exchange gripper if need
left_vertical = [0.5935, -0.0654, 0.253, 0.036, -0.692, 0.7183, -0.053]
right_vertical = [0.5798, 0.0532, 0.265, 0.4421, 0.5556, 0.5003, -0.4964]

left_horizontal = [0.5935, -0.0654, 0.243, 0.5342, -0.4895, 0.4801, 0.4934]
right_horizontal = [0.5798, 0.0532, 0.265, 0.4421, 0.5556, 0.5003, -0.4964]

def exchange_gripper(direction, method):
    if (direction == 'left2right'):
        if method == 'vertical':
            give_gripper = left_vertical
            receive_gripper = right_vertival
            give = 'left'
            receive = 'right'
        elif method == 'horizontal':
            give_gripper = left_horizontal
            receive_gripper = right_horizontal
            give = 'left'
            receive = 'right'
    elif (direction == 'right2left'):
        if method == 'vertical':
            give_gripper = right_vertical
            receive_gripper = left_vertical
            give = 'right'
            receive = 'left'
        elif method == 'horizontal':
            give_gripper = right_horizontal
            receive_gripper = left_horizontal
            give = 'right'
            receive = 'left'

    position = np.array(give_gripper[0:3])
    eular = np.asarray(
        tf.transformations.euler_from_quaternion(give_gripper[3:7]))
    print(position, eular)
    robot.baxter_ik_move(give, position, rpy=eular)

    _pre = copy.copy(receive_gripper)
    if (direction == 'left2right'):
        _pre[1] -= 0.1
    elif (direction == 'right2left'):
        _pre[1] += 0.1
    position = np.array(_pre[0:3])
    eular = np.asarray(
        tf.transformations.euler_from_quaternion(_pre[3:7]))
    robot.baxter_ik_move(receive, position, eular)

    time.sleep(1)

    position = np.array(receive_gripper[0:3])
    eular = np.asarray(tf.transformations.euler_from_quaternion(receive_gripper[3:7]))
    robot.baxter_ik_move(receive, position, eular)
    robot.gripper_control(receive, 0)
    robot.gripper_control(give, 100)
########################################################################################################################
def go_to_observe_position(left='', right=''):
    try:
        if(left == 'true'):
            baxter_interface.Limb('left').move_to_joint_positions(left_observation)
        if(right == 'true'):
            baxter_interface.Limb('right').move_to_joint_positions(right_observation)
    except Exception:
        rospy.loginfo("go_to_observe_position")
########################################################################################################################

def head_recognition():
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
            rpy = np.array([-3.14, 0, -3.14])
            robot.baxter_ik_move('right', position, head_camera='yes', offset=offset, tag='solid')

            # print(position)
    except Exception:
        rospy.loginfo("arg error")



def head_recognition_left():
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
            rpy = np.array([-3.14, 0, -3.14])
            robot.baxter_ik_move('left', position, head_camera='yes', offset=offset, tag='solid')

            # print(position)
    except Exception:
        rospy.loginfo("arg error")


def execute_tasks(gripper, gripper_tasks):
    for i in range(len(gripper_tasks)):
        robot.baxter_ik_move(gripper, gripper_tasks[i].position, offset=offset, tag='solid')




# go_to_observe_position(left='true', right='true')

# save_img()

go_to_initial_pose_1()
head_recognition()
use_hand_to_grip('right')
exchange_gripper(direction='right2left', method='vertical')




# exchange_right2left_horizontal()
# #
#
# single_left
# head_recognition_left()
# task3_left()
# exchange_gripper(direction='left2right',method='horizontal')
# exchange_left2right_horizontal()
#

# # save last gripper joint
# position = np.array([0.747341955, -0.16724122, -0.01])
# quaternion = np.asarray(robot.limb_right.endpoint_pose()["orientation"])
# print(quaternion[3], quaternion[0], quaternion[1], quaternion[2])
# eular = np.asarray(tf.transformations.euler_from_quaternion([quaternion[0], quaternion[1], quaternion[2], quaternion[3]]))
# eular = np.array([3.14054553e+00, -5.85404174e-04, -3.13926017e+00])
# print(eular)
# robot.baxter_ik_move('right', position, rpy=eular)
#

# print(robot.get_joint())
# print(robot.limb_left.joint_names())
# print(robot.limb_right.joint_names())

# robot.gripper_control('right', 0)
# time.sleep(2)
# robot.gripper_control('right', 100)

# multi_task

# task_4()
# task_4_test()
# task_5()

# dict = {'right_w2': 4.0}
#
# cnt = 5
# while cnt:
#     robot.limb_right.set_joint_velocities(dict)
#     # robot.limb_right.set_joint_torques(dict)
#     cnt -= 1
#     time.sleep(0.1)