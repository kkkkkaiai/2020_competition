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

tf_listener = tf.TransformListener()
right_frame = 'right_hand_camera'
left_frame = 'left_hand_camera'

service_name = "Image_Process_A"
client = rospy.ServiceProxy(service_name, ImageProcSrv)

offset = np.array([0.05, 0.07, 0.1])

## just open once
########################################################
# robot.camera_open('left_hand_camera', exposure=10)   #
# robot.camera_open('right_hand_camera', exposure=10)  #
########################################################

def getTfFromMatrix(matrix):
    scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(matrix)
    return trans, tf.transformations.quaternion_from_euler(*angles), angles


def lookupTransform(tf_listener, target, source):
    tf_listener.waitForTransform(target, source, rospy.Time(), rospy.Duration(4.0))

    trans, rot = tf_listener.lookupTransform(target, source, rospy.Time())
    euler = tf.transformations.euler_from_quaternion(rot)

    source_target = tf.transformations.compose_matrix(translate=trans, angles=euler)
    return source_target

########################################################################################################################
#
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
        return object.position[1]

    def print_objs(self):
        for i in range(len(self.object_list)):
            print(self.object_list[i].print_object())

    def sort(self):
        self.object_list.sort(key=self.cmp)

    def assign_task(self):
        self.sort()
        divide_pos = len(self.object_list)
        for i in range(len(self.object_list)):
            if self.object_list[i].position[1] >= 0:
                divide_pos = i
                print(divide_pos)
                break
        return len(self.object_list) - divide_pos, divide_pos, \
               self.object_list[divide_pos:len(self.object_list)], self.object_list[0:divide_pos]


# assign task
########################################################################################################################
# prepare data

left_limb_joints = {'left_w0': 0.47480141124049735, 'left_w1': 1.2314566949554813, 'left_w2': 0.15372088238146325,
                    'left_e0': -1.107928713595913, 'left_e1': 1.7039025215086192, 'left_s0': 0.5938655515377337,
                    'left_s1': -1.0688748478697863}
right_limb_joints = {'right_s0': -0.14544413158371697, 'right_s1': -1.2990578089692, 'right_w0': -0.12480716836529432,
                     'right_w1': 1.1338270746789818, 'right_w2': -0.45249113715377826, 'right_e0': 0.4325969271093139,
                     'right_e1': 1.7651020359794947}


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

def multi_object_recog_2():
    rospy.loginfo("multi_object_recog_2 start")
    client.wait_for_service()
    request = ImageProcSrvRequest()
    request.method = 3
    # rospy.loginfo(request)

    response = client.call(request)
    if isinstance(response, ImageProcSrvResponse):
        obj_list = object_sort(response)
        # obj_list.print_objs()

        left_task_num, right_task_num, left_gripper_task, right_gripper_task = obj_list.assign_task()
        print(left_task_num, right_task_num)

        for i in range(left_task_num):
            left_gripper_task[i].print_object()

        for i in range(right_task_num):
            right_gripper_task[i].print_object()

        return left_task_num, right_task_num, left_gripper_task, right_gripper_task
    rospy.loginfo("multi_object_recog_2 finish")

########################################################################################################################

########################################################################################################################
#
def use_hand_to_grip_3(gripper, pre_position):  # gripper: 'left'/'right'
    rospy.loginfo("use_hand_to_grip_3 start")
    # robot.baxter_ik_move(gripper, pre_position, offset=offset, tag='taskB')
    robot.baxter_ik_move(gripper, pre_position, tag='taskB')

    client.wait_for_service()
    request = ImageProcSrvRequest()
    if (gripper == 'left'):
        request.method = 4
        frame = left_frame
        limb = robot.limb_left
        joint = 'left_w2'
    else:
        request.method = 5
        frame = right_frame
        limb = robot.limb_right
        joint = 'right_w2'

    response = client.call(request)
    # try:
    if isinstance(response, ImageProcSrvResponse):
        theta = response.object[0].orientation[0]
        # print(theta)
        # print(robot.get_joint())
        if theta == 0:
            theta = 180
        rotate_theta = robot.cal_gripper_theta(response.object[0].orientation[0], gripper)
        position = np.asarray(response.object[0].position) / 1000
        position = np.append(position, 1)
        base_marker = lookupTransform(tf_listener, frame, '/base')
        trans_baxter, rot, rot_euler = getTfFromMatrix(np.linalg.inv(base_marker))
        translation_matrix = robot.quaternion_to_trans_matrix(rot[0], rot[1], rot[2], rot[3],
                                                              trans_baxter[0], trans_baxter[1], trans_baxter[2])

        base_tf = np.dot(translation_matrix, position)[0:3]
        # print(base_tf)
        robot.baxter_ik_move(gripper, base_tf, tag='taskB')
        robot.set_j(limb, joint, rotate_theta, flag='pure')

        # get current end-joint's eular
        quaternion = np.asarray(limb.endpoint_pose()["orientation"])
        eular = np.asarray(
            tf.transformations.euler_from_quaternion([quaternion[0], quaternion[1], quaternion[2], quaternion[3]]))

        base_tf[2] = -0.178  # -0.183
        # print(base_tf)
        robot.baxter_ik_move(gripper, base_tf, eular)
        robot.gripper_control(gripper, 0)

        time.sleep(1)
        base_tf[2] = 0
        robot.baxter_ik_move(gripper, base_tf, eular)
        rospy.loginfo("use_hand_to_grip_3 finish")


########################################################################################################################
#
r_left_observation = {'left_w0': -1.4131798008394374, 'left_w1': 0.9986214929134043, 'left_w2': -0.18752915131899184,
                    'left_e0': -0.5518495884417776, 'left_e1': 1.309636097657172, 'left_s0': -0.8248981686853812,
                    'left_s1': -0.5330583237901813}
r_right_observation = {'right_s0': 0.3834951969713534, 'right_s1': -1.252111818111469,
                     'right_w0': 1.398606983354526, 'right_w1': 1.8311895655382127, 'right_w2': 0.16528642989465334,
                     'right_e0': 0.03029612056073692, 'right_e1': 1.975767254796413}

l_left_observation = {'left_w0': -0.8149272935641261, 'left_w1': 0.9690923627466101, 'left_w2': -0.44715539966859813,
                      'left_e0': -1.3330293046724246, 'left_e1': 1.8035779113562753, 'left_s0': 0.17065536265225228,
                      'left_s1': -0.5568350260024052}

l_right_observation = {'right_s0': 0.4663301595171658, 'right_s1': -0.4962427848809313, 'right_w0': 1.0933448065653286,
                       'right_w1': 1.013577805595287, 'right_w2': -0.056373793954788955, 'right_e0': 0.6722670802907825,
                       'right_e1': 1.3909370794150988}

def hand_to_observation(gripper):
    baxter_interface.Limb('left').move_to_joint_positions(left_observation)

    client.wait_for_service()
    request = ImageProcSrvRequest()
    request.method = 2
    # try:
    response = client.call(request)
    if isinstance(response, ImageProcSrvResponse):
        theta = response.object[0].orientation[0]
        print(theta)
        if theta == 0:
            theta = 180

        # print(rotate_theta)

########################################################################################################################
#
left_vertical = [0.5935, -0.0436, 0.250, 0.036, -0.692, 0.7183, -0.053]  # -0.0654
right_vertical = [0.5798, 0.0314, 0.265, 0.4421, 0.5556, 0.5003, -0.4964]  # 0.0532

left_horizontal = [0.5935, -0.0436, 0.240, 0.5342, -0.4895, 0.4801, 0.4934]  # -0.0654
right_horizontal = [0.5798, 0.0314, 0.265, 0.4421, 0.5556, 0.5003, -0.4964]  # 0.0532


def exchange_gripper_5(direction, method):  # direction: left2right/right2left; method: vertical/horizontal
    rospy.loginfo("exchange_gripper_4 start")

    if direction == 'left2right':
        if method == 'vertical':
            give_gripper = left_vertical
            receive_gripper = right_vertical
        else:
            give_gripper = left_horizontal
            receive_gripper = right_horizontal
        give = 'left'
        receive = 'right'
        limb_joints = left_limb_joints
    else:
        if method == 'vertical':
            give_gripper = right_vertical
            receive_gripper = left_vertical
        else:
            give_gripper = right_horizontal
            receive_gripper = left_horizontal
        give = 'right'
        receive = 'left'
        limb_joints = right_limb_joints

    # prepare the give gripper's pose imformation
    give_position = np.array(give_gripper[0:3])
    give_eular = np.asarray(
        tf.transformations.euler_from_quaternion(give_gripper[3:7]))
    # print(position, eular)

    _pre = copy.copy(receive_gripper)
    if direction == 'left2right':
        back_position = copy.copy(give_position)
        back_position[1] += 0.2  # axis back plus y
        # back_position[0] += 0.15
        _pre[1] -= 0.1
    else:
        back_position = copy.copy(give_position)
        back_position[1] -= 0.2  # axis back minus y
        # back_position[0] += 0.15
        _pre[1] += 0.1

    # receive gripper to the pre_pose
    position = np.array(_pre[0:3])
    eular = np.asarray(
        tf.transformations.euler_from_quaternion(_pre[3:7]))
    # robot.baxter_ik_move(receive, position, eular)
    t1 = threading.Thread(target=robot.baxter_ik_move, args=(receive, position, eular,))
    t1.start()

    robot.baxter_ik_move(give, give_position, give_eular)
    t1.join()
    time.sleep(0.4)

    # receive gripper to the pose
    position = np.array(receive_gripper[0:3])
    eular = np.asarray(tf.transformations.euler_from_quaternion(receive_gripper[3:7]))

    robot.baxter_ik_move(receive, position, eular)
    robot.gripper_control(receive, 0)
    time.sleep(0.2)
    robot.gripper_control(give, 100)

    # give_gripper back a distance
    robot.baxter_ik_move(give, back_position, give_eular)
    t2 = threading.Thread(target=baxter_interface.Limb(give).move_to_joint_positions, args=(limb_joints,))
    t2.start()

    position[2] += 0.1
    robot.baxter_ik_move(receive, position, eular)
    time.sleep(0.2)
    t2.join()
    rospy.loginfo("exchange_gripper_4 finish")


########################################################################################################################
# method 1 right2left

r2l_l_method1 = {'left_w0': -0.9491506125040997, 'left_w1': 1.1807817114747972, 'left_w2': -0.43104860139580126,
                'left_e0': -0.6622962051695274, 'left_e1': 1.9949420146449806, 'left_s0': 0.06442719309118737,
                'left_s1': -0.44485442848677}
r2l_r_method1 = {'right_s0': 0.27228158984966094, 'right_s1': -1.2241166687325602, 'right_w0': -0.3846456825622675,
                'right_w1': 1.5470196245824397, 'right_w2': 0.8179952551398969, 'right_e0': 1.4545972821123436,
                'right_e1': 1.5508545765521533}
l2r_l_method1 = {'left_w0': 0.2661456666981193, 'left_w1': 1.398606983354526, 'left_w2': -0.9503010980950138,
                 'left_e0': -1.4043594113090963, 'left_e1': 1.6459613854010489, 'left_s0': -0.16490293469768197,
                 'left_s1': -1.305417650490487}
l2r_r_method1 = {'right_s0': 0.09587379924283836, 'right_s1': -0.604388430426853, 'right_w0': 1.048092373322709,
                 'right_w1': 1.2455923997629559, 'right_w2': 0.44868938045648354, 'right_e0': 0.5909660985328556,
                 'right_e1': 1.9669468652660718}


#  0 round base
#  1 square base
#  2 flat square base
#  3 hex base

def adjust_base(gripper, base_classify, method, pre_position):
    use_hand_to_grip_3(gripper, pre_position)
    if method == 1 or method == 3:
        pass

















########################################################################################################################
# robot.go_to_initial_pose()

# position = np.array([0.7380, -0.1261, 0.16])
# use_hand_to_grip_3('right', position)


print(robot.get_joint())



#######################################################################################################################
# print(robot.limb_right.joint_angle('right_w2'))
# print(robot.limb_right.joint_angle('right_w2'))
