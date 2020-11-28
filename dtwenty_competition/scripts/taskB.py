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

offset = np.array([0.05, 0.05, 0.1])

robot.limb_left.set_joint_position_speed(0.6)
robot.limb_right.set_joint_position_speed(0.6)


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
        self.classify = classify
        self.grip_method = grip_method

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
        return object.classify

    def print_objs(self):
        for i in range(len(self.object_list)):
            print(self.object_list[i].print_object())

    def sort(self):
        self.object_list.sort(key=self.cmp)

    def assign_task(self):
        self.sort()
        size = len(self.object_list)
        able_observe_pose = size
        base_pose = size
        able_observe_judge = True
        base_judge = True

        for i in range(size):
            # print(self.object_list[i].position, self.object_list[i].classify)
            if self.object_list[i].classify > -1 and able_observe_judge:
                able_observe_judge = False
                able_observe_pose = i
            if self.object_list[i].classify > 3 and base_judge:
                base_judge = False
                base_pose = i

        return able_observe_pose, base_pose, size, self.object_list


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
    request.method = 13
    # rospy.loginfo(request)

    response = client.call(request)
    if isinstance(response, ImageProcSrvResponse):
        obj_list = object_sort(response)
        # obj_list.print_objs()

        able_observe_pose, base_pose, size, object_list = obj_list.assign_task()
        print(able_observe_pose, base_pose)
        print('----------------------------------------------')
        for i in range(able_observe_pose):
            object_list[i].print_object()
        print('----------------------------------------------')
        for i in range(able_observe_pose, base_pose):
            object_list[i].print_object()
        print('----------------------------------------------')
        for i in range(base_pose, size):
            object_list[i].print_object()
        print('----------------------------------------------')

        rospy.loginfo("multi_object_recog_2 finish")
        return able_observe_pose, base_pose, size, object_list


########################################################################################################################

########################################################################################################################
# TODO
def use_hand_to_grip_3(gripper, pre_position, grip_method, need_judge='no'):  # gripper: 'left'/'right'
    rospy.loginfo("use_hand_to_grip_3 start")
    # robot.baxter_ik_move(gripper, pre_position, offset=offset, tag='taskB')
    robot.baxter_ik_move(gripper, pre_position, offset=offset, tag='taskB')

    client.wait_for_service()
    request = ImageProcSrvRequest()
    if (gripper == 'left'):
        if (grip_method == 0):
            request.method = 1
        else:
            request.method = 4
        frame = left_frame
        limb = robot.limb_left
        joint = 'left_w2'
    else:
        if (grip_method == 0):
            request.method = 2
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
        if grip_method == 1 and need_judge:
            theta = (theta + 90) % 180
        if theta == 0:
            theta = 180

        rotate_theta = robot.cal_gripper_theta(theta, gripper)
        position = np.asarray(response.object[0].position) / 1000
        position = np.append(position, 1)
        base_marker = lookupTransform(tf_listener, frame, '/base')
        trans_baxter, rot, rot_euler = getTfFromMatrix(np.linalg.inv(base_marker))
        translation_matrix = robot.quaternion_to_trans_matrix(rot[0], rot[1], rot[2], rot[3],
                                                              trans_baxter[0], trans_baxter[1], trans_baxter[2])

        base_tf = np.dot(translation_matrix, position)[0:3]

        robot.baxter_ik_move(gripper, base_tf, tag='taskB', joint_w2_move='yes', joint_w2=rotate_theta)

        # # get current end-joint's eular
        # quaternion = np.asarray(limb.endpoint_pose()["orientation"])
        # eular = np.asarray(
        #     tf.transformations.euler_from_quaternion([quaternion[0], quaternion[1], quaternion[2], quaternion[3]]))

        base_tf[2] = -0.142  # Large gripper
        # base_tf[2] = -0.183  # Small gripper
        # print(base_tf)

        robot.baxter_ik_move(gripper, base_tf, joint_w2_move='yes', joint_w2=rotate_theta)
        # robot.baxter_ik_move(gripper, base_tf, eular)
        robot.gripper_control(gripper, 0)

        time.sleep(0.5)
        base_tf[2] = 0
        robot.baxter_ik_move(gripper, base_tf)
        rospy.loginfo("use_hand_to_grip_3 finish")


########################################################################################################################
# same to A(only copy)
r_left_observation = {'left_w0': -1.2409904573992998, 'left_w1': 1.1711943315505133, 'left_w2': -0.22281070944035636,
                      'left_e0': -0.5875146417601135, 'left_e1': 1.4687866044002837, 'left_s0': -0.5227039534719548,
                      'left_s1': -0.5790777474267437}
r_right_observation = {'right_s0': 0.49010686172938966, 'right_s1': -1.2080098704597633, 'right_w0': 1.5274613695369008,
                       'right_w1': 1.5888206010523174, 'right_w2': 0.4506068564413403, 'right_e0': 0.27458256103148904,
                       'right_e1': 1.9285973455689365}

l_left_observation = {'left_w0': -0.7274903886546574, 'left_w1': 0.8759030298825713, 'left_w2': -0.7179030087303736,
                      'left_e0': -1.3901700890211561, 'left_e1': 1.7901555794622779, 'left_s0': 0.16758740107648146,
                      'left_s1': -0.5518495884417776}

l_right_observation = {'right_s0': 0.4601942363656241, 'right_s1': -0.5027622032294443, 'right_w0': 1.0837574266410448,
                       'right_w1': 1.015495281580144, 'right_w2': -0.06059224112147384, 'right_e0': 0.6695826139119831,
                       'right_e1': 1.3943885361878412}


def go_to_observe_position(gripper):
    if gripper == 'left':
        front = 'left'
        back = 'right'
        front_observation = l_left_observation
        back_observation = l_right_observation
    else:
        front = 'right'
        back = 'left'
        front_observation = r_right_observation
        back_observation = r_left_observation
    try:
        t1 = threading.Thread(target=baxter_interface.Limb(front).move_to_joint_positions, args=(front_observation,))
        # baxter_interface.Limb(front).move_to_joint_positions(front_observation)
        t2 = threading.Thread(target=baxter_interface.Limb(back).move_to_joint_positions, args=(back_observation,))
        # baxter_interface.Limb(back).move_to_joint_positions(back_observation)
        t1.start()
        time.sleep(0.3)
        t2.start()
        t1.join()
        t2.join()

        return back, back_observation
    except Exception:
        rospy.loginfo("go_to_observe_position")


def observe_at_the_position_5(gripper):
    back, back_observation = go_to_observe_position(gripper)

    client.wait_for_service()
    request = ImageProcSrvRequest()
    request.method = 8

    response = client.call(request)
    classify = response.object[0].position[0]

    if (back == 'left'):
        back_observation = left_limb_joints
    else:
        back_observation = right_limb_joints
    t1 = threading.Thread(target=baxter_interface.Limb(back).move_to_joint_positions, args=(back_observation,))
    t1.start()

    return classify


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
#
# move to put area
put_place_left = np.array([0.8134, 0.4581, -0.14, 0.0074, 0.99961, 0.0173, -0.0201])
put_place_left_1 = copy.copy(put_place_left)
put_place_left_1[0] -= 0.12
put_place_left_2 = copy.copy(put_place_left)
put_place_left_2[0] -= 0.12 * 2
put_place_left_3 = copy.copy(put_place_left)
put_place_left_3[0] -= 0.12 * 3
put_place_left_all = [put_place_left, put_place_left_1, put_place_left_2, put_place_left_3]

put_place_right = np.array([0.8201, -0.461, -0.14, -0.0182, 0.9989, -0.0362, 0.021])
put_place_right_1 = copy.copy(put_place_right)
put_place_right_1[0] -= 0.12
put_place_right_2 = copy.copy(put_place_right)
put_place_right_2[0] -= 0.12 * 2
put_place_right_3 = copy.copy(put_place_right)
put_place_right_3[0] -= 0.12 * 3
put_place_right_all = [put_place_right, put_place_right_1, put_place_right_2, put_place_right_3]

preset_put_place = np.array([put_place_left, put_place_left_1, put_place_left_2, put_place_left_3,
                             put_place_right, put_place_right_1, put_place_right_2, put_place_right_3])

area = 8


def area_map(classify):
    # TODO
    return classify


def get_position_and_need_exchanges(classify, gripper):
    idx = classify
    # print(classify)

    need_exchange = False

    if classify >= area / 2:
        if gripper == 'left':
            need_exchange = True
    else:
        if gripper == 'right':
            need_exchange = True

    print(idx, need_exchange)
    return idx, need_exchange


def move_workpiece_to_put_place_6(gripper, classify, grip_method):
    rospy.loginfo("move_workpiece_to_put_place_6 start")
    classify = area_map(classify)

    idx, need_exchange = get_position_and_need_exchanges(classify, gripper)

    if need_exchange:
        if grip_method:
            method = 'vertical'
        else:
            method = 'horizontal'
        if gripper == 'left':
            direction = 'left2right'
            gripper = 'right'
        else:
            direction = 'right2left'
            gripper = 'left'

        exchange_gripper_5(direction, method)

    put_place = preset_put_place[idx]
    print("classify: ", classify, "place id: ", idx, "gripper to the position: ", put_place)

    try:
        position = np.array([put_place[0], put_place[1], put_place[2]])
        eular = np.asarray(
            tf.transformations.euler_from_quaternion([put_place[3], put_place[4], put_place[5], put_place[6]]))
        orientation = [put_place[3], put_place[4], put_place[5], put_place[6]]

        preset_height = 0.10
        preset_position = copy.copy(position)
        preset_position[2] = preset_height
        robot.baxter_ik_move(gripper, preset_position, orientation)

        robot.baxter_ik_move(gripper, position, orientation)
        robot.gripper_control(gripper, 100)

        robot.baxter_ik_move(gripper, preset_position, orientation)

        rospy.loginfo("move_workpiece_to_put_place_6 finish")
    except Exception:
        rospy.loginfo("move_workpiece_to_put_place_6 error")


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

# def adjust_base(gripper, base_classify, method, pre_position):
#     use_hand_to_grip_3(gripper, pre_position)
#     if method == 1 or method == 3:
#         pass
########################################################################################################################
safe_area = 0.02


def gripper_judge(x, y):
    y_ = 1.913 * x * x - 1.422 * x - 0.1771 + safe_area
    if (y > y_):
        return 'left'
    else:
        return 'right'


########################################################################################################################
#
def taskB():
    go_to_initial_pose_1()
    # save_img()
    able_observe_pose, base_pose, size, object_list = multi_object_recog_2()

    for i in range(base_pose):
        print("----------------PICK WOCKPIECE START----------------")
        gripper_task = object_list[i]
        gripper = gripper_judge(gripper_task.position[0], gripper_task.position[1])
        if (object_list[i].classify == 2):
            use_hand_to_grip_3(gripper, gripper_task.position, gripper_task.grip_method, need_judge='yes')
        else:
            use_hand_to_grip_3(gripper, gripper_task.position, gripper_task.grip_method)
        if (gripper_task.classify == -1):
            gripper_task.classify = observe_at_the_position_5(gripper)
        move_workpiece_to_put_place_6(gripper, gripper_task.classify, gripper_task.grip_method)
        print("-----------------PICK WOCKPIECE END-----------------")

    for i in range(base_pose, size):
        print("-------------------PICK BASE START------------------")

        print("-------------------PICK BASE END------------------")

    # for i in range(left_task_num):
    #     gripper = 'left'
    #     gripper_task = left_gripper_task[i]
    #
    #     print("---------------------------")
    #     print("left gripper start")
    #
    #     if(gripper_task.classify == 2):
    #         use_hand_to_grip_3(gripper, gripper_task.position, gripper_task.grip_method, need_judge='yes')
    #     else:
    #         use_hand_to_grip_3(gripper, gripper_task.position, gripper_task.grip_method)
    #     if (gripper_task.classify == -1):
    #         gripper_task.classify = observe_at_the_position_5(gripper)
    #     #
    #     # move_to_put_place_6(gripper, gripper_task.classify, gripper_task.grip_method)
    #     print("left gripper end")
    #     print("---------------------------")
    #
    # t1 = threading.Thread(target=baxter_interface.Limb('left').move_to_joint_positions, args=(left_limb_joints,))
    # t1.start()
    # robot.gripper_control('left', 100)
    #
    # for i in range(right_task_num):
    #     gripper = 'right'
    #     gripper_task = right_gripper_task[i]
    #
    #     print("---------------------------")
    #     print("right gripper start")
    #
    #     if (gripper_task.classify == 2):
    #         use_hand_to_grip_3(gripper, gripper_task.position, gripper_task.grip_method, need_judge='yes')
    #     else:
    #         use_hand_to_grip_3(gripper, gripper_task.position, gripper_task.grip_method)
    #     if (gripper_task.classify == -1):
    #         gripper_task.classify = observe_at_the_position_5(gripper)
    #     # else:
    #     #     move_to_put_place_6(gripper, gripper_task.classify, gripper_task.grip_method)
    #     print("right gripper end")
    #     print("---------------------------")


########################################################################################################################


taskB()

#######################################################################################################################
# go_to_observe_position('left')

# robot.go_to_initial_pose()

# position = np.array([0.7380, -0.1261, 0.16])
# use_hand_to_grip_3('right', position)

#
# print(robot.get_joint())

# print(robot.limb_right.joint_angle('right_w2'))
# print(robot.limb_right.joint_angle('right_w2'))
