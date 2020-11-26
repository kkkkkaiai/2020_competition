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
# robot.camera_open('left_hand_camera', exposure=10)   #
# robot.camera_open('right_hand_camera', exposure=10)  #
########################################################

right_frame = 'right_hand_camera'
left_frame = 'left_hand_camera'
service_name = "Image_Process_A"

tf_listener = tf.TransformListener()
client = rospy.ServiceProxy(service_name, ImageProcSrv)

offset = np.array([0.05, 0.02, 0.0])
# offset = np.array([0.0, 0.0, 0.0])

robot.limb_left.set_joint_position_speed(0.5)
robot.limb_right.set_joint_position_speed(0.5)
#######################################################################################################################
# position for placing
put_place_left = np.array([0.8134, 0.4581, -0.13, 0.0074, 0.99961, 0.0173, -0.0201])
put_place_left_1 = copy.copy(put_place_left)
put_place_left_1[0] -= 0.12
put_place_left_2 = copy.copy(put_place_left)
put_place_left_2[0] -= 0.12 * 2
put_place_left_3 = copy.copy(put_place_left)
put_place_left_3[0] -= 0.12 * 3
put_place_left_all = [put_place_left, put_place_left_1, put_place_left_2, put_place_left_3]

put_place_right = np.array([0.8201, -0.461, -0.13, -0.0182, 0.9989, -0.0362, 0.021])
put_place_right_1 = copy.copy(put_place_right)
put_place_right_1[0] -= 0.12
put_place_right_2 = copy.copy(put_place_right)
put_place_right_2[0] -= 0.12 * 2
put_place_right_3 = copy.copy(put_place_right)
put_place_right_3[0] -= 0.12 * 3
put_place_right_all = [put_place_right, put_place_right_1, put_place_right_2, put_place_right_3]

preset_put_place = np.array([put_place_left, put_place_left_1, put_place_left_2, put_place_left_3,
                             put_place_right, put_place_right_1, put_place_right_2, put_place_right_3])

# observation position
# left_observation = {'left_w0': -1.000538968898261, 'left_w1': 1.1593059804444015, 'left_w2': -0.13230584295511694,
#                     'left_e0': -0.6247136758663347, 'left_e1': 1.6248691495676244, 'left_s0': -0.11083011192472114,
#                     'left_s1': -0.38157772098649667}
#
# right_observation = {'right_s0': -0.22204371904641365, 'right_s1': -0.21820876707670012, 'right_w0': 0.336325287743877,
#                      'right_w1': 1.3871021274453854, 'right_w2': 0.5506991028508635, 'right_e0': 0.9108010928069644,
#                      'right_e1': 1.268218616384266}

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
# 3 use hand to detect and grip
def use_hand_to_grip_3(gripper, pre_position, grip_method):  # gripper: 'left'/'right'
    rospy.loginfo("use_hand_to_grip_3 start")
    robot.baxter_ik_move(gripper, pre_position, offset=offset, tag='taskA')

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
        robot.baxter_ik_move(gripper, base_tf, tag='taskA')
        robot.set_j(limb, joint, rotate_theta, flag='pure')

        # get current end-joint's eular
        quaternion = np.asarray(limb.endpoint_pose()["orientation"])
        eular = np.asarray(
            tf.transformations.euler_from_quaternion([quaternion[0], quaternion[1], quaternion[2], quaternion[3]]))

        # Grasp down height
        # base_tf[2] = -0.142  # Large gripper
        base_tf[2] = -0.183  # Small gripper

        # print(base_tf)
        # offset_gripper = np.array([0.03, 0, 0])
        robot.baxter_ik_move(gripper, base_tf, eular)
        robot.gripper_control(gripper, 0)

        time.sleep(1)
        base_tf[2] = 0
        robot.baxter_ik_move(gripper, base_tf, eular)
        rospy.loginfo("use_hand_to_grip_3 finish")
    # except Exception:
    #     rospy.loginfo("use_hand_to_grip_3 error")


########################################################################################################################
# 4 exchange gripper if need
# left_vertical = [0.5935, -0.0226, 0.250, 0.036, -0.692, 0.7183, -0.053]  # -0.0654
# right_vertical = [0.5798, 0.0204, 0.265, 0.4421, 0.5556, 0.5003, -0.4964]  # 0.0532
#
# left_horizontal = [0.5935, -0.0226, 0.240, 0.5342, -0.4895, 0.4801, 0.4934]  # -0.0654
# right_horizontal = [0.5798, 0.0204, 0.265, 0.4421, 0.5556, 0.5003, -0.4964]  # 0.0532

# left_horizontal = [0.5935, -0.0654, 0.250, 0.036, -0.692, 0.7183, -0.053]  # -0.0654
# right_horizontal = [0.5798, 0.0532, 0.265, 0.4421, 0.5556, 0.5003, -0.4964]  # 0.0532
#
# left_vertical = [0.5935, -0.0654, 0.250, 0.5342, -0.4895, 0.4801, 0.4934]  # -0.0654
# right_vertical = [0.5798, 0.0532, 0.265, 0.4421, 0.5556, 0.5003, -0.4964]  # 0.0532


right_base = [0.614, 0.033, 0.373, 0.466, 0.540, 0.516, -0.471]
left_vertical = [0.614, -0.0733, 0.364, -0.000, -0.703, 0.7102, 0.033]
left_horizontal = [0.614, -0.0733, 0.359, 0.485, -0.514, 0.494, 0.504]

left_base = [0.599, -0.064, 0.369, 0.485, -0.514, 0.494, 0.504]
right_vertical = [0.599, 0.048, 0.370, -0.034, 0.712, 0.700, 0.0220]
right_horizontal = [0.599, 0.048, 0.355, 0.466, 0.540, 0.516, -0.471]


def exchange_gripper_4(direction, method):  # direction: left2right/right2left; method: vertical/horizontal
    rospy.loginfo("exchange_gripper_4 start")

    if direction == 'left2right':
        if method == 'vertical':
            give_gripper = left_base
            receive_gripper = right_vertical
        else:
            give_gripper = left_base
            receive_gripper = right_horizontal
        give = 'left'
        receive = 'right'
        limb_joints = left_limb_joints
    else:
        if method == 'vertical':
            give_gripper = right_base
            receive_gripper = left_vertical
        else:
            give_gripper = right_base
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
# 6 move the object to the wanted place
container = 2
area = 4
occupy_nums = container * area
occupy = np.zeros(occupy_nums)


def area_map(classify):
    # TODO
    return classify


def get_position_and_need_exchanges(classify, gripper):
    classify_save = copy.copy(classify)
    classify *= container
    # print(classify)
    idx = -1
    need_gripper = 'left'
    need_exchange = False
    for i in range(container):
        if occupy[classify + i] == 0:
            idx = classify + i
            occupy[idx] = 1
            break
    if idx == -1:
        idx = classify_save * container

    divide = area * container / 2

    if idx >= divide:
        need_gripper = 'right'

    if need_gripper != gripper:
        need_exchange = True
    print(idx, need_exchange)
    return idx, need_exchange


def move_to_put_place_6(gripper, classify, grip_method):
    rospy.loginfo("move_to_put_place_6 start")
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
        exchange_gripper_4(direction, method)

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

        rospy.loginfo("move_to_put_place_6 finish")
    except Exception:
        rospy.loginfo("move_to_put_place_6 error")


########################################################################################################################
#


########################################################################################################################
#

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

    except Exception:
        rospy.loginfo("go_to_observe_position")


def observe_at_the_position_5(gripper, grip_method):
    go_to_observe_position(gripper)

    client.wait_for_service()
    request = ImageProcSrvRequest()
    request.method = 8

    response = client.call(request)
    if isinstance(response, ImageProcSrvResponse):
        classify = response.object[0].position[0]
        move_to_put_place_6(gripper, classify, grip_method)


########################################################################################################################
########################################################################################################################
def taskA():
    go_to_initial_pose_1()
    # save_img()
    left_task_num, right_task_num, left_gripper_task, right_gripper_task = multi_object_recog_2()

    for i in range(left_task_num):
        print("---------------------------")
        print("left gripper start")
        use_hand_to_grip_3('left', left_gripper_task[i].position, right_gripper_task[i].grip_method)
        if (left_gripper_task[i].classify == -1):
            observe_at_the_position_5("left", left_gripper_task[i].grip_method)
        else:
            move_to_put_place_6('left', left_gripper_task[i].classify, left_gripper_task[i].grip_method)
        print("left gripper end")
        print("---------------------------")

    for i in range(right_task_num):
        print("---------------------------")
        print("right gripper start")
        use_hand_to_grip_3('right', right_gripper_task[i].position, right_gripper_task[i].grip_method)
        if (right_gripper_task[i].classify == -1):
            observe_at_the_position_5('right', right_gripper_task[i].grip_method)
        else:
            move_to_put_place_6('right', right_gripper_task[i].classify, right_gripper_task[i].grip_method)
        print("right gripper end")
        print("---------------------------")


########################################################################################################################
########################################################################################################################
# area detect
left_bound_limit, right_bound_limit, front_bound_limit, back_bound_limit = 0.58, -0.60, 0.80, 0.40


def area_detect(gripper, step):
    y_array = np.arange(right_bound_limit, left_bound_limit + step, step)

    if gripper == 'right':
        y_array = y_array[::-1]

    x_array = np.arange(back_bound_limit, front_bound_limit + step, step)

    log = []
    print(x_array)
    print(y_array)
    for x in x_array:
        for y in y_array:
            print("try to detect: ", x, y)
            if (robot.baxter_ik_move(gripper, np.array([x, y, 0]), tag='taskA', mode="detect") == True):
                log.append(np.array([x, y]))
                print("position append: ", x, y)
                break

    print(log)

    # print("---------------------------------------------------------")
    # for j in np.arange(pre_y_bound_limit, next_y_bound_limit, 0.02):
    #     print(j)
    # print(np.arange(right_bound_limit, left_bound_limit, 0.05))


########################################################################################################################
# main task

taskA()


# print(robot.get_joint())

# area_detect('right', 0.02)


# go_to_initial_pose_1()
# go_to_observe_position('right')
# go_to_initial_pose_1()
# go_to_observe_position('left')


# print(robot.get_joint())


# save_img()

# ({'left_w0': -1.4131798008394374, 'left_w1': 0.9986214929134043, 'left_w2': -0.18752915131899184,
#   'left_e0': -0.5518495884417776, 'left_e1': 1.309636097657172, 'left_s0': -0.8248981686853812,
#   'left_s1': -0.5330583237901813},
#   {'right_s0': 0.3834951969713534, 'right_s1': -1.252111818111469,
#   'right_w0': 1.398606983354526, 'right_w1': 1.8311895655382127, 'right_w2': 0.16528642989465334,
#   'right_e0': 0.03029612056073692, 'right_e1': 1.975767254796413})


#######################################################################################################################


#######################################################
def to_the_place_test():
    for i in range(len(put_place_left_all)):
        # for i in range(1):
        position = np.array(put_place_left_all[i][0:3])
        quaternion = np.array(put_place_left_all[i][3:7])
        robot.baxter_ik_move('left', position, quaternion)
    print("-----------------")
    for i in range(len(put_place_right_all)):
        position = np.array(put_place_right_all[i][0:3])
        quaternion = np.array(put_place_right_all[i][3:7])
        robot.baxter_ik_move('right', position, quaternion)


# to_the_place_test()
##########################################################

track_grip_method = 1


def track_test(gripper, grip_method):  # gripper: 'left'/'right'
    rospy.loginfo("use_hand_to_grip_3 start")

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

    while True:
        now = time.time()
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
            base_tf[2] = 0.066
            robot.baxter_ik_move(gripper, base_tf, tag='taskA')

        print(time.time() - now)

    rospy.loginfo("use_hand_to_grip_3 finish")


# track_test('left', track_grip_method)


########################################################################################################################


##########################################################
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
            robot.baxter_ik_move('right', position, head_camera='yes', offset=offset, tag='taskA')

            # print(position)
    except Exception:
        rospy.loginfo("arg error")


######################################################

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
            robot.baxter_ik_move('left', position, head_camera='yes', offset=offset, tag='taskA')

            # print(position)
    except Exception:
        rospy.loginfo("arg error")


def execute_tasks(gripper, gripper_tasks):
    for i in range(len(gripper_tasks)):
        robot.baxter_ik_move(gripper, gripper_tasks[i].position, offset=offset, tag='taskA')

# go_to_observe_position(left='true', right='true')
