#! /usr/bin/env python
import rospy
import tf
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

put_place = []


# robot.camera_open('left_hand_camera', exposure=5)
# robot.camera_open('right_hand_camera', exposure=10)


def go_to_initial_pose():
    # initial_pose_left = np.array([0.525, 0.568, 0.171])
    # initial_pose_right = np.array([0.525, -0.568, 0.171])

    left_limb_joints = {'left_w0': 0.47480141124049735, 'left_w1': 1.2314566949554813, 'left_w2': 0.15372088238146325,
                        'left_e0': -1.107928713595913, 'left_e1': 1.7039025215086192, 'left_s0': 0.5938655515377337,
                        'left_s1': -1.0688748478697863}
    right_limb_joints = {'right_s0': -0.14544413158371697, 'right_s1': -1.2990578089692,
                         'right_w0': -0.12480716836529432,
                         'right_w1': 1.1338270746789818, 'right_w2': -0.45249113715377826,
                         'right_e0': 0.4325969271093139,
                         'right_e1': 1.7651020359794947}

    t1 = threading.Thread(target=baxter_interface.Limb('left').move_to_joint_positions, args=(left_limb_joints,))
    t2 = threading.Thread(target=baxter_interface.Limb('right').move_to_joint_positions, args=(right_limb_joints,))

    t1.start()
    t2.start()
    t1.join()
    t2.join()

    # baxter_interface.Limb('left').move_to_joint_positions(left_limb_joints)
    # robot.baxter_ik_move('left', initial_pose_left)
    # robot.baxter_ik_move('right', initial_pose_right)


def getTfFromMatrix(matrix):
    scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(matrix)
    return trans, tf.transformations.quaternion_from_euler(*angles), angles


def lookupTransform(tf_listener, target, source):
    tf_listener.waitForTransform(target, source, rospy.Time(), rospy.Duration(4.0))

    trans, rot = tf_listener.lookupTransform(target, source, rospy.Time())
    euler = tf.transformations.euler_from_quaternion(rot)

    source_target = tf.transformations.compose_matrix(translate=trans, angles=euler)
    return source_target


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

    # print(robot.get_joint())
    # robot.set_j('right_w2', -1.2)
    # print(robot.get_joint())
    # print(robot.limb_right.joint_names())
    # joint_command = {'right_w2': 1.6975777632908919}
    # robot.limb_right.set_joint_positions(joint_command)


# control the wrist
def task2():
    # print(robot.get_joint())
    robot.set_j(robot.limb_right, 'right_w2', -2.2)
    # print(robot.get_joint())


def task3():
    client.wait_for_service()
    request = ImageProcSrvRequest()
    request.method = 2
    # print(request)
    # try:
    response = client.call(request)
    if isinstance(response, ImageProcSrvResponse):
        theta = response.object[0].orientation[0]
        print(theta)
        if theta == 0:
            theta = 180

        rotate_theta = robot.cal_gripper_theta(response.object[0].orientation[0])
        print(rotate_theta)
        position = np.asarray(response.object[0].position)
        print(position)
        position = np.append(position, 1)
        # position = np.array([0.01, 0.0, 0.215, 1])
        base_marker = lookupTransform(tf_listener, right_frame, '/base')
        trans_baxter, rot, rot_euler = getTfFromMatrix(np.linalg.inv(base_marker))
        # print(trans_baxter, rot)
        translation_matrix = robot.quaternion_to_trans_matrix(rot[0], rot[1], rot[2], rot[3],
                                                              trans_baxter[0], trans_baxter[1], trans_baxter[2])

        base_tf = np.dot(translation_matrix, position)[0:3]

        ###
        print(base_tf)
        robot.baxter_ik_move('right', base_tf, tag='solid')
        robot.set_j(robot.limb_right, 'right_w2', rotate_theta, flag='pure')
        quaternion = np.asarray(robot.limb_right.endpoint_pose()["orientation"])
        eular = np.asarray(
            tf.transformations.euler_from_quaternion([quaternion[0], quaternion[1], quaternion[2], quaternion[3]]))
        #
        base_tf[2] = -0.18
        print(base_tf)
        robot.baxter_ik_move('right', base_tf, rpy=eular)
        robot.gripper_control('right', 0)
        #
        time.sleep(1)
        base_tf[2] = 0
        robot.baxter_ik_move('right', base_tf, rpy=eular)
        robot.gripper_control('right', 100)
        ###

        # print(position)
    # except Exception:
    #    rospy.loginfo("arg error")


def task1_left():
    # robot.gripper_control(_gripper='left', value=100)
    # robot.left.close()
    # robot.camera_open('left_hand_camera')
    # camera_pose = np.array([-0.436535, 0.130556, 1.032])
    # robot.baxter_ik_move('left', np.array([-0.436535, 0.130556, 1.032]))

    client.wait_for_service()
    request = ImageProcSrvRequest()
    request.method = 0
    print(request)
    try:
        response = client.call(request)
        if isinstance(response, ImageProcSrvResponse):
            # print response.object[0].position[0]
            position = np.asarray(response.object[0].position)

            rpy = np.array([-3.14, 0, -3.14])
            robot.baxter_ik_move('left', position, head_camera='yes', offset=offset, tag='solid')

            # print(position)
    except Exception:
        rospy.loginfo("arg error")

    # print(robot.get_endpoint_euler('right'))
    # position = np.array([0.06341955, 0.16724122, 1.03209829])
    # rpy = np.array([-3.0517930548521597, 0.0005324968680877452, 0])
    # robot.baxter_ik_move('right', position, rpy)

    # print(robot.get_joint())
    # robot.set_j('right_w2', -1.2)
    # print(robot.get_joint())

    print(robot.limb_right.joint_names())
    # joint_command = {'right_w2': 1.6975777632908919}
    # robot.limb_right.set_joint_positions(joint_command)


def task3_left():
    client.wait_for_service()
    request = ImageProcSrvRequest()
    request.method = 1
    rospy.loginfo(request)
    # try:
    response = client.call(request)
    if isinstance(response, ImageProcSrvResponse):
        # print response.object[0].position
        position = np.asarray(response.object[0].position) / 1000
        print(position)
        position = np.append(position, 1)
        # position = np.array([0.01, 0.0, 0.215, 1])
        base_marker = lookupTransform(tf_listener, left_frame, '/base')
        trans_baxter, rot, rot_euler = getTfFromMatrix(np.linalg.inv(base_marker))
        # print(trans_baxter, rot)
        translation_matrix = robot.quaternion_to_trans_matrix(rot[0], rot[1], rot[2], rot[3],
                                                              trans_baxter[0], trans_baxter[1], trans_baxter[2])
        base_tf = np.dot(translation_matrix, position)[0:3]

        print(base_tf, "--")
        # offset = np.array([0.01, 0, 0])

        robot.baxter_ik_move('left', base_tf, tag='solid')

        base_tf[2] = -0.165
        print(base_tf)
        robot.baxter_ik_move('left', base_tf)
        robot.gripper_control('left', 0)
        time.sleep(1)
        base_tf[2] = 0
        robot.baxter_ik_move('left', base_tf)
        robot.gripper_control('left', 100)

        # position = np.asarray(response.object[0].position)
        # offset = np.array([0.05, 0.07, 0.1])
        # rpy = np.array([-3.14, 0, -3.14])
        # robot.baxter_ik_move('right', position, offset=offset)


# multi object recog
def task_4():
    client.wait_for_service()
    request = ImageProcSrvRequest()
    request.method = 3
    rospy.loginfo(request)

    try:
        response = client.call(request)
        if isinstance(response, ImageProcSrvResponse):
            # print(response)
            obj_list = object_sort(response)
            # obj_list.print_objs()
            # print response.object[0].position[0]
            # position = np.asarray(response.object[0].position)
            offset = np.array([0.05, 0.07, 0.1])
            # rpy = np.array([-3.14, 0, -3.14])

            left_task_num, right_task_num, left_gripper_task, right_gripper_task = obj_list.assign_task()
            for i in range(left_task_num):
                left_gripper_task[i].print_object()

            for i in range(right_task_num):
                right_gripper_task[i].print_object()

            #
            # for i in range(len(obj_list.object_list)):
            #     #print(obj_list.object_list[i].position)
            #     robot.baxter_ik_move('right', obj_list.object_list[i].position, offset=offset, tag='solid')
            #

            # print(position)
    except Exception:
        rospy.loginfo("arg error")


def task_4_test():
    client.wait_for_service()
    request = ImageProcSrvRequest()
    request.method = 3
    rospy.loginfo(request)

    try:
        response = client.call(request)
        if isinstance(response, ImageProcSrvResponse):
            obj_list = object_sort(response)
            offset = np.array([0.05, 0.07, 0.1])
            # rpy = np.array([-3.14, 0, -3.14])

            left_task_num, right_task_num, left_gripper_task, right_gripper_task = obj_list.assign_task()
            for i in range(left_task_num):
                left_gripper_task[i].print_object()
            print("------------------------")
            for i in range(right_task_num):
                right_gripper_task[i].print_object()
            print("------------------------")

            t1 = threading.Thread(target=execute_tasks,
                                  args=('left', left_gripper_task))
            t2 = threading.Thread(target=execute_tasks,
                                  args=('right', right_gripper_task))
            t1.start()
            t2.start()
            t1.join()
            t2.join()

            #
            # for i in range(len(obj_list.object_list)):
            #     #print(obj_list.object_list[i].position)
            #     robot.baxter_ik_move('right', obj_list.object_list[i].position, offset=offset, tag='solid')
            #

            # print(position)
    except Exception:
        rospy.loginfo("arg error")


def execute_tasks(gripper, gripper_tasks):
    for i in range(len(gripper_tasks)):
        robot.baxter_ik_move(gripper, gripper_tasks[i].position, offset=offset, tag='solid')


def task_5():
    client.wait_for_service()
    request = ImageProcSrvRequest()
    request.method = 2
    # print(request)
    # try:
    response = client.call(request)
    if isinstance(response, ImageProcSrvResponse):
        theta = response.object[0].orientation[0]
        print(theta)
        if theta == 0:
            theta = 180

        rotate_theta = robot.cal_gripper_theta(response.object[0].orientation[0])
        print(rotate_theta)

        position = np.asarray(response.object[0].position) / 1000
        print(position)
        position = np.append(position, 1)
        # position = np.array([0.01, 0.0, 0.215, 1])
        base_marker = lookupTransform(tf_listener, right_frame, '/base')
        trans_baxter, rot, rot_euler = getTfFromMatrix(np.linalg.inv(base_marker))
        # print(trans_baxter, rot)
        translation_matrix = robot.quaternion_to_trans_matrix(rot[0], rot[1], rot[2], rot[3],
                                                              trans_baxter[0], trans_baxter[1], trans_baxter[2])

        base_tf = np.dot(translation_matrix, position)[0:3]

        ###
        print(base_tf)
        robot.baxter_ik_move('right', base_tf, tag='solid')

        robot.set_j(robot.limb_right, 'right_w2', rotate_theta, flag='pure')

        quaternion = np.asarray(robot.limb_right.endpoint_pose()["orientation"])
        eular = np.asarray(
            tf.transformations.euler_from_quaternion([quaternion[0], quaternion[1], quaternion[2], quaternion[3]]))
        #

        # base_tf[2] = -0.178
        # print(base_tf)
        # robot.baxter_ik_move('right', base_tf, rpy=eular)
        # robot.gripper_control('right', 0)
        #
        # time.sleep(1)
        # base_tf[2] = 0.1
        # robot.baxter_ik_move('right', base_tf, rpy=eular)
        #
        # goal_pose = np.array([0.525, -0.568, 0.171])
        # base_tf[2] = 0
        # robot.baxter_ik_move('right', goal_pose, rpy=eular)
        # robot.gripper_control('right', 100)


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
        divide_pos = 0
        for i in range(len(self.object_list)):
            if self.object_list[i].position[1] >= 0:
                divide_pos = i
                break
        return len(self.object_list) - divide_pos, divide_pos, \
               self.object_list[divide_pos:len(self.object_list)], self.object_list[0:divide_pos],


def save_img():
    client.wait_for_service()
    request = ImageProcSrvRequest()
    request.method = -1
    # print(request)
    # try:
    response = client.call(request)
    if isinstance(response, ImageProcSrvResponse):
        print("saved")


def assign_task(response):
    if response.is_success:
        length = len(response.grip_method)
        for i in range(length):
            response.object[i].position = robot.camera_to_base_tf(np.asarray(response.object[i].position))
        print(response)
    else:
        return False


# save_img()

go_to_initial_pose()

# single

# head_recognition()
# # # #
# # # # task2()
# task3()


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
# task1_left()
# task3_left()

# multi_task

# cnt = 6
# while cnt > 0:
#     cnt -= 1

# task_4()
#task_4_test()
# task_5()


# initial_pose_left = np.array([0.525, 0.568, 0.171])
# initial_pose_right = np.array([0.525, -0.568, 0.171])
#
# t1 = threading.Thread(target=robot.baxter_ik_move, args=('left', initial_pose_left))
# t2 = threading.Thread(target=robot.baxter_ik_move, args=('right', initial_pose_right))
#
# t1.start()
# t2.start()
