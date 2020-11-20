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
    request.method = 3
    rospy.loginfo(request)

    try:
        response = client.call(request)
        if isinstance(response, ImageProcSrvResponse):
            # print(response)
            # print response.object[0].position[0]
            position = np.asarray(response.object[0].position)

            rpy = np.array([-3.14, 0, -3.14])
            robot.baxter_ik_move('right', position, head_camera='yes', tag='solid')

            # print(position)
    except Exception:
        rospy.loginfo("arg error")


def hand_recognition():
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

        rotate_theta = robot.cal_gripper_theta(response.object[0].orientation[0])
        # print(rotate_theta)

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


        # print(base_tf)

        ###
        robot.baxter_ik_move('right', base_tf, tag='solid')

        robot.set_j(robot.limb_right, 'right_w2', rotate_theta, flag='pure')

        quaternion = np.asarray(robot.limb_right.endpoint_pose()["orientation"])
        eular = np.asarray(
            tf.transformations.euler_from_quaternion([quaternion[0], quaternion[1], quaternion[2], quaternion[3]]))
        ###


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


robot.go_to_initial_pose()

head_recognition()
print(robot.limb_right.joint_angle('right_w2'))
hand_recognition()
print(robot.limb_right.joint_angle('right_w2'))
