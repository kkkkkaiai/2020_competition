#! /usr/bin/env python
# coding=utf-8

import tf
import copy
import time
import rospy
import threading
import math
from robot_utils import *
import numpy as np

import baxter_interface
import baxter_external_devices

from sensor_msgs.msg import Image
from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest)
from image_proc_msgs.srv import ImageProcSrv, ImageProcSrvRequest, ImageProcSrvResponse

from moveit_commander import conversions

from baxter_interface import CHECK_VERSION
from baxter_interface import CameraController
from baxter_core_msgs.srv import (SolvePositionIK,
                                  SolvePositionIKRequest)

rospy.init_node("control_node", anonymous=True)
robot = RobotInit()

right_frame = 'right_hand_camera'
left_frame = 'left_hand_camera'
service_name = "Image_Process_A"

tf_listener = tf.TransformListener()
client = rospy.ServiceProxy(service_name, ImageProcSrv)

offset = np.array([0.05, 0.0, 0.0])
# offset = np.array([0.0, 0.0, 0.0])

robot.limb_left.set_joint_position_speed(1)
robot.limb_right.set_joint_position_speed(1)


def generate_round_points(x_0, y_0, start_z, target_z, radius, interval):
    theta = np.arange(0, 2 * math.pi, interval)
    theta = theta[::-1]
    # print(theta)
    x = x_0 + radius * np.cos(theta)
    y = y_0 + radius * np.sin(theta)

    z = (target_z-start_z)/2 * np.sin(np.pi/2+theta)-(target_z-start_z)/2
    z = -z
    return x, y,z


joint_position = []
interval = 0.06
start_z = 0.0
target_z = start_z - 0.07
part = 5

x, y, z = generate_round_points(0.643, 0.001, start_z, target_z, 0.08, interval)  # 0.6 Test


def z_cut(start_z, target_z, lens, part):
    z_ = np.linspace(target_z, start_z, lens / part)
    return z_


# z = z_cut(start_z, target_z, len(x), part)
# z = z[::-1]

#
# print(z)

ik_request = SolvePositionIKRequest()
orientation = np.array([-3.14, 0, -3.14])

limb = 'left'

node = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
for i in range(len(x)):
    ik_request = SolvePositionIKRequest()
    ik_service = rospy.ServiceProxy(node, SolvePositionIK)

    position = np.array([x[i], y[i], z[i]])
    pose_orientation = np.append(position, orientation)
    quaternion_pose = conversions.list_to_pose_stamped(pose_orientation, "base")

    ik_request.pose_stamp.append(quaternion_pose)
    try:
        rospy.wait_for_service(node, 5.0)
        ik_response = ik_service(ik_request)
        # print(ik_response,"----------------------------------------------")
    except (rospy.ServiceException, rospy.ROSException), error_message:
        rospy.logerr("Service request failed: %r" % (error_message,))
        sys.exit("ERROR - baxter_ik_move - Failed to append pose")

    if ik_response.isValid[0]:
        limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
        # print(limb_joints)
        # move limb
        joint_position.append(limb_joints)


while True:
    for i in range(len(joint_position)):
        baxter_interface.Limb(limb).set_joint_positions(joint_position[i])
        time.sleep(0.15)
