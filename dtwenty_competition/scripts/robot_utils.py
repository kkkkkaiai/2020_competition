#! /usr/bin/env python
# coding=utf-8
import rospy
import sys
import yaml
import os
import numpy as np
# import image_geometry
# # from object_recognition.msg import ObjectInfo
# #import planning_node as pnode
import baxter_interface
import tf
import time
import threading
# import std_srvs.srv
from moveit_commander import conversions
from std_msgs.msg import Header
# from std_msgs.msg import Bool
# from std_msgs.msg import String
# from sensor_msgs.msg import CameraInfo
# from geometry_msgs.msg import Point
from baxter_interface import CHECK_VERSION
from baxter_interface import CameraController
from baxter_core_msgs.srv import (SolvePositionIK,
                                  SolvePositionIKRequest)


# import baxter_external_devices

# 1 left 215
# 2 right 215
# 4 left 185
# 5 right 185

# classification
# 0--圆柱 1--四棱柱 2--不等边四棱柱 3--六棱柱 -1--躺着的工件

# observation position
# {'left_w0': -1.000538968898261, 'left_w1': 1.1593059804444015, 'left_w2': -0.13230584295511694,
#  'left_e0': -0.6247136758663347, 'left_e1': 1.6248691495676244, 'left_s0': -0.11083011192472114,
#  'left_s1': -0.38157772098649667}
#
# {'right_s0': -0.22204371904641365, 'right_s1': -0.21820876707670012, 'right_w0': 0.336325287743877,
#  'right_w1': 1.3871021274453854, 'right_w2': 0.5506991028508635, 'right_e0': 0.9108010928069644,
#  'right_e1': 1.268218616384266}


class RobotInit:
    def __init__(self):
        self.rs = baxter_interface.RobotEnable(CHECK_VERSION)
        init_state = self.rs.state().enabled

        self.left = baxter_interface.Gripper('left', CHECK_VERSION)
        self.right = baxter_interface.Gripper('right', CHECK_VERSION)
        self.left.set_dead_band(2.5)
        self.right.set_dead_band(2.5)
        self.limb_right = baxter_interface.Limb('right')
        self.limb_left = baxter_interface.Limb('left')

        kinect_param_file = '/home/ljq/ros_ws/src/kinect_baxter_calibration/files/kinect_calibration.yaml'
        with open(kinect_param_file, 'r') as f:
            params = yaml.safe_load(f)
        rot = params['rot']
        trans = params['trans']
        # x, y, z, w = -0.691862525656, 0.695578434607, -0.147358438807, 0.125627932465
        # trans_x, trans_y, trans_z = 0.351783955273, -0.111383586522, 0.883528508991
        x, y, z, w = rot[0], rot[1], rot[2], rot[3]
        trans_x, trans_y, trans_z = trans[0], trans[1], trans[2]

        self.translation_matrix = self.quaternion_to_trans_matrix(x, y, z, w, trans_x, trans_y, trans_z)

    def quaternion_to_trans_matrix(self, x, y, z, w, trans_x, trans_y, trans_z):
        x_2, y_2, z_2 = x * x, y * y, z * z
        xy, xz, yz, wx, wy, wz = x * y, x * z, y * z, w * x, w * y, w * z
        # origin_rotation_matrix        [1 - 2 * y_2 - 2 * z_2,  2 * xy + 2 * wz,        2 * xz - 2 * wy,
        #                                2 * xy - 2 * wz,        1 - 2 * x_2 - 2 * z_2,  2 * yz + 2 * wx,
        #                                2 * xz + 2 * wy,        2 * yz - 2 * wx,        1 - 2 * x_2 - 2 * y_2]
        translation_matrix = np.array([1 - 2 * y_2 - 2 * z_2, 2 * xy - 2 * wz, 2 * xz + 2 * wy, trans_x,
                                       2 * xy + 2 * wz, 1 - 2 * x_2 - 2 * z_2, 2 * yz - 2 * wx, trans_y,
                                       2 * xz - 2 * wy, 2 * yz + 2 * wx, 1 - 2 * x_2 - 2 * y_2, trans_z,
                                       0, 0, 0, 1
                                       ])
        return translation_matrix.reshape((4, 4))

    def camera_to_base_tf(self, camera_pose):
        camera_tf = np.append(camera_pose[0:3], 1)
        camera_tf = camera_tf.reshape((4, 1))
        # print(camera_tf.shape)
        base_tf = np.dot(self.translation_matrix, camera_tf)
        return base_tf[0:3].reshape(-1)

    def baxter_ik_move(self, limb, position, rpy=np.array([-3.14, 0, -3.14]),
                       offset=np.array([0, 0, 0]), head_camera='no', tag='free', timeout=15.0):
        node = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        ik_service = rospy.ServiceProxy(node, SolvePositionIK)
        ik_request = SolvePositionIKRequest()
        if head_camera == 'yes':
            position = self.camera_to_base_tf(position)
        print(position)
        position += offset
        if tag == 'solid':
            position[2] = -0.037
        rpy_pose = np.append(position, rpy)

        hdr = Header(stamp=rospy.Time.now(), frame_id="base")
        quaternion_pose = conversions.list_to_pose_stamped(rpy_pose, "base")
        ik_request.pose_stamp.append(quaternion_pose)
        try:
            rospy.wait_for_service(node, 5.0)
            ik_response = ik_service(ik_request)
            # print(ik_response,"----------------------------------------------")
        except (rospy.ServiceException, rospy.ROSException), error_message:
            rospy.logerr("Service request failed: %r" % (error_message,))
            sys.exit("ERROR - baxter_ik_move - Failed to append pose")
        if ik_response.isValid[0]:
            print("PASS: Valid joint configuration found")
            # convert response to joint position control dictionary
            limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
            # print(limb_joints)
            # move limb
            baxter_interface.Limb(limb).move_to_joint_positions(limb_joints, timeout=timeout)
        else:
            print "requested move =", rpy_pose
            sys.exit("ERROR - baxter_ik_move - No valid joint configuration found")

        if limb == limb:  # if working arm
            quaternion_pose = baxter_interface.Limb(limb).endpoint_pose()
            position = quaternion_pose['position']

    def go_to_initial_pose(self):
        # initial_pose_left = np.array([0.525, 0.568, 0.171])
        # initial_pose_right = np.array([0.525, -0.568, 0.171])

        left_limb_joints = {'left_w0': 0.47480141124049735, 'left_w1': 1.2314566949554813,
                            'left_w2': 0.15372088238146325,
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

    def camera_open(self, _hand_camera, resolution=(960, 600), exposure=-1, gain=-1):
        hand_camera = CameraController(_hand_camera)  # left/right_hand_camera
        hand_camera.resolution = resolution
        hand_camera.exposure = exposure  # range, 0-100 auto = -1
        hand_camera.gain = gain  # range, 0-79 auto = -1
        hand_camera.white_balance_blue = -1  # range 0-4095, auto = -1
        hand_camera.white_balance_green = -1  # range 0-4095, auto = -1
        hand_camera.white_balance_red = -1  # range 0-4095, auto = -1
        # open camera
        hand_camera.open()

    def gripper_control(self, _gripper, value=0, offset=0):
        left = baxter_interface.Gripper('left', CHECK_VERSION)
        right = baxter_interface.Gripper('right', CHECK_VERSION)
        if offset == 0:
            if _gripper == 'left':
                left.command_position(value)
            elif _gripper == 'right':
                right.command_position(value)
        else:
            if _gripper == 'left':
                current_left = self.left.position()
                left.command_position(current_left + offset)
            elif _gripper == 'right':
                current_right = self.right.position()
                right.command_position(current_right + offset)

    def get_joint(self):
        return self.limb_left.joint_angles(), self.limb_right.joint_angles()

    def get_endpoint_euler(self, _limb):  # left/right
        endpoint_pose = baxter_interface.Limb(_limb).endpoint_pose()
        # endpoint_position = endpoint_pose['position']
        endpoint_orientation = endpoint_pose['orientation']
        endpoint_euler = tf.transformations.euler_from_quaternion(
            [endpoint_orientation[0], endpoint_orientation[1], endpoint_orientation[2],
             endpoint_orientation[3]])
        return endpoint_euler

    def set_j(self, limb, joint_name, delta, threshold=0.01, flag='pure'):  # limb:left/right joint_name:w2
        current_position = limb.joint_angle(joint_name)
        if flag=='pure':
            goal = delta
        else:
            goal = current_position + delta
        if abs(goal) >= 3.05:
            if goal > 0:
                goal = goal - 3.05 * 2
            else:
                goal = goal + 3.05 * 2
        joint_command = {joint_name: goal}
        # while goal-current_position >= threshold:
        #     limb.set_joint_positions(joint_command)
        #     current_position = limb.joint_angle(joint_name)
        limb.move_to_joint_positions(joint_command)

    def cal_gripper_theta(self, theta):
        print 0.23-((theta/180)*3.1415926-3.1415926/2)
        return 0.23-((theta/180)*3.1415926-3.1415926/2)
        # return 1.8642-0.017*theta


if __name__ == "__main__":
    rospy.init_node("control_node", anonymous=True)
# robot.gripper_control(_gripper='left', value=10)
# robot.camera_open('left_hand_camera')
# camera_pose = np.array([-0.436535,0.130556,1.032])
# robot.baxter_ik_move('left', np.array([-0.436535,0.130556,1.032]))
