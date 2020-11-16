#! /usr/bin/env python
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
# import std_srvs.srv
from moveit_commander import conversions
from std_msgs.msg import Header
# from std_msgs.msg import Bool
# from std_msgs.msg import String
# from sensor_msgs.msg import CameraInfo
# from geometry_msgs.msg import Point
from baxter_interface import CameraController
from baxter_core_msgs.srv import ( SolvePositionIK,
                                   SolvePositionIKRequest )
# import baxter_external_devices

from baxter_interface import CHECK_VERSION


class Robot_init:
    # rospy.init_node("control_node", anonymous=True)
    # rs = baxter_interface.RobotEnable(CHECK_VERSION)
    # init_state = rs.state().enabled
    # ik_request = SolvePositionIKRequest()
    # left = baxter_interface.Gripper('left', CHECK_VERSION)
    # right = baxter_interface.Gripper('right', CHECK_VERSION)
    # left.set_dead_band(2.5)
    # right.set_dead_band(2.5)
    #
    # kinectParamFile = '/home/ljq/ros_ws/src/kinect_baxter_calibration/files/kinect_calibration.yaml'
    # with open(kinectParamFile, 'r') as f:
    #     params = yaml.safe_load(f)
    # rot = params['rot']
    # trans = params['trans']
    # # x, y, z, w = -0.691862525656, 0.695578434607, -0.147358438807, 0.125627932465
    # # trans_x, trans_y, trans_z = 0.351783955273, -0.111383586522, 0.883528508991
    # x, y, z, w = rot[0], rot[1], rot[2], rot[3]
    # trans_x, trans_y, trans_z = trans[0], trans[1], trans[2]
    #
    # x_2, y_2, z_2 = x * x, y * y, z * z
    # xy, xz, yz, wx, wy, wz = x * y, x * z, y * z, w * x, w * y, w * z
    # translation_matrix = np.array([1 - 2 * y_2 - 2 * z_2, 2 * xy - 2 * wz, 2 * xz + 2 * wy, trans_x,
    #                                2 * xy + 2 * wz, 1 - 2 * x_2 - 2 * z_2, 2 * yz - 2 * wx, trans_y,
    #                                2 * xz - 2 * wy, 2 * yz + 2 * wx, 1 - 2 * x_2 - 2 * y_2, trans_z,
    #                                0, 0, 0, 1
    #                                ])
    # translation_matrix = translation_matrix.reshape((4, 4))

    def __init__(self):
        rospy.init_node("control_node", anonymous=True)
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        init_state = rs.state().enabled
        self.ik_request = SolvePositionIKRequest()
        self.left = baxter_interface.Gripper('left', CHECK_VERSION)
        self.right = baxter_interface.Gripper('right', CHECK_VERSION)
        self.left.set_dead_band(2.5)
        self.right.set_dead_band(2.5)

        kinectParamFile = '/home/ljq/ros_ws/src/kinect_baxter_calibration/files/kinect_calibration.yaml'
        with open(kinectParamFile, 'r') as f:
            params = yaml.safe_load(f)
        rot = params['rot']
        trans = params['trans']
        # x, y, z, w = -0.691862525656, 0.695578434607, -0.147358438807, 0.125627932465
        # trans_x, trans_y, trans_z = 0.351783955273, -0.111383586522, 0.883528508991
        x, y, z, w = rot[0], rot[1], rot[2], rot[3]
        trans_x, trans_y, trans_z = trans[0], trans[1], trans[2]

        x_2, y_2, z_2 = x * x, y * y, z * z
        xy, xz, yz, wx, wy, wz = x * y, x * z, y * z, w * x, w * y, w * z
        translation_matrix = np.array([1 - 2 * y_2 - 2 * z_2, 2 * xy - 2 * wz, 2 * xz + 2 * wy, trans_x,
                                       2 * xy + 2 * wz, 1 - 2 * x_2 - 2 * z_2, 2 * yz - 2 * wx, trans_y,
                                       2 * xz - 2 * wy, 2 * yz + 2 * wx, 1 - 2 * x_2 - 2 * y_2, trans_z,
                                       0, 0, 0, 1
                                       ])
        self.translation_matrix = translation_matrix.reshape((4, 4))

    def camera_to_base_tf(self, camera_pose):
        camera_tf = np.append(camera_pose[0:3], 1)
        camera_tf = camera_tf.reshape((4, 1))
        # print(camera_tf.shape)
        base_tf = np.dot(self.translation_matrix, camera_tf)
        return base_tf[0:3].reshape(-1)

    def baxter_ik_move(self, limb, rpy_pose):
        node = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        ik_service = rospy.ServiceProxy(node, SolvePositionIK)
        rpy_pose = self.camera_to_base_tf(rpy_pose)
        rpy_pose = np.append(rpy_pose, [-3.14, 0, -3.14])
        hdr = Header(stamp=rospy.Time.now(), frame_id="base")
        self.quaternion_pose = conversions.list_to_pose_stamped(rpy_pose, "base")
        self.ik_request.pose_stamp.append(self.quaternion_pose)
        try:
            rospy.wait_for_service(node, 5.0)
            ik_response = ik_service(self.ik_request)
        except (rospy.ServiceException, rospy.ROSException), error_message:
            rospy.logerr("Service request failed: %r" % (error_message,))
            sys.exit("ERROR - baxter_ik_move - Failed to append pose")
        if ik_response.isValid[0]:
            print("PASS: Valid joint configuration found")
            # convert response to joint position control dictionary
            limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
            # move limb
            baxter_interface.Limb(limb).move_to_joint_positions(limb_joints)
        else:
            print "requested move =", rpy_pose
            sys.exit("ERROR - baxter_ik_move - No valid joint configuration found")

        if limb == limb:  # if working arm
            quaternion_pose = baxter_interface.Limb(limb).endpoint_pose()
            position = quaternion_pose['position']

    def camera_open(self, _hand_camera, resolution=(960, 600), exposure=-1, gain=-1):
        hand_camera = CameraController(_hand_camera) # left/right_hand_camera
        hand_camera.resolution = resolution
        hand_camera.exposure = exposure  # range, 0-100 auto = -1
        hand_camera.gain = gain  # range, 0-79 auto = -1
        hand_camera.white_balance_blue = -1  # range 0-4095, auto = -1
        hand_camera.white_balance_green = -1  # range 0-4095, auto = -1
        hand_camera.white_balance_red = -1  # range 0-4095, auto = -1
        # open camera
        hand_camera.open()

    def gripper_control(self, _gripper, value, offset=0):
        if offset == 0:
            if _gripper == 'left':
                self.left.command_position(value)
            elif _gripper == 'right':
                self.right.command_position(value)
        else:
            if _gripper == 'left':
                current_left = self.left.position()
                self.left.command_position(current_left+offset)
            elif _gripper == 'right':
                current_right = self.right.position()
                self.right.command_position(current_right+offset)


robot = Robot_init()
# robot.gripper_control(_gripper='left', value=10)
# robot.camera_open('left_hand_camera')
# camera_pose = np.array([-0.436535,0.130556,1.032])
# robot.baxter_ik_move('left', np.array([-0.436535,0.130556,1.032]))