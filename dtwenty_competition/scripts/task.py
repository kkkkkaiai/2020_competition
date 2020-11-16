#! /usr/bin/env python
import rospy
from robot_utils import *
from image_proc_msgs.srv import ImageProcSrv, ImageProcSrvRequest, ImageProcSrvResponse
from image_proc_msgs.msg import ImageProcMsg
from sensor_msgs.msg import Image


rospy.init_node("control_node", anonymous=True)
robot = RobotInit()

service_name = "Image_Process_A"
client = rospy.ServiceProxy(service_name, ImageProcSrv)


def task1():
    #robot.gripper_control(_gripper='left', value=50)
    # robot.camera_open('left_hand_camera')
    camera_pose = np.array([-0.436535, 0.130556, 1.032])
    #robot.baxter_ik_move('left', np.array([-0.436535, 0.130556, 1.032]))

    client.wait_for_service()
    request = ImageProcSrvRequest()
    request.method = 1
    print(request)
    try:
        response = client.call(request)
        if isinstance(response, ImageProcSrvResponse):
            print response
    except Exception:
        print("Error")




task1()
