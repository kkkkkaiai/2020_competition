#! /usr/bin/env python
# coding=utf-8
import sys
import copy
import math
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


joint_state_topic = ['joint_states:=/robot/joint_states']
#moveit_commander.roscpp_initialize(sys.argv)
moveit_commander.roscpp_initialize(joint_state_topic)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=False)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
left_arm = moveit_commander.MoveGroupCommander("left_arm")
right_arm = moveit_commander.MoveGroupCommander('right_arm')


# print "============ Reference frame: %s" % left_arm.get_planning_frame()
# print "============ Reference frame: %s" % left_arm.get_end_effector_link()
#
# print "==============================================="
# # print robot.get_group_names()
# # print "============ Printing robot state"
# print robot.get_current_state()
# print "============"

# 'left_w0': -1.000538968898261,
# 'left_w1': 1.1593059804444015,
# 'left_w2': -0.13230584295511694,
# 'left_e0': -0.6247136758663347,
# 'left_e1': 1.6248691495676244,
# 'left_s0': -0.11083011192472114,
#  left_s1': -0.38157772098649667}

def quaternion_normailized(x, y, z, w):
    all = math.sqrt(x*x+y*y+z*z+w*w)
    print(all)
    return x/all, y/all, z/all, w/all


def go_to_joint_state(move_group):
    joint_goal = move_group.get_current_joint_values()
    print(joint_goal)
    joint_goal[0] = -0.11083011192472114
    joint_goal[1] = -0.38157772098649667
    joint_goal[2] = -0.6247136758663347
    joint_goal[3] = 1.6248691495676244
    joint_goal[4] = -1.000538968898261
    joint_goal[5] = 1.1593059804444015
    joint_goal[6] = -0.13230584295511694
    print(joint_goal)
    move_group.go(joint_goal, wait=True)
    move_group.stop()
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


left_vertical = [0.5935, -0.0434, 0.250, 0.036, -0.692, 0.7183, -0.053]
right_horizontal = [0.5798, 0.0312, 0.265, 0.4421, 0.5556, 0.5003, -0.4964]


def go_to_pose_goal(move_group):
    print(left_arm.get_current_pose().pose)
    pose_goal = geometry_msgs.msg.Pose()
    # x, y, z, w = quaternion_normailized(left_vertical[3], left_vertical[4], left_vertical[5], left_vertical[6])
    x, y, z, w = quaternion_normailized(0.639, 0.768, -0.0077, 0.015)
    pose_goal.orientation.w = w
    pose_goal.orientation.x = x
    pose_goal.orientation.y = y
    pose_goal.orientation.z = z
    pose_goal.position.x = 0.689
    pose_goal.position.y = 0.42
    pose_goal.position.z = 0.26
    print pose_goal

    move_group.set_pose_target(pose_goal)
    move_group.plan()
    move_group.go(wait=True)

    # move_group.stop()

    move_group.clear_pose_targets()

    current_pose = move_group.get_current_pose().pose
    print("----------------", current_pose)
    return all_close(pose_goal, current_pose, 0.01)

# print(left_arm.get_current_pose().pose)
go_to_pose_goal(left_arm)

print(robot.get_planning_frame())
box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = robot.get_planning_frame()
box_pose.pose.orientation.w = 1.0
box_pose.pose.position.x = 0.7 # above the panda_hand frame
box_pose.pose.position.y = 0
box_pose.pose.position.z = -0.16
box_name = "box_iiii"
scene.add_box(box_name, box_pose, size=(0.8, 1.4, 0.02))


timeout = 4
box_is_known = True

attached_objects = scene.get_objects([box_name])
print(attached_objects)

start = rospy.get_time()
seconds = rospy.get_time()
while (seconds - start < timeout) and not rospy.is_shutdown():
    # Test if the box is in attached objects
    attached_objects = scene.get_objects([box_name])
    print(attached_objects)
    # Test if the box is in the scene.
    # Note that attaching the box will remove it from known_objects
    is_known = box_name in scene.get_known_object_names()

    # Test if we are in the expected state
    if box_is_known == is_known:
        print(1)
        break

    # Sleep so that we give other threads time on the processor
    rospy.sleep(0.1)
    seconds = rospy.get_time()

# go_to_pose_goal(left_arm)


# rospy.spin()
# waypoints = []
# waypoints.append(group.get_current_pose().pose)
# wpose = geometry_msgs.msg.Pose()
# wpose.orientation.w = 1.0
# wpose.position.x = waypoints[0].position.x + 0.1
# wpose.position.y = waypoints[0].position.y
# wpose.position.z = waypoints[0].position.z
# waypoints.append(copy.deepcopy(wpose))
#
# wpose.position.z -= 0.20
# waypoints.append(copy.deepcopy(wpose))
#
# wpose.position.y += 0.3
# waypoints.append(copy.deepcopy(wpose))
#
# print(waypoints)
#
# (plan3, fraction) = group.compute_cartesian_path(
#                              waypoints,   # waypoints to follow
#                              0.01,        # eef_step
#                              0.01)         # jump_threshold
# #left_arm.execute(plan3, wait=True)
