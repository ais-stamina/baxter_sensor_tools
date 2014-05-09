#!/usr/bin/env python

import argparse
from copy import copy
import sys

import rospy

import actionlib

import baxter_interface

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

class Trajectory(object):
    def __init__(self, limb):
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]


def main():

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_trajectory_client")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable()
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")
    
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.7,
                    y=0.22,
                    z=0.05,
                ),
                orientation=Quaternion(
                    x=0.0,
                    y=0.98,
                    z=-0.14,
                    w=0.0,
                ),
            ),
        ),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.7,
                    y=-0.22,
                    z=0.05,
                ),
                orientation=Quaternion(
                    x=0.0,
                    y=0.98,
                    z=0.14,
                    w=0.0,
                ),
            ),
        ),
    }
    
    ns_left = "ExternalTools/left/PositionKinematicsNode/IKService"
    iksvc_left = rospy.ServiceProxy(ns_left, SolvePositionIK)
    ikreq_left = SolvePositionIKRequest()

    ikreq_left.pose_stamp.append(poses["left"])
    try:
        rospy.wait_for_service(ns_left, 5.0)
        resp = iksvc_left(ikreq_left)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return -1
    if (resp.isValid[0]):
        print("SUCCESS - Valid Joint Solution Found:")
        # Format solution into Limb API-compatible dictionary
        left_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        positions_left = [ left_joints['left_s0'], left_joints['left_s1'], left_joints['left_e0'], left_joints['left_e1'], left_joints['left_w0'], left_joints['left_w1'], left_joints['left_w2'] ];       
        print left_joints
        print positions_left
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        
        
    ns_right = "ExternalTools/right/PositionKinematicsNode/IKService"
    iksvc_right = rospy.ServiceProxy(ns_right, SolvePositionIK)
    ikreq_right = SolvePositionIKRequest()

    ikreq_right.pose_stamp.append(poses["right"])
    try:
        rospy.wait_for_service(ns_right, 5.0)
        resp = iksvc_right(ikreq_right)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return -1
    if (resp.isValid[0]):
        print("SUCCESS - Valid Joint Solution Found:")
        # Format solution into Limb API-compatible dictionary
        right_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        positions_right = [ right_joints['right_s0'], right_joints['right_s1'], right_joints['right_e0'], right_joints['right_e1'], right_joints['right_w0'], right_joints['right_w1'], right_joints['right_w2'] ];       
        print right_joints
        print positions_right
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

    traj_left = Trajectory("left")
    traj_right = Trajectory("right")
    rospy.on_shutdown(traj_left.stop)
    rospy.on_shutdown(traj_right.stop)

    p_left = positions_left
    p_right = positions_right
    
    traj_left.add_point(p_left, 10.0)
    traj_right.add_point(p_right, 10.0)

    traj_left.start()
    traj_right.start()
    traj_left.wait(25.0)
    traj_right.wait(1.0)
    
    
    print("Exiting - Joint Trajectory Action Test Complete")

if __name__ == "__main__":
    main()
