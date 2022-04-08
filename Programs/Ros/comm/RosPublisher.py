import rospy
from geometry_msgs.msg import PoseStamped

from scipy.spatial.transform import Rotation

import system.RobotState

class RosPublisher:

    def __init__(self, frameId):
        self.frameId = frameId

    def StateToPose(self, state):

        position = state.GetPosition()
        pose = PoseStamped()
        pose.header.stamp = rospy.get_rostime()
        pose.header.frame_id = self.frameId
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]

        orientation = state.GetOrientation()
        rot = Rotation.from_euler('xyz', orientation, degrees = False)
        quat = rot.as_quat() # (x, y, z, w)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        return pose     