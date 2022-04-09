
import sys
from tkinter.ttk import Labelframe
sys.path.append('.')

import yaml

from scipy.spatial.transform import Rotation
from scipy.linalg import expm, logm

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from system.RobotState import *

class path_publisher:
    def __init__(self):

        # load params
        with open("config/settings.yaml", 'r') as stream:
            param = yaml.safe_load(stream)
        
        pose_topic    = param['pose_topic']
        path_topic    = param['path_topic']
        ellipse_topic = param['ellipse_topic']

        gt_path_topic       = param['gt_path_topic']
        command_path_topic  = param['command_path_topic']

        self.path_frame = param['path_frame_id']

        self.path = Path()
        self.path.header.frame_id = self.path_frame

        self.gt_path = Path()
        self.gt_path.header.frame_id = self.path_frame

        self.pose_pub = rospy.Publisher(pose_topic,PoseWithCovarianceStamped,queue_size=100)
        self.path_pub = rospy.Publisher(path_topic,Path,queue_size=10)
        self.gt_path_pub = rospy.Publisher(gt_path_topic,Path,queue_size=10)
        self.cmd_path_pub = rospy.Publisher(command_path_topic,Path,queue_size=10)
        self.ellipse_pub = rospy.Publisher(ellipse_topic,Marker,queue_size=10)
 

    def publish_pose(self, state):

        position = state.GetPosition()
        orientation = state.GetOrientation()

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = self.path_frame
        msg.pose.pose.position.x = position[0]
        msg.pose.pose.position.y = position[1]
        msg.pose.pose.position.z = position[2]

        rot = Rotation.from_euler('xyz',orientation,degrees=False)
        quat = rot.as_quat()    # (x, y, z, w)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

        # We wish to visualize 3 sigma contour
        cov = np.zeros((6,6))
        cov[0:2,0:2] = state.GetPositionCovariance()[0:2,0:2]
        msg.pose.covariance = np.reshape(9*cov,(-1,)).tolist()
            
        self.pose_pub.publish(msg)
        

    def publish_state_path(self, state):

        position = state.GetPosition()
        orientation = state.GetOrientation()

        pose = PoseStamped()
        pose.header.stamp = rospy.get_rostime()
        pose.header.frame_id = self.path_frame
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]

        rot = Rotation.from_euler('xyz',orientation,degrees=False)
        quat = rot.as_quat()    # (x, y, z, w)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        self.path.poses.append(pose)

        self.path_pub.publish(self.path)

    def publish_gt_path(self, state):

        position = state.GetPosition()
        orientation = state.GetOrientation()

        pose = PoseStamped()
        pose.header.stamp = rospy.get_rostime()
        pose.header.frame_id = self.path_frame
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]

        rot = Rotation.from_euler('xyz',orientation,degrees=False)
        quat = rot.as_quat()    # (x, y, z, w)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        self.path.poses.append(pose)

        self.gt_path_pub.publish(self.path)

    def publish_command_path(self,state):

        position = state.GetPosition()
        orientation = state.GetOrientation()

        pose = PoseStamped()
        pose.header.stamp = rospy.get_rostime()
        pose.header.frame_id = self.path_frame
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]

        rot = Rotation.from_euler('xyz',orientation,degrees=False)
        quat = rot.as_quat()    # (x, y, z, w)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        self.path.poses.append(pose)

        self.cmd_path_pub.publish(self.path)

def main():

    state = RobotState()
    path_pub = path_publisher()

    path_pub.make_ellipse()

    pass

if __name__ == '__main__':
    main()