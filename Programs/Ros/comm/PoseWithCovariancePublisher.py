from comm.RosPublisher import RosPublisher

import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped

class PoseWithCovariancePublisher(RosPublisher):

    def __init__(self, frameId, topicId, queueSize = 1, frameTimeLimit = 1./30):
        
        super().__init__(frameId, frameTimeLimit)

        self.pose_pub = rospy.Publisher(topicId, PoseWithCovarianceStamped, queue_size = queueSize)

    def PublishState(self, state):

        def updateFunction():

            msg = PoseWithCovarianceStamped()
            msg.header.stamp = rospy.get_rostime()
            msg.header.frame_id = self.frameId
            msg.pose.pose = self.StateToPose(state).pose

            # We wish to visualize 3 sigma contour
            cov = np.zeros((6,6))
            cov[0:2,0:2] = state.GetPositionCovariance()[0:2,0:2]
            msg.pose.covariance = np.reshape(9*cov,(-1,)).tolist()
                
            self.pose_pub.publish(msg)

        self.TryUpdate(updateFunction)