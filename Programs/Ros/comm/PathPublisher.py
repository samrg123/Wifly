import rospy
from comm.RosPublisher import RosPublisher

import numpy as np

from nav_msgs.msg import Path
class PathPublisher(RosPublisher):

    def __init__(self, frameId, topicId, queueSize = 10):
        
        super().__init__(frameId)

        self.topicId = topicId
        self.queueSize = queueSize

        self.path = Path()
        self.path.header.frame_id = self.frameId

        self.publisher = rospy.Publisher(topicId, Path, queue_size = queueSize)

    def Clear(self):
        self.path = Path()
        self.path.header.frame_id = self.frameId
        self.publisher = rospy.Publisher(self.topicId, Path, queue_size = self.queueSize)


    def PublishState(self, state):
        pose = self.StateToPose(state)
        self.path.poses.append(pose)
        self.publisher.publish(self.path)

    def PublishStates(self, states):

        # TODO: optimize
        for state in states:
            self.PublishState(state)
