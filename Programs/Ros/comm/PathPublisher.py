import rospy
from comm.RosPublisher import RosPublisher

import numpy as np

from nav_msgs.msg import Path
class PathPublisher(RosPublisher):

    def __init__(self, 
                 frameId, 
                 topicId, 
                 queueSize = 1,
                 frameTimeLimit = 1./30):
        
        super().__init__(frameId, frameTimeLimit)

        self.topicId = topicId
        self.queueSize = queueSize

        self.path = Path()
        self.path.header.frame_id = self.frameId

        self.publisher = rospy.Publisher(topicId, Path, queue_size = queueSize)

    def Clear(self):
        self.path.poses.clear()
        self.publisher.publish(self.path)

    def PublishState(self, state):

        pose = self.StateToPose(state)
        self.path.poses.append(pose)

        self.TryUpdate(lambda : self.publisher.publish(self.path))

    def PublishStates(self, states):

        # TODO: Optimize this
        for state in states:
            pose = self.StateToPose(state)
            self.path.poses.append(pose)

        self.TryUpdate(lambda : self.publisher.publish(self.path))