import rospy
from comm.RosPublisher import RosPublisher

from nav_msgs.msg import Path
class PathPublisher(RosPublisher):

    def __init__(self, frameId, topicId, queueSize = 10):
        
        super().__init__(frameId)

        self.path = Path()
        self.path.header.frame_id = self.frameId

        self.publisher = rospy.Publisher(topicId, Path, queue_size = queueSize)

    def PublishState(self, state):
        pose = self.StateToPose(state)
        self.path.poses.append(pose)
        self.publisher.publish(self.path)