import sys
sys.path.append('.')
import yaml
import matplotlib.pyplot as plt

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

from system.RobotState import *
from comm.path_publisher import *
from utils.DataHandler import *
from utils.filter_initialization import filter_initialization
from utils.system_initialization import system_initialization
from utils.utils import *

from system.SensorValue import SensorValue

class RobotSystem:

    def __init__(self, world=None):

        rospy.init_node('robot_state_estimator', anonymous=True)

        # load params
        with open("config/settings.yaml", 'r') as stream:
            param = yaml.safe_load(stream)
        
        # load motion noise and sensor noise
        alphas = np.array(param['alphas_sqrt'])**2
        self.system = system_initialization(alphas)

        # load world and landmarks
        if world is not None:
            self.world = world
        else:
            print("Plase provide a world!")

        # load filter
        init_state_mean = np.array(param['initial_state_mean'])
        init_state_cov = np.diag(param['initial_state_variance'])**2        
        filter_name = param['filter_name']
        self.filter = filter_initialization(self.system, init_state_mean, init_state_cov, filter_name)

        # load data
        self.data_handler = DataHandler()

        self.sate_pub = path_publisher() # filter pose
        self.cmd_pub = path_publisher()  # theoratical command path
        self.gt_pub = path_publisher()   # actual robot path

        self.loop_sleep_time = param['loop_sleep_time']

    def run_filter(self):
        
        while True:
            
            sample = self.data_handler.GetSample()
            if sample == False:
                return 

            # update model
            self.filter.prediction(sample.sensorValue)

            # publish estimate 
            esitmatedState = self.filter.getState()
            self.sate_pub.publish_state_path(esitmatedState)
            self.sate_pub.publish_pose(esitmatedState)

            # publish ground truth 
            self.gt_pub.publish_gt_path(sample.groundTruthState.getMeanVector())

            # publish command state
            self.cmd_pub.publish_command_path(sample.commandState.getMeanVector())

            # TODO: account for lag
            rospy.sleep(self.loop_sleep_time)
    
def main():
    rob_sys = RobotSystem()

if __name__ == '__main__':
    main()