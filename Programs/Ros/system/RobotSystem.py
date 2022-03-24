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
            params = yaml.safe_load(stream)
        
        self.system = system_initialization(params)

        # load world and landmarks
        if world is not None:
            self.world = world
        else:
            print("Plase provide a world!")

        # load filter
        init_state_vals = np.array(params['initial_state_vals'])
        init_state = RobotState(
            orientation = init_state_vals[0:3],
            velocity    = init_state_vals[3:6],
            position    = init_state_vals[6:9]
        )
        init_state_cov = np.diag(params['initial_state_variance'])**2        
        init_state.SetCovariance(init_state_cov)

        filter_name = params['filter_name']
        self.filter = filter_initialization(self.system, init_state.GetMean(), init_state_cov, filter_name)

        # load data
        self.data_handler = DataHandler(self.system)

        self.sate_pub = path_publisher() # filter pose
        self.cmd_pub = path_publisher()  # theoratical command path
        self.gt_pub = path_publisher()   # actual robot path

    def run_filter(self):
        
        loopTime = rospy.get_time()
        while True:
            
            sample = self.data_handler.dataSampler.GetSample()
            if sample == False:
                return 

            # print("O_PRED:", self.filter.GetState())
            # print("ACCEL: ", sample.sensorValue)

            # update model
            self.filter.prediction(sample.sensorValue, sample.deltaT)
            
            # print("N_PRED:", self.filter.GetState())
            # print("CMD:   ", sample.commandState)
            # print("GT:    ", sample.groundTruthState)
            # print("")

            # publish estimate 
            esitmatedState = self.filter.GetState()
            esitmatedState.SetPosition(esitmatedState.GetPosition())
            self.sate_pub.publish_state_path(esitmatedState)
            self.sate_pub.publish_pose(esitmatedState)

            # publish ground truth 
            self.gt_pub.publish_gt_path(sample.groundTruthState)

            # publish command state
            self.cmd_pub.publish_command_path(sample.commandState)
 

            endLoopTime = rospy.get_time()
            deltaLoopTime = endLoopTime - loopTime
            loopTime = endLoopTime

            if deltaLoopTime < sample.deltaT:
                rospy.sleep(deltaLoopTime)
                 
    
def main():
    rob_sys = RobotSystem()

if __name__ == '__main__':
    main()