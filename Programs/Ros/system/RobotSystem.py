import sys
sys.path.append('.')
import yaml
import matplotlib.pyplot as plt

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

from system.RobotState import *
from comm.PathPublisher import PathPublisher
from comm.PoseWithCovariancePublisher import PoseWithCovariancePublisher
from utils.DataHandler import *
from utils.filter_initialization import filter_initialization
from utils.system_initialization import system_initialization
from utils.utils import *

import os
sys.path.append(os.path.abspath(os.path.join(os.path.join(os.path.abspath(__file__), '..', '..', '..', '..', 'wifly2'))))
from scripts.intensity_client_test import gen_pt, intensity_query_client

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
        init_state_cov = np.diag(params['initial_state_variance'])
        init_state.SetCovariance(init_state_cov)

        filter_name = params['filter_name']
        self.filter = filter_initialization(self.system, init_state.GetMean(), init_state_cov, filter_name, params)

        # load data
        self.data_handler = DataHandler(self.system)

        # create Ros publishers
        frameId = params['frameId']
        self.groundTruthPath    = PathPublisher(frameId, params["gt_path_topic"])
        self.commandPath        = PathPublisher(frameId, params["command_path_topic"])
        self.predictedPath      = PathPublisher(frameId, params["path_topic"])
        self.poseWithCovariance = PoseWithCovariancePublisher(frameId, params["pose_topic"])

    def run_filter(self):
        
        loopTime = rospy.get_rostime()
        while True:
            
            sample = self.data_handler.dataSampler.GetSample()
            if sample == False:
                return 

            # print("O_PRED:", self.filter.GetState())
            # print("ACCEL: ", sample.sensorValue)

            # update model
            # print(sample.groundTruthState.GetPosition())
            self.filter.prediction(sample.sensorValue, sample.deltaT)
            # self.filter.correction(intensity_query_client(gen_pt(sample.groundTruthState.GetPosition())).intensity)
            
            # print("N_PRED:", self.filter.GetState())
            # print("CMD:   ", sample.commandState)
            # print("GT:    ", sample.groundTruthState)
            # print("")

            # Publish data to rviz
            esitmatedState = self.filter.GetState()
            self.predictedPath.PublishState(esitmatedState)
            self.poseWithCovariance.PublishState(esitmatedState)

            self.commandPath.PublishState(sample.commandState)
            self.groundTruthPath.PublishState(sample.groundTruthState)

            # Delay until next state
            endLoopTime = rospy.get_rostime()
            deltaLoopTime = endLoopTime - loopTime
            sampleDuration = rospy.Duration(sample.deltaT)

            sleepThreshold = rospy.Duration(.05)
            if sampleDuration - deltaLoopTime > sleepThreshold:
                rospy.sleep(deltaLoopTime)
            else:
                while True:
                    ellapsedTime = rospy.get_rostime() - loopTime
                    
                    if ellapsedTime >= sampleDuration:
                        break

            loopTime = endLoopTime
                 


    
def main():
    rob_sys = RobotSystem()

if __name__ == '__main__':
    main()