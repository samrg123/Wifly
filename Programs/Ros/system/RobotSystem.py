import yaml
import matplotlib.pyplot as plt

import rospy

from system.RobotState import *

from comm.PathPublisher import PathPublisher
from comm.PoseWithCovariancePublisher import PoseWithCovariancePublisher

from utils.utils import *
from utils.DataHandler import *
from utils.filter_initialization import filter_initialization
from utils.system_initialization import system_initialization


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
        filter_name = params['filter_name']
        self.filter = filter_initialization(self.system, filter_name, params)

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

            # update model
            # print(sample.groundTruthState.GetPosition())
            self.filter.prediction(sample.sensorValue, sample.deltaT)
            self.filter.correction(sample.sensorValue, sample.deltaT)
            
            print("SENSOR:", sample.sensorValue)
            print("CMD:   ", sample.commandState)
            print("GT:    ", sample.groundTruthState)
            print("N_PRED:", self.filter.GetState())
            print("")

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