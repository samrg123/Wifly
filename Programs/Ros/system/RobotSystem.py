import yaml
import matplotlib.pyplot as plt

import rospy

from system.RobotState import *
from system.SensorValue import *

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

        # load data
        self.data_handler = DataHandler(self.system)

        # load filter
        # Warn: Needs to be loaded after data so datahandler can modify self.system  
        filter_name = params['filter_name']
        self.filter = filter_initialization(self.system, filter_name, params)

        # create Ros publishers
        frameId = params['frameId']
        self.groundTruthPath    = PathPublisher(frameId, params["gt_path_topic"])
        self.commandPath        = PathPublisher(frameId, params["command_path_topic"])
        self.predictedPath      = PathPublisher(frameId, params["path_topic"])
        self.poseWithCovariance = PoseWithCovariancePublisher(frameId, params["pose_topic"])

        self.particlePath      = PathPublisher(frameId, params["particlePathTopic"])
        self.integrationPath   = PathPublisher(frameId, params["integrationPathTopic"])

    def run_filter(self):
        
        loopTime = rospy.get_rostime()
        while True:
            
            sample = self.data_handler.dataSampler.GetSample()
            if sample == False:
                return 

            lastState = self.filter.GetState()
            # print("O_PRED:", lastState)
            lastRotationMatrix = lastState.GetRotationMatrix()


            # print("Particles BEFORE:")
            # [print(p) for p in self.filter.GetParticleStates()]
            # print("")


            # update model
            self.filter.prediction(sample.sensorValue, sample.deltaT)
            self.filter.correction(sample.sensorValue, sample.deltaT)
            
            predictedState = self.filter.GetState()
            correctedSensorValue = SensorValue.Copy(sample.sensorValue)
            correctedSensorValue.linearAcceleration -= lastRotationMatrix @ self.system.accelerometerBias
            correctedSensorValue.angularVelocity -= lastRotationMatrix @ self.system.gyroBias

            print("SENSOR:", sample.sensorValue)
            print("CORRET:", correctedSensorValue)
            print("CMD:   ", sample.commandState)
            print("GT:    ", sample.groundTruthState)
            print("N_PRED:", predictedState)            
            print("")

            # Publish data to rviz
            esitmatedState = self.filter.GetState()
            self.predictedPath.PublishState(esitmatedState)
            self.poseWithCovariance.PublishState(esitmatedState)

            self.commandPath.PublishState(sample.commandState)
            self.groundTruthPath.PublishState(sample.groundTruthState)

            self.particlePath.Clear()
            particleStates = self.filter.GetParticleStates()
            self.particlePath.PublishStates(particleStates)
            
            integrationState = self.filter.GetIntegrationState()
            self.integrationPath.PublishState(integrationState)

            # print("Particles AFTER:")
            # [print(p) for p in particleStates]
            # print("")

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