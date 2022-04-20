import yaml
import matplotlib.pyplot as plt

import rospy
import signal
import numpy as np

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

        # Register sigint
        signal.signal(signal.SIGINT, self.sigIntHandler)

        # disable line-breaks while printing np arrays
        np.set_printoptions(threshold=np.inf)
        np.set_printoptions(linewidth=np.inf)

        rospy.init_node('robot_state_estimator', anonymous=True)

        # load params
        with open("config/settings.yaml", 'r') as stream:
            params = yaml.safe_load(stream)

        self.system = system_initialization(params)
        
        # TODO: MOVE THIS INTO wifiMap.publish() or something like that...
        wifiMap = GetParam(params, "wifiMap")
        if wifiMap is not None:

            wifiData = np.loadtxt(wifiMap)
            wifiMapOrigin     = GetParam(params, "wifiMapOrigin", np.zeros(3))
            wifiMapResolution = GetParam(params, "wifiMapResolution", .01)

            from scripts.intensity_client_test import publish_intensity_map
            publish_intensity_map(wifiData, wifiMapResolution, wifiMapOrigin)

        self.realtimeSim = GetParam(params, "realtimeSim", True)
    
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
        fpsLimit = GetParam(params, "rvizFPSLimit", 30)
        frameTimeLimit = 1./fpsLimit if fpsLimit > 0 else 0 
        self.groundTruthPath    = PathPublisher(frameId, params["gt_path_topic"],       frameTimeLimit = frameTimeLimit)
        self.commandPath        = PathPublisher(frameId, params["command_path_topic"],  frameTimeLimit = frameTimeLimit)
        self.predictedPath      = PathPublisher(frameId, params["path_topic"],          frameTimeLimit = frameTimeLimit)
        self.poseWithCovariance = PoseWithCovariancePublisher(frameId, params["pose_topic"], frameTimeLimit = frameTimeLimit)

        self.particlePath      = PathPublisher(frameId, params["particlePathTopic"],    frameTimeLimit = frameTimeLimit)
        self.integrationPath   = PathPublisher(frameId, params["integrationPathTopic"], frameTimeLimit = frameTimeLimit)

    @staticmethod
    def sigIntHandler(signum, frame):
        print("SIGINT Recieved - Exiting")
        exit(0)

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


            # Publish data to rviz
            esitmatedState = self.filter.GetState()
            self.predictedPath.PublishState(esitmatedState)
            self.poseWithCovariance.PublishState(esitmatedState)

            self.commandPath.PublishState(sample.commandState)
            
            # self.groundTruthPath.Clear()
            self.groundTruthPath.PublishState(sample.groundTruthState)

            self.particlePath.Clear()
            particleStates = self.filter.GetParticleStates()
            self.particlePath.PublishStates(particleStates)
            
            integrationState = self.filter.GetIntegrationState()
            self.integrationPath.PublishState(integrationState)

            # print("DELTA: ", sample.deltaT)
            print("SENSOR:", sample.sensorValue)
            # print("CORRET:", correctedSensorValue)
            # print("CMD:   ", sample.commandState)
            print("GT:    ", sample.groundTruthState)
            print("N_PRED:", predictedState)
            print("INT:   ", integrationState)
            # print("")


            # print("Particles AFTER:")
            # [print(p) for p in particleStates]
            # print("")

            # Delay until next state
            if self.realtimeSim:
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