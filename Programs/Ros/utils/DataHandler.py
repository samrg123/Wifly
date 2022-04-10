import yaml

from utils.utils import *

from system.RobotState import RobotState
from data.DataSimulator import DataSimulator
from data.LogSampler import LogSampler

class DataHandler:

    def __init__(self, system):

        with open("config/settings.yaml", 'r') as stream:
            params = yaml.safe_load(stream)
        
        dataType = params['dataType']
        self.dataType = dataType
        
        if dataType == "simulated":

            self.dataSampler = DataSimulator(params, system)

        elif dataType == "log":
            
            self.dataSampler = LogSampler(params)
        
        elif dataType == "streamed":
            Panic("Not Implemented")
        else:
            Panic(f"Unsupported dataType: {dataType}")
        
        self.dataSampler.LoadSamples()

        computeBiasAndMeanSamples = GetParam(params, "computeBiasAndMeanSamples", 0)
        
        if computeBiasAndMeanSamples > 0:
            
            robotStates = np.array([])
            system.gyroBias[:] = 0
            system.velocityBias[:] = 0
            system.accelerometerBias[:] = 0

            for i in range(computeBiasAndMeanSamples):

                sample = self.dataSampler.GetSample()
                if sample == False:
                    computeBiasAndMeanSamples = i+1
                    break

                position = sample.groundTruthState.GetPosition()
                velocity = sample.groundTruthState.GetVelocity()

                # NOTE: We currently use sensor values to estimate orienation and bias. 
                #       This is really just a hack to average real world data samples into something usable
                # TODO: comparing these values with ground truth values once we get real world data

                angularVelocity = sample.sensorValue.GetAngularVelocity()
                linearAcceleration = sample.sensorValue.GetLinearAcceleration()

                system.gyroBias+= angularVelocity
                system.accelerometerBias+= linearAcceleration

                accelerationDirection = linearAcceleration / np.linalg.norm(linearAcceleration)

                # TODO: THIS IS WRONG. ORIENTATION DOESN'T WORK WITH EULER DIRECTION...
                #       NEED TO IMPLEMET LOOK AT FUNCTION OR SOMETHING SIMILAR
                # xAxis = np.cross[0, 1, 0] 
                print(accelerationDirection)
                cosX = [1, 0, 0] @ accelerationDirection
                cosY = [0, 1, 0] @ accelerationDirection
                cosZ = [0, 0, 1] @ accelerationDirection

                robotState = RobotState(
                    position = position,
                    velocity = velocity,
                    orientation = sample.groundTruthState.GetOrientation()
                    # orientation = np.arccos([cosX, cosY, cosZ ]) 
                )
                robotStates = np.append(robotStates, robotState)

                # TODO: Think of better way to compute velocity bias.
                if sample.deltaT != 0:
                    system.velocityBias+= (sample.groundTruthState.GetVelocity() - sample.commandState.GetVelocity()) / sample.deltaT

            normailzer = 1./computeBiasAndMeanSamples
            
            # compute mean state
            system.initialState = RobotState.MeanState(robotStates)
            
            meanPosition    = system.initialState.GetPosition() 
            meanVelocity    = system.initialState.GetVelocity() 
            meanOrientation = system.initialState.GetOrientation() 
            
            variancePosition    = np.mean([ (state.GetPosition() - meanPosition)**2 for state in robotStates ], axis = 0)
            varianceVelocity    = np.mean([ (state.GetVelocity() - meanVelocity)**2 for state in robotStates ], axis = 0)
            varianceOrientation = np.mean([ (state.GetOrientation() - meanOrientation)**2 for state in robotStates ], axis = 0)

            system.initialState.SetPositionCovariance(np.diag(variancePosition))
            system.initialState.SetVelocityCovariance(np.diag(varianceVelocity))
            system.initialState.SetOrientationCovariance(np.diag(varianceOrientation))

            rotationMatrix = system.initialState.GetRotationMatrix()
            
            system.gyroBias          = rotationMatrix.T @ (normailzer * system.gyroBias) 
            system.velocityBias      = rotationMatrix.T @ (normailzer * system.velocityBias)
            system.accelerometerBias = rotationMatrix.T @ (normailzer * system.accelerometerBias)

            Log(f"Computed Biases and Mean ({computeBiasAndMeanSamples} samples) {{\n\
                  \tgyroBias: {system.gyroBias}\n\
                  \tvelocityBias: {system.velocityBias}\n\
                  \taccelerometerBias: {system.accelerometerBias}\n\
                  \tInitialState: {system.initialState}\n\
                  }}")

            # exit(0)

            self.dataSampler.Reset()
