import yaml

from utils.utils import *

from system.RobotState import RobotState
from data.DataSample import DataSample
from data.DataSimulator import DataSimulator
from data.LogSampler import LogSampler

class DataHandler:

    def __init__(self, system):

        self.system = system

        # TODO: pass these params in instead of opening new handle
        with open("config/settings.yaml", 'r') as stream:
            params = yaml.safe_load(stream)
        
        dataType = params['dataType']
        self.dataType = dataType
        
        if dataType == "simulated":

            self.dataSampler = DataSimulator(params, system)

        elif dataType == "log":
            
            self.dataSampler = LogSampler(params, system)
        
        elif dataType == "streamed":
            Panic("Not Implemented")
        else:
            Panic(f"Unsupported dataType: {dataType}")
        
        self.dataSampler.LoadSamples()

        initialStateSamples = GetParam(params, "computeInitialStateSamples", 0)
        biasSamples         = GetParam(params, "computeBiasSamples", 0)

        # Warn: InitialState must be computed before bias
        self.ComputeInitialState(initialStateSamples)
        self.ComputeBias(biasSamples)

    def ComputeBias(self, numSamples):

        if numSamples <= 0:
            return

        system = self.system

        system.gyroBias[:] = 0
        system.velocityBias[:] = 0
        system.accelerometerBias[:] = 0

        for i in range(numSamples):

            sample = self.dataSampler.GetSample(sampleType = DataSample.Type.ComputeBias)
            if sample == False:
                numSamples = i+1
                break

            # NOTE: We currently use sensor values to estimate orientation and bias. 
            #       This is really just a hack to average real world data samples into something usable
            # TODO: comparing these values with ground truth values once we get real world data

            angularVelocity = sample.sensorValue.GetAngularVelocity()
            linearAcceleration = sample.sensorValue.GetLinearAcceleration()

            system.gyroBias+= angularVelocity
            system.accelerometerBias+= linearAcceleration

            # TODO: Think of better way to compute velocity bias.
            if sample.deltaT != 0:
                system.velocityBias+= (sample.groundTruthState.GetVelocity() - sample.commandState.GetVelocity()) / sample.deltaT

        self.dataSampler.Reset()
        
        normailzer = 1./numSamples
        rotationMatrix = system.initialState.GetRotationMatrix()
        
        # TODO: Make a function that can track bias over time?
        system.gyroBias          = rotationMatrix.T @ (normailzer * system.gyroBias) 
        system.velocityBias      = rotationMatrix.T @ (normailzer * system.velocityBias)
        system.accelerometerBias = rotationMatrix.T @ (normailzer * system.accelerometerBias) 

        Log(f"Computed Biases ({numSamples} samples) {{\n\
                \tgyroBias: {system.gyroBias}\n\
                \tvelocityBias: {system.velocityBias}\n\
                \taccelerometerBias: {system.accelerometerBias}\n\
            }}")

    def ComputeInitialState(self, numSamples):
       
        if numSamples <= 0:
            return

        system = self.system

        robotStates = np.array([], dtype="object")
        for i in range(numSamples):

            sample = self.dataSampler.GetSample(sampleType = DataSample.Type.ComputeInitialState)
            if sample == False:
                numSamples = i+1
                break

            robotState = RobotState.Copy(sample.groundTruthState)

            # Note: can't infer orientation dirction if zAxis is constrained, becuase sensed accelerationDirection won't point down due to noise
            #       Just use the ground truth values instead
            if system.constrainZAxis == False:

                linearAcceleration = sample.sensorValue.GetLinearAcceleration()
                accelerationDirection = linearAcceleration / np.linalg.norm(linearAcceleration)

                zAxis = [0, 0, -1]
                rotation, rmsDistance = Rotation.align_vectors([accelerationDirection], [zAxis])
                rotationMatrix = rotation.as_matrix()

                robotState.SetRotationMatrix(rotationMatrix)

            robotStates = np.append(robotStates, robotState)

        self.dataSampler.Reset()
        
        system.initialState = RobotState.MeanState(robotStates)
       
        Log(f"Computed InitialState ({numSamples} samples) {{\n\
                \tInitialState: {system.initialState}\n\
            }}")


