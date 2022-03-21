import numpy as np
import yaml
from scipy.io import loadmat
from data.generate_data import generateScript

from system.SensorValue import SensorValue
from system.RobotState import RobotState

class DataSample:
    def __init__(self, sensorValue, commandState, groundTruthState):
        self.sensorValue = sensorValue
        self.commandState = commandState
        self.groundTruthState = groundTruthState        

class DataHandler:

    def __init__(self):
        with open("config/settings.yaml", 'r') as stream:
            param = yaml.safe_load(stream)

        self.sampleIndex = 0

        dataType = param['dataType']
        self.dataType = dataType
        if dataType == "simulated":

            self.LoadSimulatedSamples()
            self.GetSample = self.GetSimulatedSample

        elif dataType == "streamed":
        
            Panic("Not Implemented")
        
        elif dataType == "log":
            Panic("Not Implemented")
        else:
            Panic(f"Unsupported dataType: {dataType}")
        


    def LoadSimulatedSamples(self):
        
        class Data:
            def __init__(self):

                with open("config/settings.yaml", 'r') as stream:
                    param = yaml.safe_load(stream)
                    alphas = np.array(param['alphas_sqrt'])**2 
                    beta = param['beta']/180*np.pi

                numSteps = 100
                deltaT = 0.1
                initialStateMean = [180,50,0]
                initialStateCov = np.eye(3)
                
                generatedData = generateScript(initialStateMean, numSteps, alphas, beta, deltaT)

                self.numSamples = generatedData.shape[0]

                self.motionCommand = np.array(generatedData[:,6:9]) # [Trans_vel,Angular_vel,gamma]' noisy control command

                self.actual_state = np.array(generatedData[:,15:18])
                self.noise_free_state = np.array(generatedData[:,18:21]) 

                self.noisefreeBearing_1 = np.array(generatedData[:, 9])
                self.noisefreeBearing_2 = np.array(generatedData[:, 12])

        self.data = Data()

    def GetSimulatedSample(self):
        
        t = self.sampleIndex
        if t >= self.data.numSamples:
            return False

        actualStateVec = self.data.actual_state[t,:]
        actualTheta = actualStateVec[2]
        groundTruthState = RobotState(
            position = actualStateVec[0:2],
            orientation = actualTheta
        )

        commandStateVec = self.data.noise_free_state[t]
        commandState = RobotState(
            position = commandStateVec[0:2],
            orientation = commandStateVec[2]
        )

        motionCommand = self.data.motionCommand[t,:]
        motionVelocity, angularVelocity = motionCommand[0:2]
        sensorValue = SensorValue(
            linearVelocity = np.array([
                motionVelocity * np.cos(actualTheta),
                motionVelocity * np.sin(actualTheta)
            ]),

            angularVelocity = angularVelocity
        )

        sample = DataSample(
            groundTruthState = groundTruthState,
            commandState = commandState,
            sensorValue = sensorValue
        )

        self.sampleIndex+= 1
        return sample

    def Reset(self):
        self.sampleIndex = 0
