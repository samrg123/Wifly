import numpy as np
from utils.utils import *

from data.DataSample import DataSample
from system.RobotState import RobotState
from system.SensorValue import SensorValue

class NormalNoise:
    
    def __init__(self, mean = np.zeros(3), covariance = np.diag(np.ones(3))):
        
        self.mean = mean
        self.covariance = covariance

    @staticmethod
    def Zero():
        return NormalNoise(np.zeros(3), np.zeros((3, 3)))

    def Sample(self):
        return gRng.multivariate_normal(self.mean, self.covariance)

class MotionNoise:

    def __init__(self, velocityNoise = NormalNoise(), orientationNoise = NormalNoise()):

        self.velocityNoise = velocityNoise
        self.orientationNoise = orientationNoise
 

class DataSimulator:
    
    def __init__(self, numSamples = 100, deltaT = .1, numStates = 5):
    
        self.sampleIndex = 0

        self.numSamples = numSamples
        self.deltaT = deltaT
        self.numStates = numStates

    def GenerateCommandStates(self):

        numStates = self.numStates
        stateTransitionWidth = .1 
        velocityNorm = 10

        commandStates = np.empty(self.numSamples, dtype="object")
        
        lastRobotState = RobotState()
        commandStates[0] = lastRobotState

        for i in range(self.numSamples):
            
            statePosition = numStates * (i / self.numSamples)
            state = statePosition % numStates
        
            if abs(statePosition - int(statePosition)) < .5*stateTransitionWidth:
                theta = np.radians(state * 360/numStates)
            else:
                theta = np.radians(int(state) * 360/numStates)

            robotState = RobotState(
                position    = lastRobotState.GetPosition() + lastRobotState.GetVelocity() * self.deltaT,
                velocity    = velocityNorm * np.array([np.cos(theta), np.sin(theta), 0]),
                orientation = [0, 0, theta]
            )

            lastRobotState = robotState
            commandStates[i] = robotState

        return commandStates

    # Note robotMotionNoise is noise caused by robot using motors
    #      worldMotionNoise is caused by external forces such as wind
    def ComputNoisyStates(self, commandStates, robotMotionNoise = MotionNoise(), worldMotionNoise = MotionNoise()):

        noisyStates = np.empty(len(commandStates), dtype="object")

        lastNoiseyState = RobotState()
        lastCommandSate = RobotState()
        
        for i, commandState in enumerate(commandStates):

            commandVelocity = commandState.GetVelocity()
            commandOrientation = commandState.GetOrientation()
            commandRotationMatrix = commandState.GetRotationMatrix()
            
            deltaCommandVelocity = commandVelocity - lastCommandSate.GetVelocity()
            deltaCommandOrientation = commandOrientation - lastCommandSate.GetOrientation()

            lastNoisyRotationMatrix = lastNoiseyState.GetRotationMatrix()

            # Add velocity noise from worldMotion and robotMotionNoise
            deltaNoisyVelocity = deltaCommandVelocity + worldMotionNoise.velocityNoise.Sample() + \
                                 lastNoisyRotationMatrix @ robotMotionNoise.velocityNoise.Sample()

            # Add orientation noise from worldMotion and robotMotionNoise
            deltaNoisyOrientation = deltaCommandOrientation + worldMotionNoise.orientationNoise.Sample() + \
                                    lastNoisyRotationMatrix @ robotMotionNoise.orientationNoise.Sample()
            
            noisyState = RobotState(
                position    = lastNoiseyState.GetPosition()    + lastNoiseyState.GetVelocity() * self.deltaT,
                velocity    = lastNoiseyState.GetVelocity()    + deltaNoisyVelocity,
                orientation = lastNoiseyState.GetOrientation() + deltaNoisyOrientation
            )

            lastCommandSate = commandState
            lastNoiseyState = noisyState
            noisyStates[i] = noisyState


        return noisyStates

    def ComputeSensorValues(self, states):

        sensorValues = np.empty(len(states), dtype="object")

        inverseDeltaT = 1./self.deltaT
        
        lastState = RobotState()
        for i, state in enumerate(states):

            sensorValue = SensorValue(
                angularVelocity = (state.GetOrientation() - lastState.GetOrientation()) * inverseDeltaT,
                linearAcceleration = (state.GetVelocity() - lastState.GetVelocity()) * inverseDeltaT 
            )

            lastState = state
            sensorValues[i] = sensorValue

        return sensorValues
            

    def LoadSamples(self):

        robotMotionNoise = MotionNoise(
            velocityNoise    = NormalNoise.Zero(),
            orientationNoise = NormalNoise.Zero()
        )

        worldMotionNoise = MotionNoise(
            velocityNoise    = NormalNoise.Zero(),
            orientationNoise = NormalNoise.Zero(),
        )

        self.commandStates      = self.GenerateCommandStates()
        self.groundTruthStates  = self.ComputNoisyStates(self.commandStates, robotMotionNoise, worldMotionNoise)
        self.sensorValues       = self.ComputeSensorValues(self.groundTruthStates)


    def GetSample(self):

        i = self.sampleIndex
        if i >= self.numSamples:
            return False

        sample = DataSample(
            deltaT = self.deltaT,
            sensorValue = self.sensorValues[i],
            commandState = self.commandStates[i],
            groundTruthState = self.groundTruthStates[i],
        )

        self.sampleIndex+= 1
        return sample


