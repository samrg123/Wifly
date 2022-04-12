import numpy as np
from utils.utils import *

from data.DataSample import DataSample
from system.RobotState import RobotState
from system.SensorValue import SensorValue
from scipy.linalg import logm

from utils.noiseUtils import *

class DataSimulator:
    
    def __init__(self, params, system):
    
        self.sampleIndex = 0
        self.params = params
        self.system = system

        self.worldMotionNoise = system.worldMotionNoise
        self.robotMotionNoise = system.robotMotionNoise
        
        self.sensorNoise = system.sensorNoise

        self.pathDuration = GetParam(params, "pathDuration", 3) 
        self.pathRadius   = GetParam(params, "pathRadius", 1) 
        self.numSamples   = GetParam(params, "numSamples", 100)
        
        self.deltaT = self.pathDuration / self.numSamples
        self.velocityNorm = 2*np.pi * self.pathRadius / self.pathDuration

        self.numStates = GetParam(params, "numStates",  4)
        self.stateTransitionWidth = GetParam(params, "stateTransitionWidth", .1)

    def Reset(self):
        self.sampleIndex = 0

    def GenerateCommandStates(self):

        commandStates = np.empty(self.numSamples, dtype="object")
        
        initialState = self.system.initialState
        iniitalLinearVelocity = initialState.GetVelocity()
        initialOrientation = initialState.GetOrientation()

        lastRobotState   = initialState
        commandStates[0] = RobotState.Copy(lastRobotState)

        numStates = self.numStates
        for i in range(1, self.numSamples):
            
            statePosition = numStates * (i / self.numSamples)
            state = statePosition % numStates
        
            if (statePosition - int(statePosition)) >= (1 - self.stateTransitionWidth):
                theta = np.radians(state * 360/numStates)
            else:
                theta = np.radians(int(state) * 360/numStates)

            orientation = initialOrientation + [0, 0, theta]

            deltaOrientation = orientation - lastRobotState.GetOrientation()

            if i == 1:
                # initial kick to get robot following path
                deltaLinearVelocity = [self.velocityNorm, 0, 0]
            else:
                deltaLinearVelocity = np.zeros(3)

            robotState = RobotState(
                orientation = orientation,
                velocity    = self.system.GammaLinearVelocity(lastRobotState, deltaOrientation, deltaLinearVelocity),
                position    = self.system.GammaPosition(lastRobotState, deltaOrientation, deltaLinearVelocity, self.deltaT),
            )

            lastRobotState = robotState
            commandStates[i] = robotState

        return commandStates

    # Note robotMotionNoise is noise caused by robot using motors
    #      worldMotionNoise is caused by external forces such as wind
    def ComputNoisyStates(self, commandStates, robotMotionNoise = MotionNoise(), worldMotionNoise = MotionNoise()):

        numCommandStates = len(commandStates)
        noisyStates = np.empty(numCommandStates, dtype="object")

        lastNoiseyState = commandStates[0]
        lastCommandSate = commandStates[0]
        noisyStates[0]  = commandStates[0]

        for i in range(1, numCommandStates):
            
            commandState = commandStates[i]

            commandVelocity = commandState.GetVelocity()
            commandOrientation = commandState.GetOrientation()
            commandRotationMatrix = commandState.GetRotationMatrix()
            
            lastNoisyRotationMatrix = lastNoiseyState.GetRotationMatrix()
            deltaCommandOrientation = self.system.GammaDeltaOrientation(lastCommandSate, commandState)
            deltaCommandVelocity    = self.system.GammaDeltaLinearVelocity(lastCommandSate, commandState, deltaCommandOrientation)

            worldVelocityNoiseValue = worldMotionNoise.velocityNoise.Sample()
            robotVelocityNoiseValue = robotMotionNoise.velocityNoise.Sample()

            worldOrientationNoiseValue = worldMotionNoise.orientationNoise.Sample()
            robotOrientationNoiseValue = robotMotionNoise.orientationNoise.Sample()

            # Add velocity noise from worldMotion and robotMotionNoise
            deltaNoisyVelocity = deltaCommandVelocity + \
                                 lastNoisyRotationMatrix.T @ lastNoisyRotationMatrix.T @ worldVelocityNoiseValue + \
                                 lastNoisyRotationMatrix.T @ robotVelocityNoiseValue

            # Add orientation noise from worldMotion and robotMotionNoise
            deltaNoisyOrientation = deltaCommandOrientation + \
                                    lastNoisyRotationMatrix.T @ worldOrientationNoiseValue + \
                                    robotOrientationNoiseValue

            noisyState = RobotState(
                position    = self.system.GammaPosition(lastNoiseyState, deltaNoisyOrientation, deltaNoisyVelocity, self.deltaT),
                velocity    = self.system.GammaLinearVelocity(lastNoiseyState, deltaNoisyOrientation, deltaNoisyVelocity),
            )

            noisyState.SetRotationMatrix(self.system.GammaRotationMatrix(lastNoiseyState, deltaNoisyOrientation))

            lastCommandSate = commandState
            lastNoiseyState = noisyState
            noisyStates[i] = noisyState


        return noisyStates

    def ComputeSensorValues(self, states, sensorNoise):

        sensorValues = np.empty(len(states), dtype="object")

        lastState = states[0]
        for i, state in enumerate(states):

            sensorValue = self.system.GammaSensorValue(lastState, state, self.deltaT)

            sensorValue.angularVelocity+=    sensorNoise.gyroNoise.Sample()
            sensorValue.linearAcceleration+= sensorNoise.accelerometerNoise.Sample()
            sensorValue.SetRssi(sensorValue.GetRssi() + sensorNoise.rssiNoise.Sample())

            lastState = state
            sensorValues[i] = sensorValue

        return sensorValues
            

    def LoadSamples(self):

        self.commandStates      = self.GenerateCommandStates()
        self.groundTruthStates  = self.ComputNoisyStates(self.commandStates, self.robotMotionNoise, self.worldMotionNoise)
        self.sensorValues       = self.ComputeSensorValues(self.groundTruthStates, self.sensorNoise)


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


