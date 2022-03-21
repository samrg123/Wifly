import numpy as np
import yaml
from scipy.io import loadmat
from data.generate_data import generateScript

import os
from parse import parse

from system.SensorValue import SensorValue
from system.RobotState import RobotState
from utils.utils import *

class DataSample:
    def __init__(self, deltaT = 1, sensorValue = SensorValue(), commandState = RobotState(), groundTruthState = RobotState()):
        self.deltaT = deltaT
        self.sensorValue = sensorValue
        self.commandState = commandState
        self.groundTruthState = groundTruthState        

class DataHandler:

    def __init__(self):
        with open("config/settings.yaml", 'r') as stream:
            param = yaml.safe_load(stream)
        
        self.param = param
        self.sampleIndex = 0

        dataType = param['dataType']
        self.dataType = dataType
        if dataType == "simulated":

            self.LoadSimulatedSamples()
            self.GetSample = self.GetSimulatedSample

        elif dataType == "log":
        
            self.LoadLogSamples()
            self.GetSample = self.GetLogSample
        
        elif dataType == "streamed":
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
                self.deltaT = 0.1
                initialStateMean = [180,50,0]
                initialStateCov = np.eye(3)
                
                generatedData = generateScript(initialStateMean, numSteps, alphas, beta, self.deltaT)

                self.numSamples = generatedData.shape[0]

                # Note: generatedData multiplies linear velocity by deltaT so we divide it back to get m/s
                #       angular velocity is already in rad/sec
                self.motionCommand = np.array(generatedData[:,6:9]) # [Trans_vel,Angular_vel,gamma]' noisy control command
                self.motionCommand[:, 0]/= self.deltaT

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
        commandStateVec = self.data.noise_free_state[t]
        motionCommandVec = self.data.motionCommand[t,:]
        
        actualTheta = actualStateVec[2]
        motionVelocity = np.array([
            motionCommandVec[0] * np.cos(actualTheta),
            motionCommandVec[0] * np.sin(actualTheta)
        ])
        
        if t > 0:
            lastActualStateVec = self.data.actual_state[t-1,:]  
            lastCommandStateVec = self.data.noise_free_state[t-1]
            lastMotionCommandVect = self.data.motionCommand[t-1,:]

            actualVelocity = lastActualStateVec[0:2] - actualStateVec[0:2]
            commandVelocity = lastCommandStateVec[0:2] - commandStateVec[0:2]

            lastMotionVelocity = np.array([
                lastMotionCommandVect[0] * np.cos(lastActualStateVec[2]),
                lastMotionCommandVect[0] * np.sin(lastActualStateVec[2])
            ])

            motionAcceleration = motionVelocity - lastMotionVelocity

        else:
            actualVelocity = np.zeros(2)
            commandVelocity = np.zeros(2)
            motionAcceleration = motionVelocity
  
        inverseDeltaT = 1./self.data.deltaT

        groundTruthState = RobotState(
            position = actualStateVec[0:2],
            orientation = actualTheta,
            velocity = actualVelocity * inverseDeltaT
        )
        
        commandState = RobotState(
            position = commandStateVec[0:2],
            orientation = commandStateVec[2],
            velocity = commandVelocity * inverseDeltaT
        )

        sensorValue = SensorValue(
            linearAcceleration = motionAcceleration,
            angularVelocity = motionCommandVec[1]
        ) * inverseDeltaT

        sample = DataSample(
            deltaT = self.data.deltaT,
            groundTruthState = groundTruthState,
            commandState = commandState,
            sensorValue = sensorValue
        )

        self.sampleIndex+= 1
        return sample

    def LoadLogSamples(self):

        class Data:

            def __init__(self, sensorValueFileName):                
                with open(sensorValueFileName, "r") as file:
                    lines = file.readlines()

                serverMsgPrefix = "<- "
                serverMsgPrefixLen = len(serverMsgPrefix)

                self.sensorValues = []

                lastTimestamp = None
                timestamp = None
                linearAcceleration = None
                angularVelocity = None
                for lineIndex, line in enumerate(lines):

                    lineNumber = lineIndex+1                    
                    
                    if len(line) < serverMsgPrefixLen or line[0:serverMsgPrefixLen] != "<- ":
                        continue

                    serverMsg = line[serverMsgPrefixLen:]
                    if serverMsg == "\n":

                        if timestamp is None:
                            Warn(f"Missing timestamp in for sensorValue | {sensorValueFileName}:{lineNumber}")
                        
                        elif linearAcceleration is None:
                            Warn(f"Missing linearAcceleration in for sensorValue | {sensorValueFileName}:{lineNumber}")
                        
                        elif angularVelocity is None:
                            Warn(f"Missing angularVelocity in for sensorValue | {sensorValueFileName}:{lineNumber}")
                        
                        else:

                            # filter out all reapeated sensor values
                            if lastTimestamp != timestamp:                                
                            
                                # TODO: convert to 3D when ready! 
                                sensorValue = SensorValue(
                                    linearAcceleration = np.array(linearAcceleration[0:2]),
                                    angularVelocity = np.array(angularVelocity[2])
                                )
                                
                                # Note: log timestamp is in microseconds 
                                sensorValue.timestamp = timestamp * 1e-6

                                self.sensorValues.append(sensorValue)
                                lastTimestamp = timestamp

                        timestamp = None
                        linearAcceleration = None
                        angularVelocity = None                        

                    else:
                        
                        stripedServerMsg = serverMsg.strip()
                        splitServerMsg = stripedServerMsg.split(":")
                        if len(splitServerMsg) != 2:
                            if len(stripedServerMsg) > 0:
                                Log(f"Skipping serverMsg '{stripedServerMsg}' | {sensorValueFileName}:{lineNumber}")
                            continue

                        valueType, value = splitServerMsg
                        value = value.strip()

                        if valueType == "time":

                            if timestamp is not None:
                                Warn(f"Multiple timestamp values for sensorValue | original timestamp {timestamp} new timestamp {value} | {sensorValueFileName}:{lineNumber}")

                            try:
                                timestamp = int(value)
                            except Exception:
                                Warn(f"Failed to parse timestamp '{value}' | {sensorValueFileName}:{lineNumber}")
                                continue

                        elif valueType == "accel":

                            if linearAcceleration is not None:
                                Warn(f"Multiple linearAcceleration values for sensorValue | original timestamp {linearAcceleration} new timestamp {value} | {sensorValueFileName}:{lineNumber}")


                            try:
                                x, y, z = parse("{{ {:g}, {:g}, {:g} }}", value)
                            except Exception:
                                Warn(f"Failed to parse linearAcceleration '{value}' | {sensorValueFileName}:{lineNumber}")
                                continue

                            linearAcceleration = np.array([x, y, z])

                        elif valueType == "gyro":

                            if angularVelocity is not None:
                                Warn(f"Multiple angularVelocity values for sensorValue | original timestamp {angularVelocity} new timestamp {value} | {sensorValueFileName}:{lineNumber}")

                            try:
                                x, y, z = parse("{{ {:g}, {:g}, {:g} }}", value)
                            except Exception:
                                Warn(f"Failed to parse angularVelocity '{value}' | {sensorValueFileName}:{lineNumber}")
                                continue

                            angularVelocity = np.array([x, y, z])

                    self.numSamples = len(self.sensorValues)

        self.data = Data(os.path.abspath(self.param['dataSensorValueLog']))

    def GetLogSample(self):

        if self.sampleIndex >= self.data.numSamples:
            return False

        sensorValue = self.data.sensorValues[self.sampleIndex]

        if self.sampleIndex == 0:
            deltaT = 0

        else:
            prevSensorValue = self.data.sensorValues[self.sampleIndex-1]
            deltaT = sensorValue.timestamp - prevSensorValue.timestamp
            # print(deltaT)

        sample = DataSample(
            deltaT = deltaT,
            sensorValue = sensorValue,

            # TODO: Generate and parse these files!
            groundTruthState = RobotState(),
            commandState = RobotState()
        )

        self.sampleIndex+= 1
        return sample

    def Reset(self):
        self.sampleIndex = 0
