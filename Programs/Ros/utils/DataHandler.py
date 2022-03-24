import numpy as np
import yaml
from scipy.io import loadmat
from data.generate_data import generateScript

import os
from parse import parse

from system.SensorValue import SensorValue
from utils.utils import *

from data.DataSimulator import DataSimulator

from utils.system_initialization import gAccelerometerBias, gGyroBias

class DataHandler:

    def __init__(self):
        with open("config/settings.yaml", 'r') as stream:
            param = yaml.safe_load(stream)
        
        dataType = param['dataType']
        self.dataType = dataType
        
        if dataType == "simulated":

            self.dataSampler = DataSimulator()

        elif dataType == "log":
            pass 
            # self.LoadLogSamples()
            # self.GetSample = self.GetLogSample
        
        elif dataType == "streamed":
            Panic("Not Implemented")
        else:
            Panic(f"Unsupported dataType: {dataType}")
        
        self.dataSampler.LoadSamples()

    def LoadSimulatedSamples(self):
        
        class Data:
            def __init__(self):

                with open("config/settings.yaml", 'r') as stream:
                    param = yaml.safe_load(stream)

                numSteps = 100
                self.deltaT = 0.1
                initialStateCov = np.eye(3)
                alphas = np.array(param['alphas_sqrt'])**2 
                beta = param['beta']/180*np.pi
                initialStateMean = param['initial_state_vals'][3:6]
                
                # Note: prepend 0 acceleration to generated data so we can compute acceleration for first step
                scriptData = generateScript(initialStateMean, numSteps, alphas, beta, self.deltaT)
                generatedData = np.vstack((scriptData[0], scriptData))
                generatedData[0,6:9] = 0
                
                # offset inserted state by velocity
                generatedData[0,15:18]-= generatedData[1,15:18]
                generatedData[0,18:21]-= generatedData[1,18:21]



                self.numSamples = generatedData.shape[0]

                # Note: generatedData multiplies linear velocity by deltaT so we divide it back to get m/s
                #       angular velocity is already in rad/sec
                self.motionCommand = np.array(generatedData[:,6:9]) # [Trans_vel,Angular_vel,gamma]' noisy control command
                self.motionCommand[:, 0]/= self.deltaT

                self.actual_state = np.array(generatedData[:,15:18])
                self.noise_free_state = np.array(generatedData[:,18:21]) 

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
            lastT = t-1
            lastActualStateVec = self.data.actual_state[lastT,:]  
            lastCommandStateVec = self.data.noise_free_state[lastT]
            lastMotionCommandVec = self.data.motionCommand[lastT,:]

            actualVelocity = actualStateVec - lastActualStateVec 
            commandVelocity = commandStateVec - lastCommandStateVec 

            lastMotionVelocity = np.array([
                lastMotionCommandVec[0] * np.cos(lastActualStateVec[2]),
                lastMotionCommandVec[0] * np.sin(lastActualStateVec[2])
            ])

            # motionAcceleration = motionVelocity - lastMotionVelocity

        else:
            actualVelocity = np.zeros(3)
            commandVelocity = np.zeros(3)
            motionAcceleration = np.zeros(2)

        nextT = t+1
        if nextT < self.data.numSamples:

            nextActualStateVec = self.data.actual_state[nextT,:]
            nextMotionCommandVec = self.data.motionCommand[nextT,:]

            nextActualVelocity = nextActualStateVec - actualStateVec

            nextMotionVelocity = np.array([
                nextMotionCommandVec[0] * np.cos(nextActualStateVec[2]),
                nextMotionCommandVec[0] * np.sin(nextActualStateVec[2])
            ])

            # motionAcceleration = nextMotionVelocity - motionVelocity
            motionAcceleration = (nextActualVelocity[0:2] - actualVelocity[0:2]) / self.data.deltaT
            # motionAcceleration[0]*= np.cos(actualTheta)
            # motionAcceleration[0]*= np.cos(actualTheta)
            
            # motionAcceleration = np.zeros(2)

        else:
            motionAcceleration = np.zeros(2)

        inverseDeltaT = 1./self.data.deltaT

        groundTruthState = RobotState(
            position    = np.append(actualStateVec[0:2], 0),
            orientation = np.array([0, 0, actualTheta]),
            velocity    = np.append(actualVelocity[0:2] * inverseDeltaT, 0)
        )
        
        commandState = RobotState(
            position    = np.append(commandStateVec[0:2], 0),
            orientation = np.array([0, 0, commandStateVec[2]]),
            velocity    = np.append(commandVelocity[0:2] * inverseDeltaT, 0)
        )

        sensorValue = SensorValue(
            linearAcceleration = np.append(motionAcceleration * inverseDeltaT, 0) + gAccelerometerBias.reshape(-1),
            # angularVelocity    = np.append([0, 0], motionCommandVec[1] * inverseDeltaT)
            angularVelocity    = np.append([0, 0], actualVelocity[2] * inverseDeltaT) + gGyroBias.reshape(-1)
        )

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
                             
                                sensorValue = SensorValue(
                                    linearAcceleration = linearAcceleration,
                                    angularVelocity = angularVelocity
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
