from parse import parse

from utils.utils import *
from data.DataSample import DataSample

from system.RobotState import RobotState
from system.SensorValue import SensorValue

import csv

class LogSampler:
    
    def __init__(self, params):

        self.Reset()

        self.params = params

        self.mocapFileName       = GetParam(params, "dataMocapLog") 
        self.sensorValueFileName = GetParam(params, "dataSensorValueLog") 
        self.serverMsgPrefix     = GetParam(params, "serverMsgPrefix", "<- ")

    def Reset(self):
        self.sensorIndex = 0
        self.groundTruthIndex = 0

        self.sensorTime = 0
        self.groundTruthTime = 0

    def LoadMocapSamples(self, path):

        self.groundTruthStates = []

        with open(path, newline='') as file:
            csvReader = csv.reader(file, delimiter = ',')

            header = next(csvReader)

            for row in csvReader:

                timestamp = int(row[1]) * 1e-6
                position = np.array([ float(s) for s in row[2:5] ]) 
                quaternion = np.array([ float(s) for s in row[5:9] ])
                valid = bool(row[9])

                # TODO: compute velocity
                velocity = np.zeros(3)

                state = RobotState(
                    position = position,
                    velocity = velocity
                )

                state.timestamp = timestamp
                rotationMatrix = Rotation.from_quat(quaternion).as_matrix()
                state.SetRotationMatrix(rotationMatrix)

                self.groundTruthStates = np.append(self.groundTruthStates, state)

    def LoadSensorValues(self, sensorValueFileName):

        with open(sensorValueFileName, "r") as file:
            lines = file.readlines()

        serverMsgPrefix = self.serverMsgPrefix
        serverMsgPrefixLen = len(serverMsgPrefix)

        self.sensorValues = []

        rssi = None
        timestamp = None
        lastTimestamp = None
        angularVelocity = None
        linearAcceleration = None
        for lineIndex, line in enumerate(lines):

            lineNumber = lineIndex+1                    
            
            if len(line) < serverMsgPrefixLen or line[0:serverMsgPrefixLen] != serverMsgPrefix:
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
                            angularVelocity = angularVelocity,
                            rssi = rssi
                        )
                        
                        # Note: log timestamp is in microseconds 
                        sensorValue.timestamp = timestamp * 1e-6

                        self.sensorValues.append(sensorValue)
                        lastTimestamp = timestamp

                rssi = None       
                timestamp = None
                angularVelocity = None
                linearAcceleration = None

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

                elif valueType == "rssi":

                    if rssi is not None:
                        Warn(f"Multiple rssi values for sensorValue | original rssi {rssi} new rssi {rssi} | {sensorValueFileName}:{lineNumber}")

                    try:
                        rssi = int(value)
                    except Exception:
                        Warn(f"Failed to parse timestamp '{value}' | {sensorValueFileName}:{lineNumber}")
                        continue

    def LoadSamples(self):

        self.LoadSensorValues(self.sensorValueFileName)
        self.LoadMocapSamples(self.mocapFileName)
        
    def GetSample(self):

        # Get Sensor Sample
        if self.sensorIndex >= len(self.sensorValues):
            return False

        sensorValue = self.sensorValues[self.sensorIndex]
        if self.sensorIndex == 0:
            deltaT = 0

        else:
            prevSensorValue = self.sensorValues[self.sensorIndex-1]
            deltaT = sensorValue.timestamp - prevSensorValue.timestamp

        self.sensorIndex+= 1
        self.sensorTime+= deltaT

        if self.groundTruthIndex >= len(self.groundTruthStates):
            return False
        
        groundTruth1 = self.groundTruthStates[self.groundTruthIndex]
        groundTruth2 = groundTruth1
        
        # Find first groundTruth pair that spans sensorValue
        self.groundTruthIndex+= 1
        while self.groundTruthTime < self.sensorTime and \
              self.groundTruthIndex < len(self.groundTruthStates): 

            groundTruth1 = groundTruth2
            groundTruth2 = self.groundTruthStates[self.groundTruthIndex]

            self.groundTruthIndex+= 1
            self.groundTruthTime+= (groundTruth2.timestamp - groundTruth1.timestamp)
                    

        if groundTruth1.timestamp == groundTruth2.timestamp:
            groundTruthState = groundTruth1

        else:

            # interpolate groundTruthState
            t = np.clip(0, 1, (self.groundTruthTime - self.sensorTime) / (groundTruth2.timestamp - groundTruth1.timestamp))
            # groundTruthState = t * (groundTruth2 - groundTruth1) + groundTruth1
            groundTruthState = groundTruth1
            
        sample = DataSample(
            deltaT = deltaT,
            sensorValue = sensorValue,
            groundTruthState = groundTruthState,

            # TODO: Generate and parse these files!
            commandState = RobotState()
        )

        return sample
