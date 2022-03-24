from parse import parse

from utils.utils import *
from data.DataSample import DataSample

from system.RobotState import RobotState
from system.SensorValue import SensorValue

class LogSampler:
    
    def __init__(self, params):

        self.sampleIndex = 0
        self.params = params

        self.sensorValueFileName = GetParam(params, "dataSensorValueLog") 
        self.serverMsgPrefix = GetParam(params, "serverMsgPrefix", "<- ")

    def Reset(self):
        self.sampleIndex = 0

    def LoadSamples(self):

        sensorValueFileName = self.sensorValueFileName
        with open(sensorValueFileName, "r") as file:
            lines = file.readlines()

        serverMsgPrefix = self.serverMsgPrefix
        serverMsgPrefixLen = len(serverMsgPrefix)

        self.sensorValues = []

        lastTimestamp = None
        timestamp = None
        linearAcceleration = None
        angularVelocity = None
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

    def GetSample(self):

        if self.sampleIndex >= self.numSamples:
            return False

        sensorValue = self.sensorValues[self.sampleIndex]

        if self.sampleIndex == 0:
            deltaT = 0

        else:
            prevSensorValue = self.sensorValues[self.sampleIndex-1]
            deltaT = sensorValue.timestamp - prevSensorValue.timestamp

        sample = DataSample(
            deltaT = deltaT,
            sensorValue = sensorValue,

            # TODO: Generate and parse these files!
            groundTruthState = RobotState(),
            commandState = RobotState()
        )

        self.sampleIndex+= 1
        return sample
