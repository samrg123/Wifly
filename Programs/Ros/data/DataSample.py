from system.RobotState import RobotState
from system.SensorValue import SensorValue

class DataSample:
    def __init__(self, deltaT = 1, sensorValue = SensorValue(), commandState = RobotState(), groundTruthState = RobotState()):
        self.deltaT = deltaT
        self.sensorValue = sensorValue
        self.commandState = commandState
        self.groundTruthState = groundTruthState        