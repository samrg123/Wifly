from system.RobotState import RobotState
from system.SensorValue import SensorValue

class DataSample:

    class Type:
        Default = 0
        ComputeBias = 1
        ComputeInitialState = 2

    def __init__(self, deltaT = 1, sensorValue = SensorValue(), commandState = RobotState(), groundTruthState = RobotState()):

        self.deltaT = deltaT
        self.sensorValue = sensorValue
        self.commandState = commandState
        self.groundTruthState = groundTruthState        