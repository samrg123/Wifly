import sys, os

sys.path.append(os.path.abspath(os.path.join(os.path.join(os.path.abspath(__file__), '..', '..', '..', '..', 'wifly2'))))
from scripts.intensity_client_test import gen_pt, intensity_query_client

from system.RobotState import RobotState

class WifiMap:

    def __init__(self):

        # TODO: query this
        self._width = 18.4
        self._height = 13.75
        self._depth = 0

    def GetDepth(self):
        return self._depth

    def GetWidth(self):
        return self._width

    def GetHeight(self):
        return self._height

    class MockValue:
        def __init__(self):
            self.x = -1
            self.y = -1
            self.intensity = -100

    def QueryWifi(self, robotState):
        position = robotState.GetPosition()
        return intensity_query_client(gen_pt(position))
     
     