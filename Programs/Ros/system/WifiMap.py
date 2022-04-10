import sys, os

sys.path.append(os.path.abspath(os.path.join(os.path.join(os.path.abspath(__file__), '..', '..', '..', '..', 'wifly2'))))
from scripts.intensity_client_test import gen_pt, intensity_query_client

from system.RobotState import RobotState

class WifiMap:

    def __init__(self):
        pass

    def QueryWifi(self, robotState):
        
        # TODO: placeholder until server is built
        class MockValue:
            def __init__(self):
                self.x = -1
                self.y = -1
                self.intensity = -100

        return MockValue()

        position = robotState.GetPosition()
        return intensity_query_client(gen_pt(position))
     
     