import sys, os

sys.path.append(os.path.abspath(os.path.join(os.path.join(os.path.abspath(__file__), '..', '..', '..', '..', 'wifly2'))))
from scripts.intensity_client_test import gen_pt, intensity_query_client

from sklearn.neighbors import KDTree
from system.RobotState import RobotState

from data.DataSample import DataSample

import numpy as np

class WifiSample:

    def __init__(self, position = np.zeros(3), rssi = -100):
        
        self._position = np.asarray(position, dtype="float")
        
        self.x = self._position[0:1]
        self.y = self._position[1:2]
        self.z = self._position[2:3]
        
        self.intensity = rssi

    def __add__(self, wifiSample):
        return WifiSample(
            position = self._position + wifiSample._position,
            rssi     = self.intensity + wifiSample.intensity,
        )

    def __mull__(self, scalar):
        return WifiSample(
            position = self._position * scalar,
            rssi     = self.intensity * scalar,
        )

    def __truediv__(self, scalar):
        return WifiSample(
            position = self._position / scalar,
            rssi     = self.intensity / scalar,
        )   

    def GetRssi(self):
        return self.intensity

    def SetRssi(self, rssi):
        self.intensity = rssi

    def GetPosition(self):
        return np.copy(self._position)

    def SetPosition(self, position):
        np.copyto(self._position, position)

class WifiMap:

    def __init__(self, initialWifiSamples = None, kdAverageSamples = 16):

        # TODO: query this
        if initialWifiSamples is None:
            self._width = 18.4
            self._height = 13.75
            self._depth = 0

            self.QueryWifi = self.QueryWifiClient

        else:

            self.usingKdTree = True
            self.kdAverageSamples = kdAverageSamples

            self.wifiSampleMap = np.copy(initialWifiSamples)
            
            initialTreeValues = [ s.GetPosition() for s in initialWifiSamples ]
            self.wifiPositionTree = KDTree(initialTreeValues, leaf_size=4)

            self.QueryWifi = self.QueryWifiKdTree

    @staticmethod
    def FromDataSamples(dataSamples):

        wifiSamples = np.array([], dtype="object")
        for dataSample in dataSamples:
            
            wifiSample = WifiSample(
                position = dataSample.groundTruthState.GetPosition(),
                rssi     = dataSample.sensorValue.GetRssi()
            )

            wifiSamples = np.append(wifiSamples, wifiSample)

        return WifiMap(wifiSamples) 

    def GetDepth(self):
        return self._depth

    def GetWidth(self):
        return self._width

    def GetHeight(self):
        return self._height
    
    def QueryWifiKdTree(self, robotState):

        position = robotState.GetPosition()
        
        nearestNeighborDistances, \
        nearestNeighborIndices = self.wifiPositionTree.query(position.reshape(1, 3), k = self.kdAverageSamples)

        nearestNeighborDistances = nearestNeighborDistances.reshape(-1)
        nearestNeighborIndices = nearestNeighborIndices.reshape(-1)

        cumulativeWifiSample = WifiSample() 
        for i in range(len(nearestNeighborIndices)):
            
            distance = nearestNeighborDistances[i]
            sampleIndex = nearestNeighborIndices[i]

            wifiSample = self.wifiSampleMap[sampleIndex]

            wifiPower = 1 - np.clip(0, 1, wifiSample.intensity / -100)

            falloffPower = wifiPower/(distance*distance) if distance > 1 else wifiPower
            
            falloffRssi = -100 * (1 - falloffPower)

            cumulativeWifiSample.SetPosition(cumulativeWifiSample.GetPosition() + wifiSample.GetPosition())
            cumulativeWifiSample.intensity+= falloffRssi 
        

        avgWifiSample = cumulativeWifiSample / self.kdAverageSamples
        avgWifiSample.intensity = np.round(avgWifiSample.intensity)

        return avgWifiSample


    def QueryWifiClient(self, robotState):
        position = robotState.GetPosition()
        return intensity_query_client(gen_pt(position))
     
     