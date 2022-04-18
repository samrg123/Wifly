import sys, os

sys.path.append(os.path.abspath(os.path.join(os.path.join(os.path.abspath(__file__), '..', '..', '..', '..', 'wifly2'))))
from scripts.intensity_client_test import gen_pt, intensity_query_client

from sklearn.neighbors import KDTree
from system.RobotState import RobotState

from data.DataSample import DataSample

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors
from matplotlib import cm
from mpl_toolkits.axes_grid1 import make_axes_locatable

class WifiSample:

    def __init__(self, position = np.zeros(3), rssi = 0):
        
        self._position = np.copy(position)
        
        self.x = self._position[0:1]
        self.y = self._position[1:2]
        self.z = self._position[2:3]
        
        self.intensity = rssi

    def __add__(self, wifiSample):
        return WifiSample(
            position = self._position + wifiSample._position,
            rssi     = self.intensity + wifiSample.intensity,
        )

    def __mul__(self, scalar):
        return WifiSample(
            position = self._position * scalar,
            rssi     = self.intensity * scalar,
        )

    __rmul__ = __mul__

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

    def __init__(self,
                 initialWifiSamples = None, 
                 kdMinAverageSamples = 64,
                 kdMinAverageRadius  = .00):

        # TODO: query this
        if initialWifiSamples is None:
            self._width = 18.4
            self._height = 13.75
            self._depth = 0

            self.QueryWifi = self.QueryWifiClient

        else:

            # Note: 1mw..
            # TODO: Compute/set this a better way 
            self.maxPower = .000003 #.1mw corresponds 
            self.routerPosition = np.zeros(3)

            self.usingKdTree = True
            self.kdMinAverageSamples = kdMinAverageSamples
            self.kdMinAverageRadius = kdMinAverageRadius

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
        queryPosition = position.reshape(1, 3)

        # # Sequentialy double radius until we meet required number of points
        # radius = self.kdMinAverageRadius
        # while True:
                
        #     nearestNeighborIndices, \
        #     nearestNeighborDistances = self.wifiPositionTree.query_radius(
        #         queryPosition, 
        #         r = radius,
        #         return_distance = True
        #     )

        #     nearestNeighborIndices = nearestNeighborIndices[0]
        #     nearestNeighborDistances = nearestNeighborDistances[0]

        #     if len(nearestNeighborIndices) >= self.kdMinAverageSamples:
        #         break
            
        #     radius*= 2

        # Try to query radius
        nearestNeighborIndices, \
        nearestNeighborDistances = self.wifiPositionTree.query_radius(
            queryPosition, 
            r = self.kdMinAverageRadius,
            return_distance = True
        )

        nearestNeighborIndices = nearestNeighborIndices[0]
        nearestNeighborDistances = nearestNeighborDistances[0]

        # Just query nearest neighbors
        if len(nearestNeighborIndices) < self.kdMinAverageSamples:
            nearestNeighborDistances, \
            nearestNeighborIndices = self.wifiPositionTree.query(queryPosition, k = self.kdMinAverageSamples)

            nearestNeighborDistances = nearestNeighborDistances.reshape(-1)
            nearestNeighborIndices = nearestNeighborIndices.reshape(-1)

        numNeighbors = len(nearestNeighborIndices)
        wifiSamples = np.empty(numNeighbors, dtype="object")

        epsilon = np.finfo(float).eps

        delta1 = robotState.GetPosition() - self.routerPosition
        d1 = max(epsilon, np.linalg.norm(delta1))
        d1Squared = max(epsilon, np.dot(delta1, delta1))
        for i in range(numNeighbors):
            
            sampleIndex = nearestNeighborIndices[i]
            wifiSample = self.wifiSampleMap[sampleIndex]

            # # NOTE: using https://physics.stackexchange.com/questions/24253/geometry-of-wireless-signal-strength#answer-24258          
            # B = -2
            # delta2 = wifiSample.GetPosition() - self.routerPosition 
            # d2 = max(epsilon, np.linalg.norm(delta2))
            # falloffRssi = wifiSample.GetRssi() - B * np.log(d2 / max(epsilon, d1))

            # Note: This treats power ~ intensity
            # Note: intensity1/intensity2 = (d2/d1)^2 -> intensity1 = intensity2 * (d2^2)/(d1^2)
            dbm = wifiSample.GetRssi()
            watts = np.power(10, (dbm-30)/10)
            delta2 = wifiSample.GetPosition() - self.routerPosition
            d2Squared = max(epsilon, np.dot(delta2, delta2))
            falloffPower = np.clip(0, self.maxPower, watts * d2Squared / d1Squared)  
            falloffRssi = 10 * np.log10(falloffPower) + 30

            # # Note: This averages nearest neighbors
            # falloffRssi = wifiSample.intensity

            # write generated sample 
            wifiSamples[i] = WifiSample(
                position = wifiSample.GetPosition(),
                rssi = falloffRssi   
            )

        # Note: we weight generated samples by their distance to query points
        weights = nearestNeighborDistances / np.linalg.norm(nearestNeighborDistances)
        weights = 1 - weights if numNeighbors > 1 else [1]

        avgWifiSample = np.average(wifiSamples, weights = weights)

        return avgWifiSample

    def QueryWifiClient(self, robotState):
        position = robotState.GetPosition()
        
        result = intensity_query_client(gen_pt(position))

        return WifiSample(
            position = [result.x, result.y, position[2]],
            rssi = result.intensity
        )

    def SaveToPng(self, 
                  path, 
                  resolution = [800, 600], 
                  pixelSize = .01, 
                  origin = [0, 0, 0],
                  plotSamples = True):
        
        halfResolution = .5 * np.asarray(resolution)
        
        origin = np.asarray(origin)
        xyOrigin = origin[0:2]

        xMin = -int(halfResolution[0])
        xMax =  int(halfResolution[0] + .5)
        
        yMin = -int(halfResolution[1])
        yMax =  int(halfResolution[1] + .5)

        xRange = range(xMin, xMax)
        yRange = range(yMin, yMax)

        wifiSamples = np.full((resolution), WifiSample(), dtype=object) 

        positionZ = origin[2]
        for iy, y in enumerate(yRange):
            positionY = origin[1] + y * pixelSize
            
            for ix, x in enumerate(xRange):
                positionX = origin[0] + x * pixelSize

                position = [positionX, positionY, positionZ]
                wifiSample = self.QueryWifi(RobotState(position = position))

                wifiSamples[iy, ix] = wifiSample
     
        wifiRssi      = np.array([sample.GetRssi()     for sample in wifiSamples.reshape(-1)]).reshape(resolution)
        wifiPositions = np.array([sample.GetPosition() for sample in wifiSamples.reshape(-1)]).reshape(-1, 3)

        figSize = 4 * (resolution / np.max(resolution))
        fig = plt.figure()

        ax = plt.axes(projection='3d')
        ax.view_init(elev=90., azim=0)
        ax.dist = 7.5

        # NOTE: No idea why, but voxel requires filled voxels to be smaller than vx, vy, vz so we grow ranges by 1
        vx, vy, vz = np.meshgrid(
            [x for x in range(xMin, xMax + 1)], 
            [y for y in range(yMin, yMax + 1)], 
            [0, 1]
        )
        vx = vx * pixelSize + origin[0]
        vy = vy * pixelSize + origin[1]
        vz = vz*pixelSize + origin[2]

        minMetersX = min(xMin * pixelSize + origin[0], np.min(wifiPositions[:, 0]))
        maxMetersX = max(xMax * pixelSize + origin[0], np.max(wifiPositions[:, 0]))
        minMetersY = min(yMin * pixelSize + origin[0], np.min(wifiPositions[:, 1]))
        maxMetersY = max(yMax * pixelSize + origin[0], np.max(wifiPositions[:, 1]))
        minMetersZ = min(0 * pixelSize + origin[0],    np.min(wifiPositions[:, 2]))
        maxMetersZ = max(1 * pixelSize + origin[0],    np.max(wifiPositions[:, 2]))

        ax.set_box_aspect((*resolution, 1))
        ax.set_xlim3d(minMetersX, maxMetersX)
        ax.set_ylim3d(minMetersY, maxMetersY)
        ax.set_zlim3d(minMetersZ, maxMetersZ)
        
        # Hide z-tick can't read with top-down view 
        ax.set_zticks([])

        # place y-ticks on left side (colorbar goes on right side) 
        ax.xaxis._axinfo['juggled'] = (0,2,1)
        
        filledVoxels = np.full((*resolution, 1), True, dtype=bool)

        print(f"min: {wifiRssi.min()} | max: {wifiRssi.max()}")

        rssiColorNorm = matplotlib.colors.Normalize(vmin=wifiRssi.min(), vmax=wifiRssi.max())
        rssiColors = plt.cm.magma(rssiColorNorm(wifiRssi))
        voxelColors = rssiColors.reshape(*resolution, 1, 4)

        # # Set the alpha to .5
        # voxelColors[:, :, :, 3] = .5

        # Plot colorbar
        ax.voxels(vx, vy, vz, filled=filledVoxels, facecolors=voxelColors, shade=False)
        m = cm.ScalarMappable(cmap=plt.cm.magma, norm=rssiColorNorm)
        m.set_array([])
        plt.colorbar(m)

        # plt.imshow(wifiRssi, origin='lower', cmap="magma")
        # plt.colorbar()

        if(plotSamples):
            ax.scatter(wifiPositions[:, 0], wifiPositions[:, 1], wifiPositions[:, 2], marker='.', s=1, c='green')

        # # Get rid of the panes
        # ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        # ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        # ax.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))

        # # Get rid of the spines
        # ax.xaxis.line.set_color((1.0, 1.0, 1.0, 0.0))
        # ax.yaxis.line.set_color((1.0, 1.0, 1.0, 0.0))
        # ax.zaxis.line.set_color((1.0, 1.0, 1.0, 0.0))

        # Save png 
        plt.savefig(path, format="png", dpi=600, bbox_inches="tight")
        plt.show()