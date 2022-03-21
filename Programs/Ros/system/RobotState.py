import numpy as np
import rospy
import yaml
from utils.utils import *

class RobotState:
    '''
    Robot State:
    2D: X = [x, y, theta, vx, vy]; position = [x, y]; orientation = [theta]; velocity = [vx, vy]
    '''
    def __init__(self, 
                 position = np.zeros(2), orientation = np.zeros(1), velocity = np.zeros(2) ):
        
        self._mean = np.zeros(5)
        self._position      = self._mean[0:2]
        self._orientation   = self._mean[2:3]
        self._velocity      = self._mean[3:5]

        np.copyto(self._position, position)
        np.copyto(self._orientation, orientation)
        np.copyto(self._velocity, velocity)

        self._covariance = np.diag(np.ones(len(self._mean)))
        self._positionCovariance    = self._covariance[0:2, 0:2]
        self._orientationCovariance = self._covariance[2:3, 2:3]
        self._velocityCovariance    = self._covariance[3:5, 3:5]

    def __str__(self):
        return f"RobotState {{ position: {self._position}, orientation: {self._orientation}, velocity: {self._velocity} }}"

    def SetPosition(self, position):
        np.copyto(self._position, position)

    def GetPosition(self):
        return np.copy(self._position)

    def SetOrientation(self, orientation):
        np.copyto(self._orientation, orientation)

    def GetOrientation(self):
        return np.copy(self._orientation)

    def SetVelocity(self, velocity):
        np.copyto(self._velocity, velocity)

    def GetVelocity(self):
        return np.copy(self._velocity)

    def SetPositionCovariance(self, cov):
        np.copyto(self._positionCovariance, cov)

    def GetPositionCovariance(self):
        return np.copy(self._positionCovariance)

    def SetOrientationCovariance(self, cov):
        np.copyto(self._orientationCovariance, cov)

    def GetOrientationCovariance(self):
        return np.copy(self._orientationCovariance)

    def SetCovariance(self, cov):
        np.copyto(self._covariance, cov)

    def GetCovariance(self):
        return np.copy(self._covariance)

    def SetMean(self, mean):
        np.copyto(self._mean, mean)

    def GetMean(self):
        return np.copy(self._mean)

def main():
    rob_sys = RobotState()

if __name__ == '__main__':
    main()