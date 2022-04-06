import numpy as np
from scipy.spatial.transform import Rotation
import rospy
import yaml
from utils.utils import *


class RobotState:
    '''
    Robot State:
    2D: X = [x, y, theta, vx, vy]; position = [x, y]; orientation = [theta]; velocity = [vx, vy]
    '''
    def __init__(self, 
                 position = np.zeros(3), orientation = np.zeros(3), velocity = np.zeros(3),
                 positionCovariance = np.zeros(3), orientationCovariance = np.zeros(3), velocityCovariance = np.zeros(3)):
        
        self._mean = np.diag(np.ones((5)))
        self._rotationMatrix = self._mean[0:3, 0:3] 
        self._velocity       = self._mean[0:3, 3]
        self._position       = self._mean[0:3, 4]

        np.copyto(self._position, position)
        np.copyto(self._velocity, velocity)
        self.SetOrientation(orientation)

        self._covariance = np.diag(np.ones(9))
        self._orientationCovariance = self._covariance[0:3, 0:3]
        self._velocityCovariance    = self._covariance[3:6, 3:6]
        self._positionCovariance    = self._covariance[6:9, 6:9]

        np.copyto(self._orientationCovariance, orientationCovariance)
        np.copyto(self._velocityCovariance, velocityCovariance)
        np.copyto(self._positionCovariance, positionCovariance)

    @staticmethod
    def Copy(state):
        r = RobotState()
        np.copyto(r._mean, state._mean)
        np.copyto(r._covariance, state._covariance)
        return r

    @staticmethod
    def FromMean(mean):
        r = RobotState()
        np.copyto(r._mean, mean)
        return r

    @staticmethod
    def WedgeSO3(theta):
        return np.array([
            [ 0,         -theta[2],  theta[1]],
            [ theta[2],  0,         -theta[0]],
            [-theta[1],  theta[0],   0       ] 
        ])

    @staticmethod
    def Wedge(derivative):

        dTheta    = derivative[0:3].reshape(-1)
        dVelocity = derivative[3:6].reshape(-1)
        dPosition = derivative[6:9].reshape(-1)

        return np.array([
            [ 0,         -dTheta[2],  dTheta[1], dVelocity[0], dPosition[0]],
            [ dTheta[2],  0,         -dTheta[0], dVelocity[1], dPosition[1]],
            [-dTheta[1],  dTheta[0],  0,         dVelocity[2], dPosition[2]],
            [ 0,          0,          0,         0,            0           ],
            [ 0,          0,          0,         0,            0           ]
        ])    

    def __str__(self):
        return f"RobotState {{ position: {self._position}, orientation: {self.GetOrientation()}, velocity: {self._velocity} }}"

    def SetPosition(self, position):
        np.copyto(self._position, position)

    def GetPosition(self):
        return np.copy(self._position)

    def SetRotationMatrix(self, rotationMatrix):
        np.copyto(self._rotationMatrix, rotationMatrix)

    def GetRotationMatrix(self):
        return np.copy(self._rotationMatrix)

    def SetOrientation(self, orientation):
        np.copyto(self._rotationMatrix, Rotation.from_euler("xyz", orientation).as_matrix())

    def GetOrientation(self):
        return np.copy(Rotation.from_matrix(self._rotationMatrix).as_euler("xyz"))

    def SetVelocity(self, velocity):
        np.copyto(self._velocity, velocity)

    def GetVelocity(self):
        return np.copy(self._velocity)

    def SetPositionCovariance(self, cov):
        np.copyto(self._positionCovariance, cov)

    def GetPositionCovariance(self):
        return np.copy(self._positionCovariance)

    def SetVelocityCovariance(self, cov):
        np.copyto(self._velocityCovariance, cov)

    def GetVelocityCovariance(self):
        return np.copy(self._velocityCovariance)

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