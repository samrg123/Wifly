import numpy as np
from scipy.spatial.transform import Rotation
import rospy
import yaml
from utils.utils import *


class RobotState:

    # Note: velocity is relative to sensorFrame. 
    #       Position and rotation is relative to world frame
    #       You can get the world frame velocity with state.GetRotationMatrix() @ state.GetVelocity()

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

    def __mul__(self, scaler):
        return RobotState.FromMean(self._mean * scaler)

    def __rmul__(self, scaler):
        return self.__mul__(scaler)

    @staticmethod
    def MeanState(states, weights = None):
        
        if len(states) == 0:
            return RobotState()

        rotationMatricies = [ state._rotationMatrix for state in states ]
        velocities        = [ state._velocity       for state in states ]
        positions         = [ state._position       for state in states ]
        covariances       = [ state._covariance     for state in states ]
        
        rotations = Rotation.from_matrix(rotationMatricies)

        meanRotation = rotations.mean(weights = weights)
        meanPosition = np.average(positions,  axis = 0, weights = weights)
        meanVelocity = np.average(velocities, axis = 0, weights = weights)

        r = RobotState(
            position    = meanPosition,
            velocity    = meanVelocity,
            orientation = meanRotation.as_euler("xyz")
        )

        #TODO: is this computed right?  
        meanCovariance = np.average(covariances, axis = 0, weights = weights)
        r.SetCovariance(meanCovariance)

        return r

    def SampleNeighborhood(self):

        gaussianMean = np.hstack((
            self.GetOrientation(),
            self._velocity,
            self._position,
        ))
        
        sample = gRng.multivariate_normal(gaussianMean, self._covariance)

        r = RobotState(
            orientation = sample[0:3],
            velocity    = sample[3:6],
            position    = sample[6:9],
        )
        r.SetCovariance(self._covariance)

        return r

        particleState = RobotState.FromMean(
                
            )

        # return 


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
    def WedgeSO3(dTheta):
        return np.array([
            [ 0,         -dTheta[2],  dTheta[1]],
            [ dTheta[2],  0,         -dTheta[0]],
            [-dTheta[1],  dTheta[0],   0       ] 
        ])

    @staticmethod
    def VeeSO3(so3):
        return np.array([ so3[2, 1], so3[0, 2], so3[1, 0] ])

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

    @staticmethod
    def Vee(se2_3):
        dTheta    = [se2_3[2,1], se2_3[0,2], se2_3[1,0]]
        dVelocity = se2_3[0:3,3]
        dPosition = se2_3[0:3,4]

        return np.hstack((dTheta, dVelocity, dPosition))

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