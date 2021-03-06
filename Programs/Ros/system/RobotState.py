import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp
import rospy
import yaml

from scipy.linalg import block_diag

from utils.utils import *


class RobotState:

    # Note: velocity is relative to sensorFrame. 
    #       Position and rotation is relative to world frame
    #       You can get the world frame velocity with state.GetRotationMatrix() @ state.GetVelocity()

    def __init__(self, 
                 position = np.zeros(3), orientation = np.zeros(3), velocity = np.zeros(3),
                 positionCovariance = np.zeros(3), orientationCovariance = np.zeros(3), velocityCovariance = np.zeros(3)):
        
        self._mean = np.diag(np.ones(5))
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

    def __mul__(self, scalar):
        r = RobotState(
            position = self._position * scalar,
            velocity = self._velocity * scalar,
        )
        r.SetCovariance(self._covariance * scalar)

        oldRotation = Rotation.from_matrix(self._rotationMatrix)
        newRotation = Rotation.from_rotvec(oldRotation.as_rotvec() * scalar)
        r.SetRotationMatrix(newRotation.as_matrix())

        return r

    def __rmul__(self, scalar):
        return self.__mul__(scalar)

    def __sub__(self, state):
        
        r = RobotState(
            position = self._position - state._position,
            velocity = self._velocity - state._velocity,
        )
        r.SetCovariance(self._covariance - state._covariance)
        r.SetRotationMatrix(state._rotationMatrix.T @ self._rotationMatrix)

        return r

    def __add__(self, state):
        
        r = RobotState(
            position = self._position + state._position,
            velocity = self._velocity + state._velocity,
        )
        r.SetCovariance(self._covariance + state._covariance)
        r.SetRotationMatrix(state._rotationMatrix @ self._rotationMatrix)

        return r


    # def __sub__(self, scaler):
    #     return RobotState.FromMean(self._mean * scaler)

    @staticmethod
    def Lerp(time, state1, state2):

        r = RobotState(
            position = time * (state2._position - state1._position) + state1._position,
            velocity = time * (state2._velocity - state1._velocity) + state1._velocity
        )

        r.SetCovariance( time * (state2._covariance - state1._covariance) + state1._covariance)        

        state1Rotation = Rotation.from_matrix(state1._rotationMatrix)
        state2Rotation = Rotation.from_matrix(state2._rotationMatrix)

        slepKeyRotations = Rotation.concatenate([state1Rotation, state2Rotation])

        slerp = Slerp([0, 1], slepKeyRotations)

        newRotation = slerp([time])  
        newRotationMatrix = newRotation.as_matrix()
    
        r.SetRotationMatrix(newRotationMatrix)
    
        return r


    def GetAdjoint(self):

        adj = block_diag(self._rotationMatrix, self._rotationMatrix, self._rotationMatrix)

        adj[3:6,0:3] = self.WedgeSO3(self._velocity) @ self._rotationMatrix
        adj[6:9,0:3] = self.WedgeSO3(self._position) @ self._rotationMatrix

        return adj

    @staticmethod
    def MeanState(states, weights = None):
        
        if len(states) == 0:
            return RobotState()

        rotationMatricies = [ state._rotationMatrix for state in states ]
        velocities        = [ state._velocity       for state in states ]
        positions         = [ state._position       for state in states ]
        covariances       = [ state._covariance     for state in states ]
        
        rotations       = Rotation.from_matrix(rotationMatricies)
        meanRotation    = rotations.mean(weights = weights)
        meanOrientation = meanRotation.as_euler("xyz")

        meanPosition = np.average(positions,  axis = 0, weights = weights)
        meanVelocity = np.average(velocities, axis = 0, weights = weights)

        meanState = RobotState(
            position    = meanPosition,
            velocity    = meanVelocity,
            orientation = meanOrientation
        )
        
        variancePosition    = np.average([ (state._position - meanPosition)**2           for state in states ], axis = 0, weights = weights)
        varianceVelocity    = np.average([ (state._velocity - meanVelocity)**2           for state in states ], axis = 0, weights = weights)
        varianceOrientation = np.average([ (state.GetOrientation() - meanOrientation)**2 for state in states ], axis = 0, weights = weights)

        meanState.SetPositionCovariance(np.diag(variancePosition))
        meanState.SetVelocityCovariance(np.diag(varianceVelocity))
        meanState.SetOrientationCovariance(np.diag(varianceOrientation))
        
        return meanState

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