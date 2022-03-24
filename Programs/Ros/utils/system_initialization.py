
import numpy as np
from scipy.linalg import expm, block_diag

from utils.utils import *
from system.RobotState import RobotState

class system_initialization:

    def __init__(self, params):

        self.gyroBias          = GetParam(params, "gyroBias",     np.zeros(3))
        self.velocityBias      = GetParam(params, "velocityBias", np.zeros(3))
        self.accelerometerBias = GetParam(params, "acclerometerBias", np.array([0, 0, gForceOfGravity]))

    def MotionFunction(self, state, sensorValue, deltaT):

        # # Mani Gamma functions
        # rotationMatrix     = state.GetRotationMatrix()
        # angularVelocity    = sensorValue.GetAngularVelocity()
        # linearAcceleration = sensorValue.GetLinearAcceleration()

        # Gamma1 = system_initialization.Gamma1
        # Gamma2 = system_initialization.Gamma2
        # deltaVelocity  = (rotationMatrix @ Gamma1(angularVelocity * deltaT) @ linearAcceleration - gAccelerometerBias.T) * deltaT
        # deltaPosition  = state.GetVelocity() * deltaT + rotationMatrix @ Gamma2(angularVelocity * deltaT) @ (linearAcceleration  - .5*gGravity.reshape(-1)) * deltaT*deltaT
        # r = RobotState(
        #     velocity = state.GetVelocity() + deltaVelocity,
        #     position = state.GetPosition() + deltaPosition
        # )
        # r.SetRotationMatrix(rotationMatrix @ expm( RobotState.WedgeSO3(angularVelocity * deltaT) ) )

        # Normal Integration
        rotationMatrix     = state.GetRotationMatrix()
        angularVelocity    = sensorValue.GetAngularVelocity()
        linearAcceleration = sensorValue.GetLinearAcceleration()
        
        r = RobotState(
            velocity    = state.GetVelocity() + (rotationMatrix.T @ linearAcceleration - self.accelerometerBias) * deltaT,
            position    = state.GetPosition() + (state.GetVelocity() - self.velocityBias) * deltaT
        )
        r.SetRotationMatrix(rotationMatrix @ expm( RobotState.WedgeSO3( (angularVelocity - self.gyroBias) * deltaT) ))
        
        return r

    @staticmethod
    def Gamma1(omega):
        omegaNorm = np.linalg.norm(omega)

        if omegaNorm == 0.:
            return np.eye(3)

        omegaNormSquared = omegaNorm*omegaNorm
        omegaWedge = RobotState.WedgeSO3(omega)

        return np.eye(3) + \
            (1 - np.cos(omegaNorm))/omegaNormSquared * omegaWedge + \
            (omegaNorm - np.sin(omegaNorm))/(omegaNormSquared * omegaNorm) * omegaWedge @ omegaWedge

    @staticmethod
    def Gamma2(omega):

        halfI = .5*np.eye(3)

        omegaNorm = np.linalg.norm(omega)
        if omegaNorm == 0.:
            return halfI

        omegaNormSquared = omegaNorm*omegaNorm
        omegaWedge = RobotState.WedgeSO3(omega)

        return halfI + \
            (omegaNorm - np.sin(omegaNorm))/(omegaNormSquared * omegaNorm) * omegaWedge + \
            (omegaNormSquared + 2*np.cos(omegaNorm) - 2)/(2*omegaNormSquared * omegaNormSquared) * omegaWedge @ omegaWedge        

