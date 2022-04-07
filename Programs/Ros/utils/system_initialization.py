
import numpy as np
from scipy.linalg import expm, block_diag

from utils.utils import *
from system.RobotState import RobotState

class system_initialization:

    def __init__(self, params):

        self.gyroBias          = GetParam(params, "gyroBias",     np.zeros(3))
        self.velocityBias      = GetParam(params, "velocityBias", np.zeros(3))
        self.accelerometerBias = GetParam(params, "acclerometerBias", np.array([0, 0, gForceOfGravity]))

        self.Q = 0.1*np.eye(3)
        self.R = 3
        self.noisy = False

    def MotionFunction(self, state, sensorValue, deltaT):

        position           = state.GetPosition()
        linearVelocity     = state.GetVelocity()
        rotationMatrix     = state.GetRotationMatrix()
        angularVelocity    = sensorValue.GetAngularVelocity()
        linearAcceleration = sensorValue.GetLinearAcceleration()

        # Mani Gamma functions - TODO: Figure out why these are different than expm. Are we interpretting Mani's notes wrong?
        # Gamma1 = system_initialization.Gamma1
        # Gamma2 = system_initialization.Gamma2
        # # deltaVelocity  = rotationMatrix @ Gamma1(biasCorrectedAngularVelocity * deltaT) @ biasCorrectedLinearAcceleration * deltaT
        # # deltaPosition  = linearVelocity * deltaT + rotationMatrix @ Gamma2(biasCorrectedAngularVelocity * deltaT) @ biasCorrectedLinearAcceleration * deltaT*deltaT
        
        # # EXPM integration of velocity and position by hand
        # # See: https://arxiv.org/pdf/2102.12897.pdf for reference
        # biasCorrectedAngularVelocity = angularVelocity - rotationMatrix @ self.gyroBias
        # biasCorrectedLinearAcceleration = linearAcceleration - rotationMatrix @ self.accelerometerBias
        # deltaVelocity  = rotationMatrix @ (np.linalg.norm(biasCorrectedAngularVelocity) * biasCorrectedLinearAcceleration * deltaT)
        # deltaPosition  = rotationMatrix @ (np.linalg.norm(biasCorrectedAngularVelocity) * linearVelocity * deltaT * deltaT)
        # r = RobotState(
        #     position = position + deltaPosition,
        #     velocity = linearVelocity + deltaVelocity
        # )
        # r.SetRotationMatrix(rotationMatrix @ expm( RobotState.WedgeSO3(biasCorrectedAngularVelocity * deltaT) ) )

        # EXPM integration - Most accurate for real world data 
        r = RobotState()
        derivative = np.hstack((
            angularVelocity     - rotationMatrix @ self.gyroBias,
            linearAcceleration  - rotationMatrix @ self.accelerometerBias,
            linearVelocity      - rotationMatrix @ self.velocityBias 
        ))
        r.SetMean(state.GetMean() @ expm( RobotState.Wedge(derivative * deltaT) ))

        # # Step-wise Integration . Note: position update lags behind yielding poor results with real-world data
        # linearVelocity = state.GetVelocity()
        # r = RobotState(
        #     velocity    = linearVelocity + (rotationMatrix.T @ linearAcceleration - self.accelerometerBias) * deltaT,
        #     position    = position       + (linearVelocity - self.velocityBias) * deltaT
        # )
        # r.SetRotationMatrix(rotationMatrix @ expm( RobotState.WedgeSO3( (angularVelocity - rotationMatrix @ self.gyroBias) * deltaT) ))

        return r

    @staticmethod
    def Gamma1(omega):
        omegaNorm = np.linalg.norm(omega)

        if omegaNorm == 0.:
            return np.eye(3)

        omegaNormSquared = omegaNorm*omegaNorm
        omegaWedge = RobotState.WedgeSO3(omega)

        return np.eye(3) + \
            ((1 - np.cos(omegaNorm))/omegaNormSquared) * omegaWedge + \
            ((omegaNorm - np.sin(omegaNorm))/(omegaNormSquared * omegaNorm)) * omegaWedge @ omegaWedge

    @staticmethod
    def Gamma2(omega):

        halfI = .5*np.eye(3)

        omegaNorm = np.linalg.norm(omega)
        if omegaNorm == 0.:
            return halfI

        omegaNormSquared = omegaNorm*omegaNorm
        omegaWedge = RobotState.WedgeSO3(omega)

        return halfI + \
            ((omegaNorm - np.sin(omegaNorm))/(omegaNormSquared * omegaNorm)) * omegaWedge + \
            ((omegaNormSquared + 2*np.cos(omegaNorm) - 2)/(2*omegaNormSquared * omegaNormSquared)) * omegaWedge @ omegaWedge        

