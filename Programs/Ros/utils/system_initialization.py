
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

    # Note: Really fast, but position update lags behind yielding poor results with real-world data 
    def StepWiseMotionFunction(self, state, sensorValue, deltaT):

        position       = state.GetPosition()
        linearVelocity = state.GetVelocity()
        rotationMatrix = state.GetRotationMatrix()

        angularVelocity    = sensorValue.GetAngularVelocity()
        linearAcceleration = sensorValue.GetLinearAcceleration()

        deltaOrientation    = deltaT * (angularVelocity    - rotationMatrix @ self.gyroBias)
        deltaLinearVelocity = deltaT * (linearAcceleration - rotationMatrix @ self.accelerometerBias)
        deltaPosition       = deltaT * (linearVelocity     - rotationMatrix @ self.velocityBias) 

        r = RobotState(
            velocity = linearVelocity + rotationMatrix @ deltaLinearVelocity,
            position = position       + rotationMatrix @ deltaPosition
        )
        r.SetRotationMatrix(rotationMatrix @ expm( RobotState.WedgeSO3(deltaOrientation) ))

        return r

    # Note: Moderatly fast, uses first order taylor series. Correctly integarates velocity, but position integration slightly off    
    def ExpmMotionFunction(self, state, sensorValue, deltaT):

        linearVelocity = state.GetVelocity()
        rotationMatrix = state.GetRotationMatrix()

        angularVelocity    = sensorValue.GetAngularVelocity()
        linearAcceleration = sensorValue.GetLinearAcceleration()

        derivative = np.hstack((
            angularVelocity     - rotationMatrix @ self.gyroBias,
            linearAcceleration  - rotationMatrix @ self.accelerometerBias,
            linearVelocity      - rotationMatrix @ self.velocityBias
        ))

        return RobotState.FromMean(state.GetMean() @ expm( RobotState.Wedge(derivative * deltaT) ))

    # Note: Slowest, but uses second order taylor series. Correct integration for both position and velocity
    def GammaMotionFunction(self, state, sensorValue, deltaT):

        position       = state.GetPosition()
        linearVelocity = state.GetVelocity()
        rotationMatrix = state.GetRotationMatrix()

        angularVelocity    = sensorValue.GetAngularVelocity()
        linearAcceleration = sensorValue.GetLinearAcceleration()        

        deltaOrientation    = deltaT * (angularVelocity    - rotationMatrix @ self.gyroBias)
        deltaLinearVelocity = deltaT * (linearAcceleration - rotationMatrix @ self.accelerometerBias)
        deltaPosition       = deltaT * (linearVelocity     - rotationMatrix @ self.velocityBias)
        
        velocity = linearVelocity + \
                   rotationMatrix @ self.Gamma1(deltaOrientation) @ deltaLinearVelocity

        position = position + \
                   rotationMatrix @ deltaPosition + \
                   rotationMatrix @ self.Gamma2(deltaOrientation) @ deltaLinearVelocity*deltaT

        r = RobotState(
            position = position,
            velocity = velocity
        )
        r.SetRotationMatrix(rotationMatrix @ expm( RobotState.WedgeSO3(deltaOrientation) ))

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

