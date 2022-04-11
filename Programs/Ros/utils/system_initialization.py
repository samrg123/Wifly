
import numpy as np
from scipy.linalg import expm, block_diag

from utils.utils import *
from system.RobotState import RobotState
from system.SensorValue import SensorValue
from system.WifiMap import WifiMap

class system_initialization:

    def __init__(self, params):

        self.wifiMap = WifiMap()

        self.gyroBias          = GetParam(params, "gyroBias",     np.zeros(3))
        self.velocityBias      = GetParam(params, "velocityBias", np.zeros(3))
        self.accelerometerBias = GetParam(params, "acclerometerBias", np.array([0, 0, gForceOfGravity]))

        init_state_vals = np.array(params['initial_state_vals'])
        self.initialState = RobotState(
            orientation = init_state_vals[0:3],
            velocity    = init_state_vals[3:6],
            position    = init_state_vals[6:9]
        )

        init_state_cov = np.diag(params['initial_state_variance'])
        self.initialState.SetCovariance(init_state_cov)

        # TODO: Store position noise in robotMotionNoise model / yaml file?
        robotMotionNoise = GetParam(params, "robotMotionNoise", np.zeros((2,2,3)))
        self.motionNoiseDensity = np.hstack((
            robotMotionNoise[1][1], # orientation
            robotMotionNoise[0][1], # velocity
            np.zeros(3)             # position 
        ))


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
        r.SetCovariance(state.GetCovariance())

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

        r = RobotState.FromMean(state.GetMean() @ expm( RobotState.Wedge(derivative * deltaT) ))
        r.SetCovariance(state.GetCovariance())

        return r

    @staticmethod
    def GammaRotationMatrix(oldState, deltaOrientation):

        rotationMatrix = oldState.GetRotationMatrix()

        return rotationMatrix @ expm(RobotState.WedgeSO3(deltaOrientation))

    @staticmethod
    def GammaDeltaOrientation(lastState, currentState):
        
        lastRotationMatrix    = lastState.GetRotationMatrix()
        currentRotationMatrix = currentState.GetRotationMatrix()
        
        return RobotState.VeeSO3(logm(lastRotationMatrix.T @ currentRotationMatrix))

    @staticmethod
    def GammaLinearVelocity(oldState, deltaOrientation, deltaLinearVelocity):

        rotationMatrix = oldState.GetRotationMatrix()
        linearVelocity = oldState.GetVelocity()

        return linearVelocity + \
               rotationMatrix @ system_initialization.Gamma1(deltaOrientation) @ deltaLinearVelocity

    @staticmethod
    def GammaDeltaLinearVelocity(lastState, currentState, deltaOrientation):

        lastRotationMatrix = lastState.GetRotationMatrix()

        return np.linalg.inv(lastRotationMatrix @ system_initialization.Gamma1(deltaOrientation)) @ \
               (currentState.GetVelocity() - lastState.GetVelocity())

    def GammaPosition(self, oldState, deltaOrientation, deltaLinearVelocity, deltaT):
        
        position       = oldState.GetPosition()
        rotationMatrix = oldState.GetRotationMatrix()
        linearVelocity = oldState.GetVelocity()

        deltaPosition = deltaT * (linearVelocity - rotationMatrix @ self.velocityBias)

        return position + \
               rotationMatrix @ deltaPosition + \
               rotationMatrix @ system_initialization.Gamma2(deltaOrientation) @ deltaLinearVelocity * deltaT        

    def GammaSensorValue(self, lastState, currentState, deltaT):
  
        deltaOrientation    = system_initialization.GammaDeltaOrientation(lastState, currentState)
        deltaLinearVelocity = system_initialization.GammaDeltaLinearVelocity(lastState, currentState, deltaOrientation)

        inverseDeltaT = 1./deltaT
        lastRotationMatrix = lastState.GetRotationMatrix()

        return SensorValue(
            angularVelocity    = inverseDeltaT * deltaOrientation    + lastRotationMatrix @ self.gyroBias,
            linearAcceleration = inverseDeltaT * deltaLinearVelocity + lastRotationMatrix @ self.accelerometerBias,
            rssi               = self.wifiMap.QueryWifi(currentState).intensity
        )

    # Note: Slowest, but uses second order taylor series. Correct integration for both position and velocity
    def GammaMotionFunction(self, state, sensorValue, deltaT):

        rotationMatrix = state.GetRotationMatrix()

        angularVelocity    = sensorValue.GetAngularVelocity()
        linearAcceleration = sensorValue.GetLinearAcceleration()        

        deltaOrientation    = deltaT * (angularVelocity    - rotationMatrix @ self.gyroBias)
        deltaLinearVelocity = deltaT * (linearAcceleration - rotationMatrix @ self.accelerometerBias)
        
        r = RobotState(
            velocity = self.GammaLinearVelocity(state, deltaOrientation, deltaLinearVelocity),
            position = self.GammaPosition(state, deltaOrientation, deltaLinearVelocity, deltaT),
        )
        r.SetRotationMatrix(self.GammaRotationMatrix(state, deltaOrientation))
        
        r.SetCovariance(state.GetCovariance())
        
        # TODO: GET THIS TO WORK!
        # stateCovariance = state.GetCovariance()
        # stateAdjoint = state.GetAdjoint()
        # blockRotation = block_diag(rotationMatrix, rotationMatrix, rotationMatrix)
        # covariance = stateCovariance + stateAdjoint @ self.motionNoiseDensity @ stateAdjoint.T
        # covariance = stateCovariance + np.diag(self.motionNoiseDensity)
    
        # print("covariance")
        # print(covariance)
        # print("")
        
        # r.SetCovariance(covariance)

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

