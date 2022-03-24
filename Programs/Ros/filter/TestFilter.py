import numpy as np
from scipy.linalg import block_diag
from copy import deepcopy, copy
import rospy

from system.RobotState import RobotState
from utils.utils import *

class TestFilter:

    def __init__(self, system, init):

        #   system: system and noise models
        #   init:   initial state mean and covariance
        self.motionFunction = system.motionFunction
        self.motionNoiseMatrix = system.motionNoiseMatrix

        self.state = RobotState()

        # init state
        self.state.SetMean(init.mu)
        self.state.SetCovariance(init.Sigma)

    
    def prediction(self, sensorValue, deltaT):
        state = self.state
        covariance = self.state.GetCovariance()

        # TODO: WE NEED TO SWITCH TO PROPER SE(3) OTHERWISE FORCE OF GRAVITY MESSES UP 
        #       ACCELERATION MODEL AND INTRODUCES DRIFT!

        # simply propagate the state and assign identity to covariance
        predictedState = self.motionFunction(state, sensorValue, deltaT)
        predictedCovariance = covariance

        self.state.SetMean(predictedState.GetMean())
        self.state.SetCovariance(predictedCovariance)


    def correction(self, z):
        assert False, "Not Implemented"

    def GetState(self):
        return deepcopy(self.state)

    def SetState(self, state):
        self.state = deepcopy(state)