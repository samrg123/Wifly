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
        self.state.setMeanVector(init.mu)
        self.state.setCovariance(init.Sigma)

    
    def prediction(self, sensorValue):
        state = self.state
        P = self.state.getCovariance()

        # simply propagate the state and assign identity to covariance
        predictedState = self.motionFunction(state, sensorValue)
        P_pred = np.eye(3)

        self.state.setTime(rospy.Time.now())
        self.state.setMean(predictedState)
        self.state.setCovariance(P_pred)


    def correction(self, z):
        assert False, "Not Implemented"

    def getState(self):
        return deepcopy(self.state)

    def setState(self, state):
        self.state = deepcopy(state)