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
        self.motionFunction = system.MotionFunction

        self.state = RobotState()

        self.Q = system.Q # Measuerment Covariance
        self.LQ = np.linalg.cholesky(self.Q)
        self.R = system.R # Wifi Noise
        self.noisy = system.noisy # Flag to determine if noise is added

        self.n = init.n
        w = 1/self.n
        # self.p = system.p
        self.p_w = w*np.zeros(init.n).reshape(init.n, 1)
        L = np.linalg.cholesky(init.Sigma)
        self.p = L@np.random.randn(size(init.Sigma)[0], init.n) + init.mu

        # init state
        self.state.SetMean(init.mu)
        self.state.SetCovariance(init.Sigma)

    
    def prediction(self, sensorValue, deltaT):
        state = self.state
        covariance = self.state.GetCovariance()

        # TODO: WE NEED TO SWITCH TO PROPER SE(3) OTHERWISE FORCE OF GRAVITY MESSES UP 
        #       ACCELERATION MODEL AND INTRODUCES DRIFT!

        # simply propagate the state and assign identity to covariance
        for i in self.p.size()[1]: 
            if self.noisy: 
                state = self.p[:, i] + self.LQ@np.random.randn(len(self.p[:, 0]), 1)
            else: 
                state = self.p[:, i]
            self.p[:, i] = self.motionFunction(state, sensorValue, deltaT)
        predictedCovariance = covariance

        self.state.SetMean(np.mean(self.p))
        # self.state.SetMean(predictedState.GetMean())
        self.state.SetCovariance(predictedCovariance)

    def intensity_query_client(pos): 
        rospy.wait_for_service("intensity")
        try: 
            intensity_serv = rospy.ServiceProxy("intensity", intensity)
            intensity_val = intensity_serv(pos)
            return intensity_val
        except rospy.ServiceException as e: 
            print(f"Exception {e} occurred")

    def correction(self, z):

        w = np.zeros((self.n, 1))
        
        for i in self.p.size()[1]: 
            v = z - intensity_query_client(self.p[:, i])
            w[i] = multivariate_normal.pdf(v.reshape(-1), np.zeros(len(self.p.size()[0])), self.R)

        self.p_w = np.multiply(self.p_w, w)
        self.p_w = self.p_w/np.sum(self.p_w)
        self.Neff = 1/np.sum(np.power(self.p_w), 2)

    def resampling(self): 
        W = np.cumsum(self.p_w)
        r = rand(1) / self.n
        j = 1
        for i in range(self.n): 
            u = r + (i-1)/self.n
            while u > W[j]: 
                j = j + 1
            self.p[:, i] = self.p[:, j]
            self.p_w[i] = 1 / self.n


    def GetState(self):
        return deepcopy(self.state)

    def SetState(self, state):
        self.state = deepcopy(state)