import numpy as np
from scipy.linalg import block_diag
from copy import deepcopy, copy
import rospy

from scipy.stats import multivariate_normal
from scipy.spatial.transform import Rotation

from system.RobotState import RobotState
from utils.utils import *
from utils.noiseUtils import *

class TestFilter:

    #   system: system and noise models
    #   init:   initial state mean and covariance
    def __init__(self, system, params):

        # self.motionFunction = system.StepWiseMotionFunction
        # self.motionFunction = system.ExpmMotionFunction
        self.motionFunction = system.GammaMotionFunction
        
        self.wifiMap = system.wifiMap

        initialState = system.initialState
        self.state = RobotState.Copy(initialState)

        initialMean       = initialState.GetMean()
        initialCovariance = initialState.GetCovariance()

        #TODO: What is this? store params in settings.yaml 
        self.R = GetParam(params, "wifiErrorCovariance", 5) # Wifi Noise
        self.noisy = False # Flag to determine if noise is added

        self.n = GetParam(params, "numParticles", 100)

        w = 1/self.n if self.n > 0 else 0
        self.p_w = w*np.ones(self.n).reshape(self.n, 1)

        self.particleSensorNoise = SensorNoise(
            accelerometerNoise = NormalNoise(mean = np.zeros(3), covariance = [1, 1, 0] * np.full(3, 7.61543504e-7)),
            gyroNoise          = NormalNoise(mean = np.zeros(3), covariance = [1, 1, 0] * np.full(3, 2.0397832)),
            rssiNoise          = NormalNoise(mean = [0], covariance = [10])
        ) 

        # Note: covariance can be zero if we know our initial position
        #       Instead we scatter particles uniformly around the map
        # L = np.linalg.cholesky(initialCovariance)[0:3, 0:3]
        L = np.array([
            self.wifiMap.GetWidth(),
            self.wifiMap.GetHeight(),
            self.wifiMap.GetDepth() 
        ])

        self.p = []
        for i in range(self.n): 
            
            particleState = RobotState(
                position = L * np.random.uniform(size = 3),
                velocity = initialState.GetVelocity(),
                orientation = initialState.GetOrientation()
            )

            particleState.SetCovariance(initialCovariance)

            self.p.append(particleState)

    def GetParticleStates(self):
        return self.p


    def prediction(self, sensorValue, deltaT):
        state = np.copy(self.state)
        covariance = self.state.GetCovariance()

        # simply propagate the state and assign identity to covariance
        for i in range(len(self.p)): 

            state = self.p[i]

            sensorNoise = self.particleSensorNoise.Sample()
            self.p[i] = self.motionFunction(state, sensorValue + sensorNoise, deltaT)


        predictedCovariance = covariance

        # TODO: Update the mean values
        # self.state.SetMean(np.mean(self.p))
        # self.state.SetMean(predictedState.GetMean())
        # self.state.SetCovariance(predictedCovariance)

    # def intensity_query_client(pos): 
    #     rospy.wait_for_service("intensity")
    #     try: 
    #         intensity_serv = rospy.ServiceProxy("intensity", intensity)
    #         intensity_val = intensity_serv(pos)
    #         print(intensity_val)
    #         return intensity_val
    #     except rospy.ServiceException as e: 
    #         print(f"Exception {e} occurred")


    # def gen_pt(pos):
    #     pt = Point()
    #     pt.x = pos[0]
    #     pt.y = pos[1]
    #     return pt

    def correction(self, sensorValue, deltaT):

        # # NOTE: THIS IS PLACEHOLDER FOR DEBUGGING
        # self.state = self.motionFunction(self.state, sensorValue, deltaT)
        # return

        wifiMap = self.wifiMap
    
        rssi = sensorValue.GetRssi()
        
        w = np.zeros((self.n, 1))
        for i in range(len(self.p)):
            
            particleState = self.p[i]
             
            wifi = wifiMap.QueryWifi(particleState)
        
            if (wifi.x == -1 and wifi.y == -1) or wifi.occupied: 
                w[i] = 0
                continue
            
            v = rssi - wifi.intensity
            
            w[i] = multivariate_normal.pdf(v, 0, self.R)

        if np.sum(w) != 0: 
            self.p_w = np.multiply(self.p_w, w)
            self.p_w = self.p_w/np.sum(self.p_w)
        self.Neff = 1/np.sum(np.power(self.p_w, 2))

        if self.Neff < self.n/5: 
            self.resampling()

        self.mean_variance()

    def resampling(self): 
        W = np.cumsum(self.p_w)
        r = np.random.rand(1) / self.n
        j = 1
        for i in range(self.n): 
            u = r + (i-1)/self.n
            while u > W[j]: 
                j = j + 1

            # NOTE: jitter added to prevent particles from collapsing on each other  
            # self.p[i] = self.p[j]
            self.p[i] = RobotState(
                position = self.p[j].GetPosition() + [.05, .05, 0] * np.random.randn(3), 
                orientation = self.p[j].GetOrientation(), 
                velocity = self.p[j].GetVelocity()
            )

            self.p_w[i] = 1 / self.n

    def mean_variance(self): 

        means = np.zeros((6, self.n))
        quats = np.zeros((4, self.n))
        mean = np.zeros((6))
        quat = np.array([0., 0., 0., 1.]) 

        for s in range(len(self.p)): 
            pv = np.hstack((self.p[s].GetPosition(), self.p[s].GetVelocity()))
            means[:, s] = pv
            mean[:] += pv*self.p_w[s]

            rot = Rotation.from_matrix(self.p[s].GetRotationMatrix()).as_quat()
            quats[:, s] = rot/np.linalg.norm(rot)
            quat[:] += rot*self.p_w[s]

        quat = quat/np.linalg.norm(quat)
        rot = Rotation.from_quat(quat).as_matrix()

        means = means - mean.reshape(-1, 1)
        pos_cov = means[0:3, :]@means[0:3, :].T
        vel_cov = means[3:6, :]@means[3:6, :].T
        cov = block_diag(np.eye(3), vel_cov, pos_cov)
        mean_lie = np.eye(5)
        mean_lie[0:3, 0:3] = rot
        mean_lie[0:3, 3] = mean[3:6]
        mean_lie[0:3, 4] = mean[0:3] 

        self.state.SetMean(mean_lie)
        self.state.SetCovariance(cov)


    def GetState(self):
        return deepcopy(self.state)

    def SetState(self, state):
        self.state = deepcopy(state)