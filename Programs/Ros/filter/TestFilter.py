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
        self.integrationState = RobotState.Copy(initialState)

        initialMean       = initialState.GetMean()
        initialCovariance = initialState.GetCovariance()

        #TODO: What is this? store params in settings.yaml 
        self.R = GetParam(params, "wifiErrorCovariance", 5)
        self.n = GetParam(params, "numParticles", 100)

        w = 1/self.n if self.n > 0 else 0
        self.p_w = w * np.ones(self.n)

        self.particleSensorNoise = SensorNoise(
            # gyroNoise          = NormalNoise(mean = np.zeros(3), covariance = [0, 0, 1] * np.full(3, 7.61543504e-7)),
            # accelerometerNoise = NormalNoise(mean = np.zeros(3), covari
            gyroNoise          = NormalNoise(mean = np.zeros(3), covariance = [0, 0, 1] * np.full(3, .5)),
            accelerometerNoise = NormalNoise(mean = np.zeros(3), covariance = [1, 1, 0] * np.full(3, 20.0397832)),
            rssiNoise          = NormalNoise(mean = [0], covariance = [10])
        ) 

        self.p = [ RobotState.Copy(initialState) ]
        for i in range(self.n - 1): 
            particleState = initialState.SampleNeighborhood()
            self.p.append(particleState)

    def GetParticleStates(self):
        return np.copy(self.p)

    def GetIntegrationState(self):
        return RobotState.Copy(self.integrationState)

    def prediction(self, sensorValue, deltaT):

        covariance = self.state.GetCovariance()

        self.p[0] = self.motionFunction(self.p[0], sensorValue, deltaT)

        # simply propagate the state and assign identity to covariance
        for i in range(1, self.n): 

            state = self.p[i]
            sensorNoise = self.particleSensorNoise.Sample()
            self.p[i] = self.motionFunction(state, sensorValue + sensorNoise, deltaT)

        predictedCovariance = covariance

        self.integrationState = self.motionFunction(self.integrationState, sensorValue, deltaT)

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

        wifiMap = self.wifiMap    
        rssi = sensorValue.GetRssi()
        
        w = np.zeros(self.n)
        for i in range(self.n):
            
            particleState = self.p[i]
            wifi = wifiMap.QueryWifi(particleState)

            # # TODO: simulated data ignores these values
            # if (wifi.x == -1 and wifi.y == -1) or wifi.occupied: 
            #     w[i] = 0
            #     continue
            
            v = rssi - wifi.intensity
            
            w[i] = multivariate_normal.pdf(v, 0, self.R)

        if np.sum(w) != 0: 
            self.p_w = self.p_w * w
            self.p_w = self.p_w/np.sum(self.p_w)

        self.Neff = 1/np.sum(self.p_w * self.p_w)
        if self.Neff < (.2 * self.n): 
            self.resampling()

        self.p_w = self.p_w/np.sum(self.p_w)

        self.state = RobotState.MeanState(self.p, self.p_w)
        # self.mean_variance()


    def resampling(self): 
        W = np.cumsum(self.p_w)
        r = np.random.rand(1) / self.n
        j = 1
        for i in range(self.n): 
            u = r + (i-1)/self.n
            while u > W[j]: 
                j = j + 1

            self.p[i] = RobotState.Copy(self.p[j])

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