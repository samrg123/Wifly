import numpy as np
import yaml
from scipy.io import loadmat
from data.generate_data import generateScript

class DataHandler:

    def __init__(self):
        with open("config/settings.yaml", 'r') as stream:
            param = yaml.safe_load(stream)

        self.data_pth = param['data_path']

    def load_2d_data(self):
        out = {}
        
        with open("config/settings.yaml", 'r') as stream:
            param = yaml.safe_load(stream)
            alphas = np.array(param['alphas_sqrt'])**2 
            beta = param['beta']/180*np.pi

        numSteps = 100
        deltaT = 0.1
        initialStateMean = [180,50,0]
        initialStateCov = np.eye(3)

        data = generateScript(initialStateMean, numSteps, alphas, beta, deltaT)
        self.data = data

        out['motionCommand'] = np.array(data[:,6:9]) # [Trans_vel,Angular_vel,gamma]' noisy control command

        out['actual_state'] = np.array(data[:,15:18])
        out['noise_free_state'] = np.array(data[:,18:21]) 

        out['noisefreeBearing_1'] = np.array(data[:, 9])
        out['noisefreeBearing_2'] = np.array(data[:, 12])

        return out
