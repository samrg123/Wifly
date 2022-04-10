import numpy as np

from utils.utils import *
from system.SensorValue import SensorValue

class NormalNoise:
    
    def __init__(self, mean = np.zeros(3), covariance = np.diag(np.ones(3))):
        
        self.mean = mean

        covariance = np.asarray(covariance)

        if covariance.size == len(mean): 
            self.covariance = np.diag(covariance)
        elif covariance.size == len(mean)**2: 
            self.covariance = covariance
        else:
            Panic(f"Unsupported covariance size: {covariance.size} | meanLen: {len(mean)}")

    @staticmethod
    def Zero():
        return NormalNoise(np.zeros(3), np.zeros((3, 3)))

    def Sample(self):
        return gRng.multivariate_normal(self.mean, self.covariance)

class MotionNoise:

    def __init__(self, velocityNoise = NormalNoise.Zero(), orientationNoise = NormalNoise.Zero()):

        self.velocityNoise = velocityNoise
        self.orientationNoise = orientationNoise
 
class SensorNoise:

    def __init__(self, 
                gyroNoise          = NormalNoise.Zero(), 
                accelerometerNoise = NormalNoise.Zero(), 
                rssiNoise          = NormalNoise([0], [0])):

        self.gyroNoise = gyroNoise
        self.accelerometerNoise = accelerometerNoise

        # TODO: implement special type of RSSI noise that is slow changing, but can swing a large range
        self.rssiNoise = rssiNoise   

    def Sample(self):
        return SensorValue(
            linearAcceleration = self.accelerometerNoise.Sample(),
            angularVelocity    = self.gyroNoise.Sample(),
            rssi               = self.rssiNoise.Sample(),
        )
