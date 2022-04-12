import numpy as np

from utils.utils import *
from system.SensorValue import SensorValue

class NormalNoise:
    
    def __init__(self, mean = np.zeros(3), covariance = np.diag(np.ones(3))):
        
        self.mean = np.asarray(mean, dtype="float")

        covariance = np.asarray(covariance, dtype="float")

        n = len(mean)
        if covariance.size == n: 
            self.covariance = np.diag(covariance).reshape(n, n)
        elif covariance.size == n**2: 
            self.covariance = covariance
        else:
            Panic(f"Unsupported covariance size: {covariance.size} | meanLen: {len(mean)}")

    def __str__(self):
        return f"NormalNoise {{\nMean: {self.mean}\nCovariance:\n{self.covariance}\n}}"

    @staticmethod
    def Zero(size = 3):
        return NormalNoise(np.zeros(size), np.zeros((size, size)))

    def Sample(self):
        return gRng.multivariate_normal(self.mean, self.covariance)

class MotionNoise:

    def __init__(self, 
                 velocityNoise    = NormalNoise.Zero(), 
                 orientationNoise = NormalNoise.Zero(),
                 positionNoise    = NormalNoise.Zero()):

        self.positionNoise    = positionNoise
        self.velocityNoise    = velocityNoise
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
