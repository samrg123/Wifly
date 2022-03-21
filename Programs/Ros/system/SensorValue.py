import numpy as np
import rospy
import yaml
from utils.utils import *

class SensorValue:

    def __init__(self, linearVelocity = np.zeros(2), angularVelocity = np.zeros(1)):
     
        with open("config/settings.yaml", 'r') as stream:
            param = yaml.safe_load(stream)

        # TODO: make these 3d when ready
        self.linearVelocity = linearVelocity
        self.angularVelocity = angularVelocity
    
    def __mul__(self, x):
        self.linearVelocity*= x
        self.angularVelocity*= x
        return self

    def __rmul__(self, x):
        return self.__mul__(x)
            
    def __str__(self):
        return f"SensorValue {{ linearVelocity: {self.linearVelocity}, angularVelocity: {self.angularVelocity} }}"

    def getLinearVelocity(self):
        return self.linearVelocity

    def getAngularVelocity(self):
        return self.angularVelocity

    def setLinearVelocity(self, v):
        self.linearVelocity = np.copy(v)

    def setAngularVelocity(self, w):
        self.angluar_vel = np.copy(w)

