import numpy as np
import rospy
import yaml
from utils.utils import *

class Sensor:

    def __init__(self, linearVelocity = np.zeros(2), angularVelocity = np.zeros(1)):
     
        with open("config/settings.yaml", 'r') as stream:
            param = yaml.safe_load(stream)

        # TODO: make these 3d when ready
        self.linear_vel = linearVelocity
        self.angular_vel = angularVelocity
        
    def getLinearVelocity(self):
        return self.linear_vel

    def getAngularVelocity(self):
        return self.angular_vel

    def setLinearVelocity(self, v):
        self.linear_vel = np.copy(v)

    def setAngularVelocity(self, w):
        self.angluar_vel = np.copy(w)

