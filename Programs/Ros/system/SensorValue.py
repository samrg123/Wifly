import numpy as np
import rospy
import yaml
from utils.utils import *

class SensorValue:

    def __init__(self, 
                 linearAcceleration = np.zeros(3), 
                 angularVelocity = np.zeros(3),
                 rssi = -100):

        self.linearAcceleration = linearAcceleration
        self.angularVelocity = angularVelocity
        self.SetRssi(rssi)

    def __mul__(self, x):
        self.linearAcceleration*= x
        self.angularVelocity*= x
        return self

    def __rmul__(self, x):
        return self.__mul__(x)
            
    def __str__(self):
        return f"SensorValue {{ linearAcceleration: {self.linearAcceleration}, angularVelocity: {self.angularVelocity}, rssi: {self._rssi} }}"

    def GetLinearAcceleration(self):
        return np.copy(self.linearAcceleration)

    def GetAngularVelocity(self):
        return np.copy(self.angularVelocity)

    def SetLinearAcceleration(self, a):
        np.copyto(self.linearAcceleration, a)

    def SetAngularVelocity(self, w):
        np.copyto(self.angluar_vel, w)

    def GetRssi(self):
        return self._rssi

    def SetRssi(self, rssi):
        self._rssi = np.clip(int(rssi), -100, 0)

