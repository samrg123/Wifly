import numpy as np
import rospy
import yaml
from utils.utils import *

class RobotState:
    '''
    Robot State:
    2D: X = [x,y,theta]; position = [x,y]; orientation = [theta]
    '''
    def __init__(self, time_stamp=None, position=None, orientation=None):
        
        # load params
        with open("config/settings.yaml", 'r') as stream:
            param = yaml.safe_load(stream)

        if time_stamp is not None:
            self.time_stamp_ = time_stamp
        else:
            self.time_stamp_ = 0
        
        self.world_dim = param["world_dimension"]
        if self.world_dim != 2:
            Panic(1)

        # Note: SE(2)
        self.X = np.zeros(3)
        self.P = np.eye(3)

        if position is not None:
            self.setPosition(position)

        if orientation is not None:
            self.setOrientation(orientation)
    
    def setTime(self, time_stamp):
        if time_stamp is not None and isinstance(id,float):
            self.time_stamp_ = time_stamp

    def getTime(self):
        return self.time_stamp_

    def setPosition(self, position):
        self.X[0:2] = position

    def getPosition(self):
        return np.copy(self.X[0:2])

    def setOrientation(self, orientation):
        self.X[2] = orientation

    def getOrientation(self):
        return np.copy(self.X[2])

    def setPositionCovariance(self, cov_in):
        self.P[0:2,0:2] = cov_in


    def getPositionCovariance(self):
        return np.copy(self.P[0:2,0:2])


    def setOrientationCovariance(self, cov_in):
        self.P[3,3] = cov_in

    def getOrientationCovariance(self):
        return np.copy(self.P[3,3])

    def setCovariance(self, cov_in):
        self.P = cov_in

    def getCovariance(self):
        return np.copy(self.P)

    def setState(self, X_in):
        self.X = np.copy(X_in)
        
    def getState(self):
        return np.copy(self.X)

def main():
    rob_sys = RobotState()

if __name__ == '__main__':
    main()