
import numpy as np
from functools import partial

from utils.utils import wrap2Pi
from system.RobotState import RobotState

class myStruct:
    pass

def motionFunction(state, sensor):

    position = state.getPosition()
    theta = state.getOrientation()

    linearVelocity = sensor.getLinearVelocity()
    angularVelocity = sensor.getAngularVelocity()

    return RobotState(
        position = position + linearVelocity,
        orientation = theta + angularVelocity 
    )

def GetNoise(v, w, velocityNoiseDensity, angularVelocityNoiseDensity):
    return velocityNoiseDensity*np.linalg.norm(v) + \
           angularVelocityNoiseDensity*w*w

def MotionNoiseFunction(sensor, alphas):
    v = sensor.getLinearVelocity()
    w = sensor.getAngularVelocity()
    
    return np.diag([
        GetNoise(v, w, alphas[0], alphas[1]),
        GetNoise(v, w, alphas[2], alphas[3]),
        GetNoise(v, w, alphas[4], alphas[5])
    ])

def system_initialization(alphas):

    sys = myStruct()
    sys.motionFunction = motionFunction
    sys.motionNoiseMatrix = partial(MotionNoiseFunction, alphas=alphas)

    return sys

