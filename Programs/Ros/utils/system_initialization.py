
import numpy as np
from functools import partial

from utils.utils import wrap2Pi
from system.RobotState import RobotState

class myStruct:
    pass

def MotionFunction(state, sensorValue, deltaT):

    theta          = state.GetOrientation()
    position       = state.GetPosition()
    linearVelocity = state.GetVelocity()

    angularVelocity    = sensorValue.GetAngularVelocity()
    linearAcceleration = sensorValue.GetLinearAcceleration()

    return RobotState(
        position    = position + linearVelocity * deltaT,
        orientation = theta + angularVelocity * deltaT,
        velocity    = linearVelocity + linearAcceleration * deltaT
    )

def GetNoise(linearVelocity, angularVelocity, velocityNoiseDensity, angularVelocityNoiseDensity):
    return velocityNoiseDensity * np.linalg.norm(linearVelocity) + \
           angularVelocityNoiseDensity * angularVelocity*angularVelocity

# TODO: add alphas for the accleration terms
def MotionNoiseFunction(state, sensorValue, alphas):
    linearVelocity = sensorValue.GetVelocity()
    angularVelocity = sensorValue.GetAngularVelocity()
    
    return np.diag([
        GetNoise(linearVelocity, angularVelocity, alphas[0], alphas[1]),
        GetNoise(linearVelocity, angularVelocity, alphas[2], alphas[3]),
        GetNoise(linearVelocity, angularVelocity, alphas[4], alphas[5])
    ])

def system_initialization(alphas):

    sys = myStruct()
    sys.motionFunction = MotionFunction
    sys.motionNoiseMatrix = partial(MotionNoiseFunction, alphas=alphas)

    return sys

