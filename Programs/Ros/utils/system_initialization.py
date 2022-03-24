
import numpy as np
from functools import partial
from scipy.linalg import expm, block_diag

from utils.utils import *
from system.RobotState import RobotState

class myStruct:
    pass

gGyroBias = np.zeros((3, 1))
# gGyroBias = np.array([[ -0.02051761, 0.006928025, 0.3009694]]).T
# gGyroBias = np.array([[ -0.08460183, 0.008393568, 0.008526799]]).T

gAccelerometerBias = np.zeros((3, 1))
# gAccelerometerBias = np.array([[0, 0, 9.80665]]).T
# gAccelerometerBias = np.array([[ 0.6105214, -0.3591303, 9.437943 ]]).T

gVelocityBias = np.zeros((3, 1))

def MotionFunction(state, sensorValue, deltaT):

    # derivative of state in sensor frame
    biasedDerivate = np.hstack((
        sensorValue.GetAngularVelocity(),
        sensorValue.GetLinearAcceleration(),
        state.GetVelocity()
    )).reshape((-1, 1))

    # sensor bias in world frame
    bias = np.vstack((gGyroBias, gAccelerometerBias, gVelocityBias))

    # compute delta state in world frame
    # inverseRotationMatrix = np.linalg.inv(state.GetRotationMatrix())
    inverseRotationMatrix = state.GetRotationMatrix().T
    stackedInverseRotationMatrix = block_diag(inverseRotationMatrix, inverseRotationMatrix, inverseRotationMatrix)

    derivative = stackedInverseRotationMatrix @ biasedDerivate - bias
    deltaState = RobotState.Wedge(derivative*deltaT)
    
    # print("deltaState")
    # print(deltaState)

    # print("expm(deltaState)")
    # print(expm(deltaState))

    # return integrated state

    r = RobotState.FromMean( state.GetMean() @ expm(deltaState) )

    return r

def GetNoise(linearVelocity, angularVelocity, velocityNoiseDensity, angularVelocityNoiseDensity):
    return velocityNoiseDensity * np.linalg.norm(linearVelocity) + \
           angularVelocityNoiseDensity * angularVelocity*angularVelocity

# TODO: add alphas for the accleration terms
def MotionNoiseFunction(state, sensorValue, alphas):
    
    Panic("RETURN 9x9 noise matrix that uses noise density on diagonal")
    
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

