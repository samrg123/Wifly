import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import expm, logm

from numpy.random import default_rng
from system.RobotState import *

gForceOfGravity = 9.80665
gGravity = np.array([[0, 0, gForceOfGravity]]).T

gRng = default_rng()

def Panic(str, errorCode = 1):
    print(f"PANIC - {str}")
    exit(errorCode)

def Warn(str):
    print(f"WARN - {str}")

def Log(str):
    print(f"MSG - {str}")        

def GetParam(params, arg, default=None):
    return params[arg] if arg in params else default

def wrap2Pi(input):
    phases =  (( -input + np.pi) % (2.0 * np.pi ) - np.pi) * -1.0

    return phases
