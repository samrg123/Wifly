import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import expm, logm

from system.RobotState import *

def Panic(str, errorCode = 1):
    print(f"PANIC - {str}")
    exit(errorCode)

def Warn(str):
    print(f"WARN - {str}")

def Log(str):
    print(f"MSG - {str}")        

def wrap2Pi(input):
    phases =  (( -input + np.pi) % (2.0 * np.pi ) - np.pi) * -1.0

    return phases
