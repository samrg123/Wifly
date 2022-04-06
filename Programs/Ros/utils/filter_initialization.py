
import numpy as np
from functools import partial
from numpy.random import default_rng
from utils.utils import *

rng = default_rng()

class myStruct:
    pass

init = myStruct()

def filter_initialization(sys, initialStateMean, initialStateCov, filter_name, params):

    if filter_name == "TestFilter":

        init.mu = initialStateMean
        init.Sigma = initialStateCov
    
        from filter.TestFilter import TestFilter
        filter_ = TestFilter(sys, init, params)
    else:

        Panic(f"Unsupported filter {filter_name}")
    
    return filter_