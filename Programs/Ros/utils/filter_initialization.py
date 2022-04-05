
import numpy as np
from functools import partial
from numpy.random import default_rng
from utils.utils import *

rng = default_rng()

class myStruct:
    pass

init = myStruct()

def filter_initialization(sys, initialStateMean, initialStateCov, filter_name):

    if filter_name == "TestFilter":

        init.mu = initialStateMean
        init.Sigma = initialStateCov

        init.n = 100
        init.p = np.zeros((len(initialStateMean), init.n))
        init.p_w = np.zeros(init.n)
    
        from filter.TestFilter import TestFilter
        filter_ = TestFilter(sys, init)
    else:

        Panic(f"Unsupported filter {filter_name}")
    
    return filter_