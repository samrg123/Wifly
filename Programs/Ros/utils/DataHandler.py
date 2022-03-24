import yaml

from utils.utils import *

from data.DataSimulator import DataSimulator
from data.LogSampler import LogSampler

class DataHandler:

    def __init__(self, system):

        with open("config/settings.yaml", 'r') as stream:
            params = yaml.safe_load(stream)
        
        dataType = params['dataType']
        self.dataType = dataType
        
        if dataType == "simulated":

            self.dataSampler = DataSimulator(params, system)

        elif dataType == "log":
            
            self.dataSampler = LogSampler(params)
        
        elif dataType == "streamed":
            Panic("Not Implemented")
        else:
            Panic(f"Unsupported dataType: {dataType}")
        
        self.dataSampler.LoadSamples()

        computeBiasSamples = GetParam(params, "computeBiasSamples", 0)
        
        if computeBiasSamples > 0:
            
            system.gyroBias[:] = 0
            system.velocityBias[:] = 0
            system.accelerometerBias[:] = 0

            for i in range(computeBiasSamples):

                sample = self.dataSampler.GetSample()
                if sample == False:
                    computeBiasSamples = i+1
                    break


                # TODO: This is really just a hack to average real world data 
                #       We should really be comparing these values by comparing to ground truth
                system.gyroBias+= sample.sensorValue.GetAngularVelocity()
                system.accelerometerBias+= sample.sensorValue.GetLinearAcceleration()

                if sample.deltaT != 0:
                    system.velocityBias+= (sample.groundTruthState.GetVelocity() - sample.commandState.GetVelocity()) / sample.deltaT

            normailzer = 1./computeBiasSamples
            system.gyroBias*= normailzer
            system.velocityBias*= normailzer
            system.accelerometerBias*= normailzer

            Log(f"Computed Biases ({computeBiasSamples} samples) {{\n\
                  \tgyroBias: {system.gyroBias}\n\
                  \tvelocityBias: {system.velocityBias}\n\
                  \taccelerometerBias: {system.accelerometerBias}\n\
                  }}")

            self.dataSampler.Reset()
