import sys
sys.path.append('.')
import yaml
import matplotlib.pyplot as plt

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

from system.RobotState import *
from comm.path_publisher import *
from utils.DataHandler import *
from utils.filter_initialization import filter_initialization
from utils.system_initialization import system_initialization
from utils.utils import *

from system.Sensor import *

class RobotSystem:
    

    def __init__(self, world=None):

        rospy.init_node('robot_state_estimator', anonymous=True)

        # load params
        with open("config/settings.yaml", 'r') as stream:
            param = yaml.safe_load(stream)
        
        # load motion noise and sensor noise
        alphas = np.array(param['alphas_sqrt'])**2
        self.system = system_initialization(alphas)

        # load world and landmarks
        if world is not None:
            self.world = world
        else:
            print("Plase provide a world!")

        # load filter
        init_state_mean = np.array(param['initial_state_mean'])
        init_state_cov = np.diag(param['initial_state_variance'])**2        
        filter_name = param['filter_name']
        self.filter = filter_initialization(self.system, init_state_mean, init_state_cov, filter_name)

        #TODO: TWEAK ROS STUFF 
        # load data.
        # in real-world application this should be a subscriber that subscribes to sensor topics
        # but for this homework example we load all data at once for simplicity
        self.data_handler = DataHandler()
        self.data = self.data_handler.load_2d_data()

        self.num_step = np.shape(self.data['motionCommand'])[0]

        self.pub = path_publisher()     # filter pose
        self.cmd_pub = path_publisher() # theoratical command path
        self.gt_pub = path_publisher()  # actual robot path

        self.loop_sleep_time = param['loop_sleep_time']

    def run_filter(self):
        
        for t in range(self.num_step):
            
            # get data for current timestamp
            motionCommand = self.data['motionCommand'][t,:]
            motionVelocity, angularVelocity = motionCommand[0:2] 

            actualState = self.data['actual_state'][t,:]
            x, y, theta = actualState
 
            # create sensor reading
            sensorValues = Sensor(
                linearVelocity = np.array([
                    motionVelocity * np.cos(theta),
                    motionVelocity * np.sin(theta)
                ]),

                angularVelocity = angularVelocity
            )
            
            self.filter.prediction(sensorValues)

            # publisher 
            self.state = self.filter.getState()
            
            self.pub.publish_state_path(self.state)
            self.pub.publish_pose(self.state)
        
            noiseFreeState = self.data['noise_free_state'][t]
            self.gt_pub.publish_gt_path(actualState)
            self.cmd_pub.publish_command_path(noiseFreeState)

            rospy.sleep(self.loop_sleep_time)
        
        

def main():
    rob_sys = RobotSystem()

if __name__ == '__main__':
    main()