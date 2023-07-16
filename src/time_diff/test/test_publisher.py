import os
import sys
import time
import unittest
import pandas as pd
import numpy as np


import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from rclpy.node import Node

from interfaces.msg import Gps
from std_msgs.msg import Float32

class PublisherTest(Node):

    def __init__(self):
        super().__init__("test_node")
        self.subscription = self.create_subscription(Gps,'data', self.listener_callback,10)
        self.subscription
        self.count = 0         # Counter to count the number of times a message has been received

        # Two values to store the time stamp of the first and last value to verify publisher rate
        self.time_stamp_of_first_val = 0    
        self.time_stamp_of_last_val = 0


    def listener_callback(self,data):

        # Verify Each message is Correct
        if self.count == 0:
            assert (data.seq == 1 and data.latitude == 41.233283 and data.longitude == -105.434187
                    and data.actual_speed == 2.77628018691555 and data.time == 1406394235)   
            self.time_stamp_of_first_val  = data.header.stamp.sec + data.header.stamp.nanosec*10**-9                               

        elif self.count == 1:
            assert (data.seq == 2 and data.latitude == 61.233283 and data.longitude == 105.434187
                    and data.actual_speed == 3.77628018691555 and data.time == 1406462635)

        elif self.count == 2:
            assert (data.seq == 3 and data.latitude == -61.233283 and data.longitude == -105.4187
                    and data.actual_speed == 2.71555 and data.time == 1437930235)
            
        elif self.count == 3:
            assert (data.seq != 3 and data.latitude != -61.233283 and data.longitude != -105.4187
                    and data.actual_speed != 2.71555 and data.time != 1437930235)
            self.time_stamp_of_last_val  = data.header.stamp.sec + data.header.stamp.nanosec*10**-9

            
        self.count = self.count+1


@pytest.mark.rostest
class TestSubscriber(unittest.TestCase):

    @classmethod 
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = PublisherTest()
        

    def tearDown(self) -> None:
        self.node.destroy_node()
    
    
    # Test if the publisher is publishing properly with correct values
    def test_publisher(self):        
        
        # Create a testing data frame
        df = pd.DataFrame(data = {"Latitude":[np.float64(41.233283).item() , np.float64(61.233283).item(), np.float64(-61.233283).item(), np.float64(-1.233283).item() ],
                                  "Longitude":[np.float64(-105.434187).item() , np.float64(105.434187).item(), np.float64(-105.4187).item(), np.float64(-1.233283).item()],
                                  "Time":["2014-07-26 13:03:55+00:00", "2014-07-27 13:03:55+05:00","2015-07-26 13:03:55+00:00","2014-07-26 13:03:55+00:00"],
                                  "Actual_Speed":[np.float64(2.77628018691555).item() , np.float64(3.77628018691555).item(), np.float64(2.71555).item(), np.float64(0.71555).item() ]
                                  })                            
        
        df.to_csv("test_data.csv")    # Save the test CSV

        os.system('screen -S ros_node -dm ros2 run time_diff data_publisher --ros-args --param file:=test_data.csv ')                                   # Launching the publisher node

        # Generating 3 custom messages to publish
        try:
            
            # spin each time to get the callback

            rclpy.spin_once(self.node,timeout_sec=2)
            
            rclpy.spin_once(self.node,timeout_sec=2)

            rclpy.spin_once(self.node,timeout_sec=2)

            rclpy.spin_once(self.node,timeout_sec=2)

            # Verify the Frequency of messages being published
            self.assertAlmostEqual(self.node.time_stamp_of_last_val-self.node.time_stamp_of_first_val, 1.0,places=2)

            # spin one extra time to confirm no extra message is published when the 
            # publisher has gone through the entire data frame
            rclpy.spin_once(self.node,timeout_sec=2)

            # # Verify that we only received messages three times
            self.assertEqual(self.node.count, 4)                                                                        

        finally:
            os.system("screen -S ros_node -X quit")
            os.system("rm test_data.csv")           # Delete the test csv



if __name__=="__main__":
    unittest.main()