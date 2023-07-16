import os
import sys
import time
import unittest


import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from rclpy.node import Node

from interfaces.msg import Gps
from std_msgs.msg import Float32

class SubscriberTest(Node):

    def __init__(self):
        super().__init__("test_node")
        self.subscription = self.create_subscription(Float32,'diff', self.listener_callback,10)
        self.subscription
        self.count = 0         # Counter to count the number of times a message has been received


    def listener_callback(self,data):

        if self.count == 0:
            assert (data.data == 3)                                    # Time difference between message 1 and 2 is 3
            

        elif self.count == 1:
            assert (data.data == 17) and (data.data != 3)             # Time difference between message 2 and 3 is 17
        
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
        self.node = SubscriberTest()
        

    def tearDown(self) -> None:
        self.node.destroy_node()

    def create_msg(self, seq, lon, lat, _time, speed):
        msg = Gps()
        msg.seq = seq
        msg.longitude = lon
        msg.latitude = lat
        msg.time = _time
        msg.actual_speed = speed

        return msg        
        
    
    # Testing if the subscriber receives messages, performs the correct calculation and then publishes successfully
    def test_calculation_and_publishing_of_messages(self ):                             

        

        os.system('screen -S ros_node -dm  ros2 run  time_diff data_subscriber ')                                   # Launching the subscriber node

        # Generating 3 custom messages to publish
        try:
            msg1 = self.create_msg(1,-105.434187,41.233283,1406388235.0 , 3.0 )
            msg2 = self.create_msg(2,-105.434177,41.233209,1406388238.0 , 3.0 )
            msg3 = self.create_msg(3,-105.434183,41.233247,1406388255.0 , 3.0 )

            self.node.publisher_ = self.node.create_publisher(Gps, 'data', 10)                                          # Create a publisher

            
            time.sleep(3)                                                                                               # Give some time for subscriber to initialize

            
            # Publish the messages in a sequence and spin each time to get the callback
            self.node.publisher_.publish(msg1)

            
            self.node.publisher_.publish(msg2)
            rclpy.spin_once(self.node,timeout_sec=2)

            

            self.node.publisher_.publish(msg3)
            rclpy.spin_once(self.node,timeout_sec=2)


            # Giving one additional spin to ensure the subscriber was not subscribing to some bogus topic
            rclpy.spin_once(self.node,timeout_sec=2)        

            # Verify that we received time differences twice
            self.assertEqual(self.node.count, 2)  

        finally:
            os.system("screen -S ros_node -X quit")



if __name__=="__main__":
    unittest.main()