import rclpy
from rclpy.node import Node
import pandas as pd
import os
import sys
sys.path.insert(0,os.path.abspath(os.path.join(os.path.dirname(__file__),"./submodules")))
import parsing


from std_msgs.msg import Header
from interfaces.msg import Gps

class DataPublisher(Node):

    # initialize the node
    def __init__(self):

        # ROS Formalities
        super().__init__('data_publisher')                              # Name the node data_publisher
        self.publisher_ = self.create_publisher(Gps, 'data', 10)        # Ask it to publish on topic data
        timer_period = 1/3.0                                            # set publish rate to 3 Hz             
        self.timer = self.create_timer(timer_period, self.timer_callback)

        
        # Obtain path to the data file
        self.declare_parameter("file", value = os.path.join(os.getcwd()+"/Dataset.csv") )
        file_path = self.get_parameter("file").get_parameter_value().string_value


        # Read the data file
        try:
            self.data = pd.read_csv(file_path)
        except:
            self.get_logger().error("The requested file does not exist")
            raise FileNotFoundError("File Does not Exist")
        
        # Confirming that the file is not empty
        if (len(self.data.index)==0):
            self.get_logger().error("The requested file is empty")
            raise ValueError("No data in file")
                

        # Confirming that file has the required columns
        cols = ["Longitude","Latitude","Time","Actual_Speed"]        
        for col in cols:
            if col not in list(self.data.columns):
                self.get_logger().error("The requested file has columns missing. Missing : " + col)
                raise ValueError("File has missing column : " + col)                

        # Other initializations
        self.seq = 0                                                    # a sequence number to track how many messages have been published
        

       



    
    
    # The global executer for the ROS node
    def timer_callback(self):       

        # Create an object for custom message
        msg = Gps()


        # Obtain the Latitude value and check if it is a valid entry
        ############################################################

        msg.latitude = parsing.getLatitude(self.data.iloc[self.seq])



        # Obtain the Longitude value and check if it is a valid value
        #############################################################

        msg.longitude = parsing.getLongitude(self.data.iloc[self.seq])


        # Obtain the Time value and do checks to ensure it is a valid value
        ###################################################################

        msg.time = parsing.getTime(self.data.iloc[self.seq])
        

        # Obtain the speed value and check if it is a valid value
        ##############################################################

        msg.actual_speed =  parsing.getSpeed(self.data.iloc[self.seq])        

        
        # Publishing the Message after final logistics
        ##############################################                 
        
        self.seq = self.seq+1

        msg.seq = self.seq                                  # Assigning the Frame Sequence
        msg.header.frame_id =""                             # Assigning the Frame ID
        msg.header.stamp = self.get_clock().now().to_msg()  # Time stamping the message
        

        self.publisher_.publish(msg)                        #Publishing the message
        
        self.get_logger().info('Publishing message number : "%s"' % self.seq)

        

        if self.seq == len(self.data.index):
            self.get_logger().info("Completed traversing file. Shutting Down")
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)
        
        


def main(args=None):    
    
    # Create a node
    rclpy.init(args=args)    
    node = DataPublisher()  

    # Spin
    rclpy.spin(node)

    # End node
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()