import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from interfaces.msg import Gps


class DataSubscriber(Node):

    def __init__(self):
        
        # ROS Formalities
        super().__init__('data_subscriber')
        self.subscription = self.create_subscription(Gps,'data', self.listener_callback,10)
        self.publisher_ = self.create_publisher(Float32, 'diff', 10)
        self.subscription

        # Logistics
        self.previous_seq = None             # Stores the previous sequence ID
        self.previous_time = None           # Stores the previous Time in the message



    def listener_callback(self, msg):

        if (self.previous_seq == None):      # Stores the values for when we run it the first time
            self.previous_seq = msg.seq
            self.previous_time = msg.time
        else:

            if(msg.seq - self.previous_seq!=1):  # Check if frames are received in sequence
                self.get_logger().error("Frames received are not in sequence, the difference of time calculation will be incorrect.")
                raise AssertionError("Frames received are not in sequence, the difference of time calculation will be incorrect.")
            
            diff = Float32()
            diff.data = (msg.time - self.previous_time)
            self.publisher_.publish(diff)      # Publish the message
            self.get_logger().info('The most recent difference in seconds was : "%s"' % diff.data)

            # Update the current values of sequence and time
            self.previous_time = msg.time
            self.previous_seq = msg.seq


def main(args=None):
    
    # Create Node
    rclpy.init(args=args)
    minimal_subscriber = DataSubscriber()

    #Spin
    rclpy.spin(minimal_subscriber)

    #Destroy Node
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()