import rclpy
from rclpy.node import Node
import pandas as pd
import os
import sys
import numpy
from datetime import datetime


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
        

       

    # Function to convert strings to integers and perform relevant checks
    def convert_to_int(self, string, label):

        converted_val = None
        try:                                                            # Converting string to integer
            converted_val = int(string)
        except:
            self.get_logger().error("Could not convert"+ label +" to int.")
            raise TypeError("Could not convert"+ label +" to int.")
        
        return converted_val

    
    
    # The global executer for the ROS node
    def timer_callback(self):           

        # Create an object for custom message
        msg = Gps()




        # Obtain the Latitude value and check if it is a valid entry
        ############################################################
        
        latitude =  self.data["Latitude"][self.seq]
        
        if type(latitude) != numpy.float64:
            
            self.get_logger().error("Incorrect data type of Latitude")
            raise TypeError("Incorrect data type of Latitude")
        
        if latitude<-90.0 or latitude>90.0:
            
            self.get_logger().error("Invalid value of Latitude")
            raise ValueError("Invalid value of Latitude")

        msg.latitude = latitude




        # Obtain the Longitude value and check if it is a valid value
        #############################################################

        longitude =  self.data["Longitude"][self.seq]
        
        if type(longitude) != numpy.float64:
            
            self.get_logger().error("Incorrect data type of Longitude")
            raise TypeError("Incorrect data type of Longitude")
        
        if longitude<-180.0 or longitude>180.0:
            
            self.get_logger().error("Invalid value of Longitude")
            raise ValueError("Invalid value of Longitude")

        msg.longitude = longitude




        # Obtain the Time value and do checks to ensure it is a valid value
        ###################################################################

        time_string = self.data["Time"][self.seq]
        time_secs = 0.0

        if(len(time_string) != 25):                          # Checking if the time string is of the expected length
            
            self.get_logger().error("String is incomplete")
            raise ValueError("String is incomplete")

        date_and_clock = time_string.split(" ")              # Separating the date from the 24 hour time

        if(len(date_and_clock)!=2):                         # Checking if we have the date and clock time in the original string
           
            self.get_logger().error("Date and clock string is incorrect.")
            raise ValueError("Date and clock string is incorrect.")
        
        
        date = date_and_clock[0]                            # Getting the date part of the string
        clock = date_and_clock[1]                           # Getting the clock part of the string

        YYYYMMDD = date.split("-")                          # Splitting the day, month and year from the date

        if(len(YYYYMMDD)!=3):                               # Checking if the day, month and year are present in the date
            
            self.get_logger().error("Day Month and Year part of Time string is incorrect.")
            raise ValueError("Day Month and Year part of Time string is incorrect.")
        

        year = YYYYMMDD[0]                                  # Getting the year string
        month = YYYYMMDD[1]                                 # Getting the month string
        day = YYYYMMDD[2]                                   # Getting the day string

        if(len(year)!=4 or len(month)!=2 or len(day)!=2):   # Checking if day month and year are in the correct format i.e. someone did not make it MMDDYYYY or YYMMDD etc
            
            self.get_logger().error("Day Month and Year lengths of Time string is incorrect.")
            raise ValueError("Day Month and Year lengths of Time string is incorrect.")
        
        year = self.convert_to_int(year, "YEAR")            # Converting year to int
        month = self.convert_to_int(month, "MONTH")         # Converting month to int
        day = self.convert_to_int(day, "DAY")               # Converting day to int

        if year<1970:                                          # Check if year has a valid value
            self.get_logger().error("Year can not be less than 1970.")
            raise ValueError("Year can not be less than 1970")
        
        if month<=0 or month>12:                            # Check if month has a valid value
            self.get_logger().error("Invalid value for month.")
            raise ValueError("Invalid value for month")
        
        if day<=0 or day>31:                                # Check if day has a valid value
            self.get_logger().error("Invalid value for day.")
            raise ValueError("Invalid value for day.")
        
        elif day==31 and month in [1,3,5,7,8,10,12]:        # Check if day is 31 in month of 30 days
            self.get_logger().error("Current month can not have 31 days.")
            raise ValueError("Current month can not have 31 days.")
        
        elif day>29 and month==2:                           # Check if day range is a valid value for February
            self.get_logger().error("February can not have more than 29 days.")
            raise ValueError("February can not have more than 29 days.")
        
        elif day==29 and month==2 and year%4!=0:            # Check if the year is leap year, if February has 29 days
            self.get_logger().error("February can only have 29 days in a leap year.")
            raise ValueError("February can only have 29 days in a leap year.")
        

        if ("-" in clock):
           HHMMSS_and_zone = clock.split("-")                            # Separating the clock time and time zone (when the time zone is for locations on the West part of the world) 
           sign = 1                                                      # The sign which will decide how to accomodate the time zone 
        else:

            HHMMSS_and_zone = clock.split("+")                           # Separating the clock time and time zone (when the time zone is for locations on the East part of the world)
            sign = -1                                                    # The sign which will decide how to accomodate the time zone

        
        if(len(HHMMSS_and_zone)!=2):                                 # Checking if time zone and clock were separated correctly
            
            self.get_logger().error("Clock and Time zone part of time Time string is incorrect.")
            raise ValueError("Clock and Time zone part of time Time string is incorrect.")
        
        HHMMSS = HHMMSS_and_zone[0].split(":")                       # Splitting the clock part into hours minutes and seconds

        if(len(HHMMSS)!=3):                                          # Checking if time zone and clock were separated correctly
            
            self.get_logger().error("Hours Minutes and Seconds part of string is not complete.")
            raise ValueError("Hours Minutes and Seconds part of string is not complete.")
        
        if(len(HHMMSS[0])!=2 or len(HHMMSS[1])!=2 or len(HHMMSS[2])!=2):    #Checking if hours, minutes and seconds are 2 digit numbers
            self.get_logger().error("Hours Minutes and Seconds are not in correct format.")
            raise ValueError("Hours Minutes and Seconds are not in correct format.")

        
        hours = self.convert_to_int(HHMMSS[0],"hours")
        minutes = self.convert_to_int(HHMMSS[1],"minutes")
        seconds = self.convert_to_int(HHMMSS[2],"seconds")

        if hours<0 or hours>23:                                        # Check if the hours value is valid

            self.get_logger().error("Invalid value for hour.")
            raise ValueError("Invalid value for hour.")

        if minutes<0 or minutes>59:                                        # Check if the minutes value is valid

            self.get_logger().error("Invalid value for minutes.")
            raise ValueError("Invalid value for minutes.")

        if seconds<0 or seconds>59:                                        # Check if the seconds value is valid

            self.get_logger().error("Invalid value for seconds.")
            raise ValueError("Invalid value for seconds.")
        

        time_zone = HHMMSS_and_zone[1].split(":")                              # Obtaining time zone.

        if (len(time_zone)!=2):                                             # Checking if the time zone is complete
            self.get_logger().error("Time zone part of string is not complete.")
            raise ValueError("Time zone part of string is not complete.")
        
        if (len(time_zone[0])!=2 or len(time_zone[1])!=2):                  # Checking if the time zone format is correct
            self.get_logger().error("Time zone part of string is not correct.")
            raise ValueError("Time zone part of string is not correct.")
        

        UTC_hours = self.convert_to_int(time_zone[0],"UTC Hours")           # Converting UTC hours to int
        UTC_minutes = self.convert_to_int(time_zone[1],"UTC Minutes")       # Converting UTC minutes to int

        if UTC_hours < -12 or UTC_hours > 14:                               # Check if the UTC hours value is valid

            self.get_logger().error("Invalid value for UTC hours.")
            raise ValueError("Invalid value for UTC hours.")
        
        if UTC_minutes != 0 and UTC_minutes != 30 and UTC_minutes !=45:     # Check if the UTC hours value is valid

            self.get_logger().error("Invalid value for UTC minutes.")
            raise ValueError("Invalid value for UTC minutes.")    
        
        
        # All checks completed, converting Time string to seconds
        hours = UTC_hours*sign + hours                                      #Correcting for UTC difference
        minutes = UTC_minutes*sign + minutes

        msg.time = datetime(year,month,day).timestamp() + (3600*hours) + (60*minutes) + seconds

        

        # Obtain the speed value and check if it is a valid value
        ##############################################################

        speed =  self.data["Actual_Speed"][self.seq]
        
        if type(speed) != numpy.float64:
            
            self.get_logger().error("Incorrect data type of actual speed")
            raise TypeError("Incorrect data type of actual speed")
                
        msg.actual_speed = speed




        
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