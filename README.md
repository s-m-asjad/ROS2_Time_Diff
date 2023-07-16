# Description
This repository contains a simple ROS2 publisher and subscriber to find the time difference of GPS measurements in the Dataset file. It further contains unit tests to evaluate the correctness of the two ROS nodes.

Credits for dataset file : https://github.com/Lavkushwaha/GPS-Data-Prediction-ML-RNN


# Requirements

Install Screen

    sudo apt-get install screen


Python Dependencies

    pip3 install -r requirements.txt


# Running



## Clone and Build


Clone and build the repository

    git clone https://github.com/s-m-asjad/ROS2_Time_Diff.git
    cd ROS2_Time_Diff
    colcon build
    source install/setup.bash




## Running the Publisher and Subscriber


To run both two ROS nodes (Publisher and Subscriber)

    ros2 launch time_diff time_diff.launch.py "file_path:=<Absolute File Path>.csv"         # This is a Required argument.



To run only publisher (publishes to topic /data)

     ros2 run time_diff data_publisher --ros-args --param file:=<absolute path to file>.csv 
     # Not a Required argument, will use Dataset.csv in the workspace if value is not provided



To run only subscriber (subscribes to /data and publishes to /diff when more than one message has been subscribed)

    ros2 run time_diff data_subscriber




## Running Unit Tests


To run unit tests

    colcon test

The results of the tests are saved to log file under log/latest_test/time_diff


# Code Overview

### `data_publisher.py`
- Checks if data file exists 

- Checks data file is not empty 

- Asks `parsing.py` to parse the data 

- publishes data to topic /data since it is a ROS node 

- Shuts down when end of file is reached


### `parsing.py`
- Checks if the required columns exist 
- Evaluates if the data types and values are correct / within bounds 
- returns parsed values to `data_publisher.py` in the format required by `Gps.msg`


### `data_subscriber.py`
- Receives data from topic /data 
- If there is no previously received message then waits for the next message 
- Evaluates if the messages received are in sequence -> Calculates the time difference (this happens NOT for the header.stamp but for the time variable containing the time from csv file) 
- publishes the difference to topic /diff


### `test_parsing.py` 
- Tests for correct and incorrect values/failing cases for `parsing.py`


### `test_subscriber.py`
- Tests if subscriber is calculating the difference correctly
- Tests if subscriber is subscribed to the correct topic (and does not listen to any other topic)
- Tests if subscruber is publishing the difference to the correct topic

### `test_publisher.py`
- Tests if publisher is publishing data correctly (in the correct format and their respective fileds in the message)
- Tests if publisher is publishing on the correct topic
- Tests if publisher stops publishing once end of file is reached
- Tests if publisher is publishing at the correct rate
`






