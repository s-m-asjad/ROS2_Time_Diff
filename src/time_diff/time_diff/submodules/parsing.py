import numpy
import pandas as pd
from datetime import datetime


###### Parse Latitude #########
def getLatitude(data):
    
    try:
        latitude =  data["Latitude"]
    except: 
        raise ValueError("No Latitude column")                      # Verify existence of Latitude column
        
    if type(latitude) != numpy.float64:                             # Verify Latitude Data type
        raise TypeError("Incorrect data type of Latitude")
    
    if latitude<-90.0 or latitude>90.0:                             # Verify value of Latitude
        raise ValueError("Invalid value of Latitude")
    
    return latitude



    

#### Parse Longitude ########
def getLongitude(data):

    try:
        longitude =  data["Longitude"]
    except: 
        raise ValueError("No Latitude column")                      # Verify existence of Longitude column

    if type(longitude) != numpy.float64:
        raise TypeError("Incorrect data type of Longitude")        # Verify Latitude Data type
    
    if longitude<-180.0 or longitude>180.0:
        raise ValueError("Invalid value of Longitude")              # Verify Longitude Data type
            
    return longitude




# Function to convert strings to integers and perform relevant checks
def convert_to_int(string, label):

    converted_val = None
    try:                                                            # Converting string to integer
        converted_val = int(string)
    except:
        raise TypeError("Could not convert"+ label +" to int.")
    
    return converted_val






#### Parse Time ########
def getTime(data):

    try:
        time_string =  data["Time"]
    except: 
        raise ValueError("No Time column")                      # Verify existence of Time column
    

    if(len(time_string) != 25):                          # Checking if the time string is of the expected length
        
        
        raise ValueError("String is incomplete")

    date_and_clock = time_string.split(" ")              # Separating the date from the 24 hour time

    if(len(date_and_clock)!=2):                         # Checking if we have the date and clock time in the original string
        
        
        raise ValueError("Date and clock string is incorrect.")
    
    
    date = date_and_clock[0]                            # Getting the date part of the string
    clock = date_and_clock[1]                           # Getting the clock part of the string

    YYYYMMDD = date.split("-")                          # Splitting the day, month and year from the date

    if(len(YYYYMMDD)!=3):                               # Checking if the day, month and year are present in the date
        
        
        raise ValueError("Day Month and Year part of Time string is incorrect.")
    

    year = YYYYMMDD[0]                                  # Getting the year string
    month = YYYYMMDD[1]                                 # Getting the month string
    day = YYYYMMDD[2]                                   # Getting the day string

    if(len(year)!=4 or len(month)!=2 or len(day)!=2):   # Checking if day month and year are in the correct format i.e. someone did not make it MMDDYYYY or YYMMDD etc
        
        raise ValueError("Day Month and Year lengths of Time string is incorrect.")
    
    year = convert_to_int(year, "YEAR")            # Converting year to int
    month = convert_to_int(month, "MONTH")         # Converting month to int
    day = convert_to_int(day, "DAY")               # Converting day to int

    if year<1970:                                          # Check if year has a valid value
        raise ValueError("Year can not be less than 1970")
    
    if month<=0 or month>12:                            # Check if month has a valid value
        raise ValueError("Invalid value for month")
    
    if day<=0 or day>31:                                # Check if day has a valid value
        raise ValueError("Invalid value for day.")
    
    elif day==31 and month in [1,3,5,7,8,10,12]:        # Check if day is 31 in month of 30 days
        raise ValueError("Current month can not have 31 days.")
    
    elif day>29 and month==2:                           # Check if day range is a valid value for February
        raise ValueError("February can not have more than 29 days.")
    
    elif day==29 and month==2 and year%4!=0:            # Check if the year is leap year, if February has 29 days
        raise ValueError("February can only have 29 days in a leap year.")
    

    if ("-" in clock):
        HHMMSS_and_zone = clock.split("-")                            # Separating the clock time and time zone (when the time zone is for locations on the West part of the world) 
        sign = 1                                                      # The sign which will decide how to accomodate the time zone 
    else:

        HHMMSS_and_zone = clock.split("+")                           # Separating the clock time and time zone (when the time zone is for locations on the East part of the world)
        sign = -1                                                    # The sign which will decide how to accomodate the time zone

    
    if(len(HHMMSS_and_zone)!=2):                                 # Checking if time zone and clock were separated correctly
        
        raise ValueError("Clock and Time zone part of time Time string is incorrect.")
    
    HHMMSS = HHMMSS_and_zone[0].split(":")                       # Splitting the clock part into hours minutes and seconds

    if(len(HHMMSS)!=3):                                          # Checking if time zone and clock were separated correctly
        
        raise ValueError("Hours Minutes and Seconds part of string is not complete.")
    
    if(len(HHMMSS[0])!=2 or len(HHMMSS[1])!=2 or len(HHMMSS[2])!=2):    #Checking if hours, minutes and seconds are 2 digit numbers
        raise ValueError("Hours Minutes and Seconds are not in correct format.")

    
    hours = convert_to_int(HHMMSS[0],"hours")
    minutes = convert_to_int(HHMMSS[1],"minutes")
    seconds = convert_to_int(HHMMSS[2],"seconds")

    if hours<0 or hours>23:                                        # Check if the hours value is valid

        raise ValueError("Invalid value for hour.")

    if minutes<0 or minutes>59:                                        # Check if the minutes value is valid

        raise ValueError("Invalid value for minutes.")

    if seconds<0 or seconds>59:                                        # Check if the seconds value is valid

        raise ValueError("Invalid value for seconds.")
    

    time_zone = HHMMSS_and_zone[1].split(":")                              # Obtaining time zone.

    if (len(time_zone)!=2):                                             # Checking if the time zone is complete
        raise ValueError("Time zone part of string is not complete.")
    
    if (len(time_zone[0])!=2 or len(time_zone[1])!=2):                  # Checking if the time zone format is correct
        raise ValueError("Time zone part of string is not correct.")
    

    UTC_hours = convert_to_int(time_zone[0],"UTC Hours")           # Converting UTC hours to int
    UTC_minutes = convert_to_int(time_zone[1],"UTC Minutes")       # Converting UTC minutes to int

    if UTC_hours < -12 or UTC_hours > 14:                               # Check if the UTC hours value is valid

        raise ValueError("Invalid value for UTC hours.")
    
    if UTC_minutes != 0 and UTC_minutes != 30 and UTC_minutes !=45:     # Check if the UTC hours value is valid

        raise ValueError("Invalid value for UTC minutes.")    
    
    
    # All checks completed, converting Time string to seconds
    hours = UTC_hours*sign + hours                                      #Correcting for UTC difference
    minutes = UTC_minutes*sign + minutes

    return datetime(year,month,day).timestamp() + (3600*hours) + (60*minutes) + seconds
     





#### Parse Speed ########
def getSpeed(data):
 

    try:
        speed = data["Actual_Speed"]
    except: 
        raise ValueError("No Speed column")                      # Verify existence of Speed column
    
    if type(speed) != numpy.float64:
        raise TypeError("Incorrect data type of actual speed")
            
    return speed


def __main__():
    pass


if __name__ == "__main__":
    __main__()