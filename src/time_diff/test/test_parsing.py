import unittest
import sys
import os
import numpy as np
import pandas as pd

sys.path.insert(0,os.path.abspath(os.path.join(os.path.dirname(__file__),"../time_diff/submodules")))
import parsing


class TestParsing(unittest.TestCase):
    
    ################################# Unit Testing Latitude ###########################
    def test_latitude(self):
        
        # Checking Correct Input
        self.assertEqual( parsing.getLatitude( pd.DataFrame(data = {"Latitude":[np.float64(45.2).item()]}).iloc[0] ), 45.2 )

        # Checking Incorrect Output
        self.assertNotEqual( parsing.getLatitude( pd.DataFrame(data = {"Latitude":[np.float64(45.2).item()]}).iloc[0] ), 45.3 )

        #Checking Incorrect Data Type
        with self.assertRaises(TypeError):
            parsing.getLatitude( pd.DataFrame(data = {"Latitude":["45.3"]}).iloc[0] )
            parsing.getLatitude( pd.DataFrame(data = {"Latitude":[25]}).iloc[0] )

        # Checking Incorrect Value
        with self.assertRaises(ValueError):
            parsing.getLatitude( pd.DataFrame(data = {"Latitude":[np.float64(95).item()]}).iloc[0] )
            parsing.getLatitude( pd.DataFrame(data = {"Latitude":[np.float64(-91).item()]}).iloc[0] )


            parsing.getLatitude( pd.DataFrame(data = {"latetude":[np.float64(45).item()]}).iloc[0] )    # Checking column names

            




    #################################Unit Testing Longitude ##############################

    def test_longitude(self):
        
        # Checking Correct Input
        self.assertEqual( parsing.getLongitude( pd.DataFrame(data = {"Longitude":[np.float64(125).item()]}).iloc[0] ), 125 )

        # Checking Incorrect Output
        self.assertNotEqual( parsing.getLongitude( pd.DataFrame(data = {"Longitude":[np.float64(122.2).item()]}).iloc[0] ), 45.3 )

        #Checking Incorrect Data Type
        with self.assertRaises(TypeError):
            parsing.getLongitude( pd.DataFrame(data = {"Longitude":["45.3"]}).iloc[0] )
            parsing.getLongitude( pd.DataFrame(data = {"Longitude":[25]}).iloc[0] )

        # Checking Incorrect Value
        with self.assertRaises(ValueError):
            parsing.getLongitude( pd.DataFrame(data = {"Longitude":[np.float64(360).item()]}).iloc[0] )
            parsing.getLongitude( pd.DataFrame(data = {"Longitude":[np.float64(-404).item()]}).iloc[0] )     

            parsing.getLatitude( pd.DataFrame(data = {"longotide":[np.float64(145).item()]}).iloc[0] )  # Checking column names




    

    #################################### Unit Testing Actual Speed ################################
    def test_actual_speed(self):
        
        # Checking Correct Input
        self.assertEqual( parsing.getSpeed( pd.DataFrame(data = {"Actual_Speed":[np.float64(28.9).item()]}).iloc[0] ), 28.9 )

        # Checking Incorrect Output
        self.assertNotEqual( parsing.getSpeed( pd.DataFrame(data = {"Actual_Speed":[np.float64(122.2).item()]}).iloc[0] ), 45.3 )

        #Checking Incorrect Data Type
        with self.assertRaises(TypeError):
            parsing.getSpeed( pd.DataFrame(data = {"Actual_Speed":["11.3"]}).iloc[0] )
            parsing.getSpeed( pd.DataFrame(data = {"Actual_Speed":[1]}).iloc[0] )

            parsing.getLatitude( pd.DataFrame(data = {"Speed":[np.float64(12).item()]}).iloc[0] )  # Checking column names





    
    #################################### Unit Testing Time ########################################
    def test_time(self):

        # Checking Correct Input
        self.assertNotEqual( parsing.getTime( pd.DataFrame(data = {"Time":["2014-07-26 20:41:17+00:00"]}).iloc[0] ), 1406419277 )
        self.assertNotEqual( parsing.getTime( pd.DataFrame(data = {"Time":["2012-02-29 20:41:17+00:00"]}).iloc[0] ), 1330518077 )
        self.assertNotEqual( parsing.getTime( pd.DataFrame(data = {"Time":["2014-07-26 20:41:17+05:00"]}).iloc[0] ), 1406383277 )


        # Checking Incorrect Output
        self.assertNotEqual( parsing.getTime( pd.DataFrame(data = {"Time":["2014-07-26 20:41:17+00:00"]}).iloc[0] ), 1406419271 )

        # Checking Incorrect Data Type
        with self.assertRaises(TypeError):
            parsing.getTime( pd.DataFrame(data = {"Time":[1.4]}).iloc[0] )
            parsing.getTime( pd.DataFrame(data = {"Time":[1]}).iloc[0] )
            parsing.getTime( pd.DataFrame(data = {"Time":["2014-07-26 20:41:39+000"]}).iloc[0] )
            parsing.getTime( pd.DataFrame(data = {"Time":["2014-07-26 20:41:39+00:0:0"]}).iloc[0] )
            parsing.getTime( pd.DataFrame(data = {"Time":["2014-07-2620:41:39+00:00"]}).iloc[0] )
            parsing.getTime( pd.DataFrame(data = {"Time":["2014-07-26 20:41:39+00:00"]}).iloc[0] )
            parsing.getTime( pd.DataFrame(data = {"Time":["2014-07-1-26 20:41:39+00:00"]}).iloc[0] )
            parsing.getTime( pd.DataFrame(data = {"Time":["2014-07-1 20:12:41:39+00:00"]}).iloc[0] )
        
        # Checking Incorrect Values
        with self.assertRaises(ValueError):
            parsing.getTime( pd.DataFrame(data = {"Time":["1969-07-26 20:41:17+00:00"]}).iloc[0] )
            parsing.getTime( pd.DataFrame(data = {"Time":["2014-25-26 20:41:17+00:00"]}).iloc[0] )
            parsing.getTime( pd.DataFrame(data = {"Time":["2014-00-26 20:41:17+00:00"]}).iloc[0] )
            parsing.getTime( pd.DataFrame(data = {"Time":["2014-07-45 20:41:17+00:00"]}).iloc[0] )
            parsing.getTime( pd.DataFrame(data = {"Time":["2014-02-29 20:41:17+00:00"]}).iloc[0] )
            parsing.getTime( pd.DataFrame(data = {"Time":["2014-02-30 20:41:17+00:00"]}).iloc[0] )
            parsing.getTime( pd.DataFrame(data = {"Time":["2014-07-00 20:41:17+00:00"]}).iloc[0] )
            parsing.getTime( pd.DataFrame(data = {"Time":["2014-06-31 20:41:17+00:00"]}).iloc[0] )
            parsing.getTime( pd.DataFrame(data = {"Time":["2012-02-30 20:41:17+00:00"]}).iloc[0] )
            parsing.getTime( pd.DataFrame(data = {"Time":["2014-07-30 24:41:17+00:00"]}).iloc[0] )
            parsing.getTime( pd.DataFrame(data = {"Time":["2014-07-30 20:62:17+00:00"]}).iloc[0] )
            parsing.getTime( pd.DataFrame(data = {"Time":["2014-07-30 20:41:97+00:00"]}).iloc[0] )
            parsing.getTime( pd.DataFrame(data = {"Time":["2014-07-30 20:41:17+15:00"]}).iloc[0] )
            parsing.getTime( pd.DataFrame(data = {"Time":["2014-07-30 -20:41:17+00:10"]}).iloc[0] )
            parsing.getTime( pd.DataFrame(data = {"Time":["2014-07-30 00:-41:17+00:00"]}).iloc[0] )
            parsing.getTime( pd.DataFrame(data = {"Time":["2014-07-30 00:41:17+00:00"]}).iloc[0] )
            parsing.getTime( pd.DataFrame(data = {"Time":["2014-07-30 00:41:-17+00:00"]}).iloc[0] )
            parsing.getTime( pd.DataFrame(data = {"Time":["2014-07-30 00:61:17+00:00"]}).iloc[0] )
            parsing.getTime( pd.DataFrame(data = {"Time":["2014-07-30 00:41:67+00:00"]}).iloc[0] )

            parsing.getLatitude( pd.DataFrame(data = {"time":["2014-07-26 20:41:17+00:00"]}).iloc[0] )  # Checking column names





if __name__ == "__main__":
    unittest.main()

