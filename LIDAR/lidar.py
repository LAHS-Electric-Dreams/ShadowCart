#  This file contains the class structure for OUR lidar class. Obviously, it will import the RPLIDAR class as well. 

#  The final output of this class is a dictionary with bucket names and average k-minimum distances. These are all parameters that can be set at the initializationof the Lidar class. 

#  Import the original RPLidar class. 
from rplidar import RPLidar, RPLidarException
import numpy as np
from time import sleep, time

class Lidar(object):
    def __init__(self, front_angle, num_mins_to_average, usb_address, scan_speed, zero_angle):
        self.front_angle = front_angle
        self.num_mins_to_average = num_mins_to_average
        self.usb_address = usb_address
        self.zero_angle = zero_angle
        self.scan_speed = scan_speed                            #  Default speed is what teh LiDAR spins at when it is idle and not colelcting data; it sins at scan for one data collection. 

        self.rplidar = None              #  Set this to null for now, initialize it later as when one initializes an RPLidar, it can immediately start the motor.

    def start_up_rplidar(self):
        #  This METHOD only starts up the LiDAR and sets it to a low default speed.
        self.rplidar = RPLidar(self.usb_address, self.scan_speed)                  #  Note that rplidar.py was changed in order to allow for entering the default speed at init.

        #  LOL XD: This function is only one line :smile:

    def get_raw_single_scan(self):
        #  This function has to return the raw scan data as a dicationary. This MUST BE THE SECOND SCAN TAKEN because the first scan taken tends to exclude a large portion of the unit circle. 
        for i, scan in enumerate(self.rplidar.iter_scans()):
            if i > 1:
                break

        #  Now we have the correct scan. :smile:.

        #  Iterate over the array of tuples to extract each angle and distance measurement.
        return scan

    def get_bucket_scan(self):
        #  This function better be VVVVVEEEEERRRRRYYYYY efficient or else the car won't work ;(

        #  Here, we need to return the average of teh minimum nun_mins_to_average scan valuesto the left and to the right of 0 degrees with a range of front_angle. This will be returned as an array.

        scan = self.get_raw_single_scan()
        print(scan)

        bucket_left_data = []
        bucket_right_data = []

        for measurement in scan:
            if (measurement[1] < self.zero_angle and measurement[1] > (self.zero_angle - self.front_angle)):
                bucket_left_data.append(measurement[2])

        for measurement in scan:
            if (measurement[1] > self.zero_angle and measurement[1] < (self.zero_angle + self.front_angle)):
                bucket_right_data.append(measurement[2])

        left_sorted = list(np.sort(bucket_left_data))
        right_sorted  = list(np.sort(bucket_right_data))

        #  Now, we just need to take the first num_mins_to_average elements of the above two arrays.
        minimum_left_elements = left_sorted[0:self.num_mins_to_average]
        minimum_right_elements = right_sorted[0:self.num_mins_to_average]

        left_minimum_mean = float(np.mean(minimum_left_elements))
        right_minimum_mean = float(np.mean(minimum_right_elements))

        return [left_minimum_mean, right_minimum_mean]

    def get_bucket_scan_with_catching(self):
        #  We have to declare scan here since otherwise it will be a local variable, making it useless. 
        scan = None
        
        while True:
            try:
                scan = self.get_bucket_scan()
            except RPLidarException:
                print("Successfully caught an exception")
                self.rplidar.clear_input()
                self.rplidar.disconnect()
                self.rplidar.connect()
                continue

            break
        
        return scan
            


lidar1 = Lidar(20, 5, '/dev/ttyUSB0', 250, 100)
lidar1.start_up_rplidar()

start = time()
for i in range(0, 1000):
    print(lidar1.get_bucket_scan_with_catching())
    #lidar1.rplidar.reset()
end = time()

print(end-start)
