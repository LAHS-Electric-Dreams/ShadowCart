# This file contains the class structure for OUR lidar class. Obviously, it will import the RPLIDAR class as well. 

# The final output of this class is a dictionary with bucket names and average k-minimum
# distances. These are all parameters that can be set at the initialization of the Lidar class.

# Import the original RPLidar class. 
from rplidar import RPLidar, RPLidarException
from time import time


class Lidar(object):
    def __init__(self, front_angle, num_mins_to_average, usb_address, scan_speed, zero_angle):
        self.front_angle = front_angle
        self.num_mins_to_average = num_mins_to_average
        self.usb_address = usb_address
        self.zero_angle = zero_angle

        # Default speed is what the LiDAR spins at when it is idle and not collecting
        # data; it sins at scan for one data collection.
        self.scan_speed = scan_speed

        # Set this to null for now, initialize it later as when one initializes an
        # RPLidar, it can immediately start the motor.
        self.rplidar = None

    def start_up_rplidar(self):
        # This METHOD only starts up the LiDAR and sets it to a low default speed.
        # Note that rplidar.py was changed in order to allow for entering the default speed at init.
        self.rplidar = RPLidar(self.usb_address, self.scan_speed)

    def get_raw_single_scan(self):
        # Returns a single LiDAR scan as a dictionary of angles and distances.
        # {angle: distance}

        # This MUST BE THE SECOND SCAN TAKEN because the first scan taken tends to
        # exclude a large portion of the unit circle.
        # Access 2nd scan (or the last one if there are less than 2 so far)

        #  This for loop is an ugly solution to iterating over the received scans, but it seems to work for now.
        for i, raw_scan in enumerate(self.rplidar.iter_scans()):
            if i > 1:
                break

        # Convert the raw array of tuples: (quality, angle, distance)
        # to a nicely formatted dictionary: {angle: distance}
        scan = {}
        for quality, angle, distance in raw_scan:
            scan[angle] = distance

        return scan

    def get_bucket_scan(self):
        # Considering only angles within "front_angle" of "zero_angle",
        # finds the average of the N closest distance readings on either side.
        # N is defined by "num_mins_to_average"
        # Returns an array: [left_mean, right_mean]

        # This function better be EXTREMELY efficient or else the car won't work ;) – Ravi
        # We should name these variables better. – Darryl

        # Get the latest scan.
        scan = self.get_raw_single_scan()
        print(scan)

        # Define lists of distances - one for angles to the left, and one for angles to the right
        left_distances = []
        right_distances = []

        # For every scanned angle within "front_angle" of "zero_angle",
        # record the distance at that angle into the appropriate list.
        for angle, distance in scan.items():
            calibrated_angle = angle - self.zero_angle
            if -self.front_angle < calibrated_angle < 0:
                left_distances.append(distance)
            elif 0 < calibrated_angle < self.front_angle:
                right_distances.append(distance)

        # Sort the distances by magnitude.
        left_distances.sort()
        right_distances.sort()

        # Find the mean of the closest N distances, where N is "num_mins_to_average"
        n = self.num_mins_to_average
        left_mean = sum(left_distances[0:n]) / n
        right_mean = sum(right_distances[0:n]) / n

        # Return the left and right means as an array.
        return [left_mean, right_mean]

    def get_bucket_scan_with_catching(self):
        # We have to declare scan here since otherwise it will be a local variable, making it useless.
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

    # This will return a tuple containing information about the detection
    # of objects to the left, right, front, or back. It is meant to be a very
    # simple situational awareness algorithm that can be tested easily.
    # ** Return Format: (front, back, left, right)
    # ** Example: No objects within the radius -> (false, false, false, false)
    # ** Example: Object to the left -> (false, false, true, false)
    # ** Example: Object to the right -> (false, false, false, true)
    def simple_obstacle_detect(self, range_radius):

        # Declare variables for direction detection
        front = False
        left = False
        right = False
        back = False

        # Run a scan (This may changed at a later date)
        scan = self.get_raw_single_scan()

        # Run through the scan by angle
        for angle, dist in scan:

            # Front Section (Part 1)
            if angle <= 30:
                if dist <= range_radius:
                    front = True
                print("We're less than 30º!")

            # Right Section
            elif angle <= 120:
                if dist <= range_radius:
                    right = True
                print("We're less than 120º!")

            # Back Section
            elif angle <= 210:
                if dist <= range_radius:
                    back = True
                print("We're less than 210º!")

            # Left Section
            elif angle <= 300:
                if dist <= range_radius:
                    left = True
                print("We're less than 300º!")

            # Front Section (Part 2)
            else:  # The other half of the "front" section
                if dist <= range_radius:
                    front = True

        return {front, back, left, right}

    def object_detect(self, range_radius, similar_by):
        scan = self.get_raw_single_scan()

        found_objects = []  # Format: (Start Angle, End Angle, Average Distance)

        in_object = False
        start_angle = -1
        distances = []
        for angle, dist in enumerate(scan):
            if dist < range_radius and (scan[angle - 1] > angle - similar_by or scan[angle + 1] < angle + similar_by):
                if not in_object:
                    start_angle = scan[angle - 1]
                else:
                    distances.append(dist)
            elif in_object:  # Found the end of the object
                in_object = False
                found_objects.append((start_angle, scan[angle - 1], calc_mean(distances)))

        return found_objects


# Calculate the average number for a list
def calc_mean(nums):
    total = 0
    for i in nums:
        total += i
    return total / nums.count


lidar1 = Lidar(20, 5, '/dev/ttyUSB0', 250, 100)
lidar1.start_up_rplidar()

start = time()
for i in range(0, 1000):
    print(lidar1.get_bucket_scan_with_catching())
# lidar1.rplidar.reset()

# Test the simple obstacle detection
testAware = lidar1.simple_obstacle_detect(5)
print("Alert Front: %s \nAlert Back: %s \nAlert Left: %s \n Alert Right: %s",
      str(testAware[0]), str(testAware[1]), str(testAware[2]), str(testAware[3]))

end = time()

print(end - start)
