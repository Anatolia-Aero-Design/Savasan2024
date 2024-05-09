import math
import rospy
import numpy as np
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
# draft for rocket guidance systems 

lat_plane = 0.0
lon_plane = 0.0
alt_plane = 0.0

linear_acceleration_x = 0.0
linear_acceleration_y = 0.0
linear_acceleration_z = 0.0

linear_velocity_x = 0.0
linear_velocity_y = 0.0
linear_velocity_z = 0.0

class InertialNavigation:
    def __init__(self, initial_position, initial_velocity, initial_acceleration):
        self.position = np.array(initial_position)
        self.velocity = np.array(initial_velocity)
        self.acceleration = np.array(initial_acceleration)

    def update_position(self, time_interval):
        # Using kinematic equations to update position
        self.position += self.velocity * time_interval + 0.5 * self.acceleration * (time_interval ** 2)

    def update_velocity(self, time_interval):
        # Using kinematic equations to update velocity
        self.velocity += self.acceleration * time_interval

# Function to convert geodetic coordinates to Cartesian coordinates
def geodetic_to_cartesian(lat, lon, alt):
    # Radius of the Earth (in meters)
    EARTH_RADIUS = 6371000  # Approximate value for mean Earth radius
    
    # Convert latitude and longitude from degrees to radians
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)

    # Convert altitude from meters to kilometers
    alt_km = alt / 1000

    # Calculate Cartesian coordinates
    x = (EARTH_RADIUS + alt_km) * math.cos(lat_rad) * math.cos(lon_rad)
    y = (EARTH_RADIUS + alt_km) * math.cos(lat_rad) * math.sin(lon_rad)
    z = (EARTH_RADIUS + alt_km) * math.sin(lat_rad)

    return [x, y, z]

def callback(data):
    global lat_plane
    global lon_plane
    global alt_plane
    lat_plane = data.latitude
    lon_plane = data.longitude
    alt_plane = data.altitude
    
def callback2(data):
    global linear_velocity_x 
    global linear_velocity_y 
    global linear_velocity_z 
    # Extract linear velocity values from the message
    linear_velocity_x = data.twist.linear.x
    linear_velocity_y = data.twist.linear.y
    linear_velocity_z = data.twist.linear.z
    
def callback3(data):
    global linear_acceleration_x 
    global linear_acceleration_y 
    global linear_acceleration_z
    # Extract linear acceleration values from the message
    linear_acceleration_x = data.linear_acceleration.x
    linear_acceleration_y = data.linear_acceleration.y
    linear_acceleration_z = data.linear_acceleration.z

if __name__ == "__main__":
    rospy.init_node('INS_node')
    sub_coordinates = rospy.Subscriber('/plane1/mavros/global_position/global',NavSatFix,callback)
    sub_vel = rospy.Subscriber('/plane1/mavros/local_position/velocity_local',TwistStamped,callback2)
    sub_acc  = rospy.Subscriber('/plane1/mavros/imu/data',Imu,callback3)
    rospy.sleep(2)  # Wait for topics to populate with initial values
    initial_lat = lat_plane
    initial_lon = lon_plane
    initial_alt = alt_plane
    initial_vx = linear_velocity_x
    initial_vy = linear_velocity_y
    initial_vz = linear_velocity_z
    initial_ax = linear_acceleration_x
    initial_ay = linear_acceleration_y
    initial_az = linear_acceleration_z

    # Convert geodetic coordinates to Cartesian coordinates
    initial_position = geodetic_to_cartesian(initial_lat, initial_lon, initial_alt)

    # Initial velocity and acceleration
    initial_velocity = [initial_vx, initial_vy, initial_vz]  # Initial velocity [vx, vy, vz] in m/s
    initial_acceleration = [initial_ax, initial_ay, initial_az]  # Initial acceleration [ax, ay, az] in m/s^2 (assuming downward acceleration due to gravity)

    # Time interval for each iteration (in seconds)
    time_interval = 1  # Assuming updates every second

    # Initialize inertial navigation system
    inav_system = InertialNavigation(initial_position, initial_velocity, initial_acceleration)

    # Update position and velocity for 10 iterations
    for i in range(10):
        inav_system.update_position(time_interval)
        inav_system.update_velocity(time_interval)
        print(f"Iteration {i+1}: Position = {inav_system.position}, Velocity = {inav_system.velocity}")
