import rospy
from mavros_msgs.msg import PlaneControl
import math
from mavros_msgs.srv import CommandInt
from sensor_msgs.msg import NavSatFix

#*************follow nearest plane with the data coming from the server******************

our_latitude = 0.0
our_longitude = 0.0
our_altitude = 0.0

def send_goto_position(target_latitude, target_longitude, target_altitude): # for sending command to our plane
    rospy.wait_for_service('/plane1/mavros/cmd/command_int')
    try:
        command_int = rospy.ServiceProxy('/plane1/mavros/cmd/command_int', CommandInt)
        response = command_int(
            frame = 3,  
            command = 192,  
            current = 2,  
            autocontinue = 0, 
            param1 = 30,  
            param2 = 0,  
            param3 = 0,
            param4 = 0, 
            x = int(target_latitude * 1e7),  
            y = int(target_longitude * 1e7),
            z = target_altitude  
        )
        rospy.loginfo("Position command sent")
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)
        
# global variables for storing nearest distance and team
nearest_distance = float('inf')
nearest_team = None
latest_coordinates = None

def callback(data):
    global nearest_distance, nearest_team, latest_coordinates
    
    distances = {}
    rospy.loginfo("THEIR Lat %f, Lon %f, Alt %f", data.IHA_enlem, data.IHA_boylam, data.IHA_irtifa)

    # calculate distance for the received plane
    distance = haversine(data.IHA_enlem, data.IHA_boylam, data.IHA_irtifa, our_latitude, our_longitude, our_altitude, data.takim_numarasi, distances)
    
    # update nearest distance and team if the calculated distance is less than current nearest distance
    if distance < nearest_distance:
        nearest_distance = distance
        nearest_team = data.takim_numarasi
        latest_coordinates = (data.IHA_enlem, data.IHA_boylam, data.IHA_irtifa)
        
        # send the our plane to the latest coordinates of the nearest plane
        send_goto_position(data.IHA_enlem, data.IHA_boylam, data.IHA_irtifa)
        
    else:
        # If the received plane is not closer than the nearest plane, update the nearest distance
        # This ensures that the nearest distance stays updated even if the nearest plane doesn't change
        nearest_distance = distance

    # log the nearest plane and distance
    print("Nearest plane number:", nearest_team)
    print("Nearest distance:", nearest_distance)

    




    
# func for calculating distance with us and planes on the server
def haversine(lat1, lon1, alt1, lat2, lon2, alt2, takim_no, distances_dict):
    """
    Calculate the great-circle distance between two points
    on the Earth's surface using the Haversine formula.
    """
    # Convert latitude and longitude from degrees to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    # Earth radius in meters
    earth_radius = 6371000  # approximate radius of Earth in meters

    # Haversine formula
    d_lat = lat2_rad - lat1_rad
    d_lon = lon2_rad - lon1_rad
    a = (math.sin(d_lat / 2) ** 2) + math.cos(lat1_rad) * math.cos(lat2_rad) * (math.sin(d_lon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # Distance in meters
    distance = earth_radius * c

    # Adding altitude difference
    distance += abs(alt2 - alt1)

    # Update or add distance for the team number
    if takim_no in distances_dict:
        distances_dict[takim_no] += distance
    else:
        distances_dict[takim_no] = distance
        
    return distance
   
def callback2(data): # callback for our coordinates
    global our_position
    our_position = data
    global our_latitude
    global our_longitude
    global our_altitude
    rospy.loginfo("OUR Lat %s, Lon %s, Alt %s", data.latitude, data.longitude, data.altitude)
    our_latitude = data.latitude
    our_longitude = data.longitude
    our_altitude = data.altitude

 

if __name__ == '__main__':
    rospy.init_node('nearest_plane_node')
    rospy.Subscriber('/plane_coordinates_topic', PlaneControl, callback) # getting coordinates of other planes
    rospy.Subscriber('/plane1/mavros/global_position/global', NavSatFix, callback2) # getting our coordinates for calculations
    rospy.spin()

