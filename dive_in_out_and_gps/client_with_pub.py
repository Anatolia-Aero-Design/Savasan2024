import rospy
from sensor_msgs.msg import Imu, BatteryState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.msg import PlaneControl
import requests
import time
import json
from utils import quaternion_to_euler, calculate_speed, mode_guided


def callback(imu_msg, battery_msg, rel_altitude_msg, position_msg, speed_sub, state_sub):
    global data_dict 
    data_dict = {
        "takim_numarasi" : 1,
        "IHA_enlem": position_msg.latitude,
        "IHA_boylam": position_msg.longitude,
        "IHA_irtifa": rel_altitude_msg.data,
        "IHA_yonelme": imu_msg.orientation.x,
        "IHA_dikilme": imu_msg.orientation.y,
        "IHA_yatis": imu_msg.orientation.z,
        "IHA_hiz": calculate_speed(speed_sub.twist.linear.x, speed_sub.twist.linear.y, speed_sub.twist.linear.z),
        "IHA_batarya": int(battery_msg.percentage * 100),
        "IHA_otonom": mode_guided(state_sub.guided)
    }
    server_url = 'http://192.168.1.45:5000/update_data' 

    response = requests.post(server_url, json=data_dict)

    if response.status_code == 200:
        print(response.status_code)
        print(response.json())
        parsed_data = response.json() # Assuming the server returns parsed data
        for i in range(len(parsed_data['konumBilgileri'])):
            for j in range(len(parsed_data['konumBilgileri'][i])):
                data_to_be_published = parsed_data.get(j, parsed_data['konumBilgileri'][i])
                
            publish_parsed_data(data_to_be_published)  # Pass parsed_data and takim_numarasi
            if i == len(parsed_data['konumBilgileri']):
                i = 0
 

    
    else:
        print(f"Hata: {response.status_code}")
        

def publish_parsed_data(parsed_data):
    # Create ROS publisher for PlaneCoordinates message
    pub = rospy.Publisher('plane_coordinates_topic', PlaneControl, queue_size=10)
    
    # Create PlaneControl message
    plane_data_msg = PlaneControl()


    plane_data_msg.IHA_boylam = parsed_data.get('IHA_boylam', data_dict['IHA_boylam'])
    plane_data_msg.IHA_dikilme = parsed_data.get('IHA_dikilme', data_dict['IHA_dikilme'])
    plane_data_msg.IHA_enlem = parsed_data.get('IHA_enlem', data_dict['IHA_enlem'])
    plane_data_msg.IHA_hiz = parsed_data.get('IHA_hiz', data_dict['IHA_hiz'])
    plane_data_msg.IHA_irtifa = parsed_data.get('IHA_irtifa', data_dict['IHA_irtifa'])
    plane_data_msg.IHA_otonom = parsed_data.get('IHA_otonom', data_dict['IHA_otonom'])
    plane_data_msg.IHA_yatis = parsed_data.get('IHA_yatis', data_dict['IHA_yatis'])
    plane_data_msg.IHA_yonelme = parsed_data.get('IHA_yonelme', data_dict['IHA_yonelme'])
    plane_data_msg.takim_numarasi = parsed_data.get('takim_numarasi', data_dict['takim_numarasi'])  # Assuming default value is 0 for int type

    # Publish the message
    pub.publish(plane_data_msg)


       

def synchronize_topics():
    rospy.init_node('sync_node', anonymous=True)

    imu_sub = Subscriber('/mavros/imu/data', Imu)
    battery_sub = Subscriber('/mavros/battery', BatteryState)
    rel_altitude_sub = Subscriber('/mavros/global_position/rel_alt', Float64)
    position_sub = Subscriber('/mavros/global_position/global', NavSatFix)
    speed_sub = Subscriber('/mavros/local_position/velocity_local', TwistStamped)
    state_sub = Subscriber('/mavros/state' , State)

    # ApproximateTimeSynchronizer to synchronize messages based on timestamps
    sync = ApproximateTimeSynchronizer(
        [imu_sub, battery_sub, rel_altitude_sub, position_sub, speed_sub, state_sub],
        queue_size=10,
        slop=0.1,  # Adjust this parameter based on your message timestamp tolerances
        allow_headerless=True
    )
    sync.registerCallback(callback)

    rospy.spin()


if __name__ == '__main__':
    synchronize_topics()
   
   
