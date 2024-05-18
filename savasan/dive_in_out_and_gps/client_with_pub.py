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
        "IHA_yonelme": imu_msg.orientation.z,
        "IHA_dikilme": imu_msg.orientation.y,
        "IHA_yatis": imu_msg.orientation.x,
        "IHA_hiz": calculate_speed(speed_sub.twist.linear.x, speed_sub.twist.linear.y, speed_sub.twist.linear.z),
        "IHA_batarya": int(battery_msg.percentage * 100),
        "IHA_otonom": mode_guided(state_sub.guided)
    }
    server_url = 'http://192.168.1.45:5000/update_data' 

    response = requests.post(server_url, json=data_dict)

    if response.status_code == 200:
        print(response.status_code)
        print(response.json())
        parsed_data = response.json().get('konumBilgileri', []) #
    
        for data in parsed_data: # iterate parsed data and publish it
            publish_parsed_data(data)

        if len(parsed_data) == 0: #reset index (if needed
            i = 0

    else:
        print(f"Hata: {response.status_code}")

        

def publish_parsed_data(parsed_data):
    pub = rospy.Publisher('plane_coordinates_topic', PlaneControl, queue_size=10)
    plane_data = PlaneControl()
    upcomin_data = {
        "IHA_boylam": data_dict['IHA_boylam'],
        "IHA_dikilme": data_dict['IHA_dikilme'],
        "IHA_enlem": data_dict['IHA_enlem'],
        "IHA_hiz": data_dict['IHA_hiz'],
        "IHA_irtifa": data_dict['IHA_irtifa'],
        "IHA_otonom": data_dict['IHA_otonom'],
        "IHA_yatis": data_dict['IHA_yatis'],
        "IHA_yonelme": data_dict['IHA_yonelme'],
        "takim_numarasi": data_dict['takim_numarasi']
    }

    for key, value in parsed_data.items(): # update with upcoming data
        setattr(plane_data, key, value)

    for key, value in upcomin_data.items(): 
        if getattr(plane_data, key) is None: # if theres missing val fill it
            setattr(plane_data, key, value)
            
    pub.publish(plane_data)



       

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
   
   
