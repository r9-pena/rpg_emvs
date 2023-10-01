import rospy
import rosbag
from dvs_msgs.msg import Event,EventArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
import pandas as pd

# Path to text files
events_txt = 'events.txt' 
ground_txt = 'groundtruth.txt'

# Import data into dataframe
df_events = pd.read_csv(events_txt,header=None, sep=" ")
df_ground = pd.read_csv(ground_txt,header=None, sep=" ")
# print(df_ground)
# timestamp = rospy.Time.from_sec(df[0][:])
# print(timestamp)

with rosbag.Bag('output.bag', 'w') as bag:
    event_array_msg = EventArray()
    for row in range(df_events.shape[0]):
        event_timestamp = rospy.Time.from_sec(df_events[0][row])
        event_msg = Event()
        event_msg.ts = event_timestamp
        event_msg.x = df_events[1][row]
        event_msg.y = df_events[2][row]
        event_msg.polarity = df_events[3][row]

        event_array_msg.events.append(event_msg)

    event_array_msg.header.seq = 10000
    event_array_msg.header.stamp.secs = 0
    event_array_msg.header.stamp.nsecs = 0
    event_array_msg.header.frame_id = ''
    event_array_msg.height = 180
    event_array_msg.width = 240
    bag.write("/dvs/events", event_array_msg, event_timestamp)
    

    seq = 40606
    for row in range(661):
        event_timestamp = rospy.Time.from_sec(df_events[0][row])
        camera_msg = CameraInfo()
        camera_msg.header.seq = seq
        camera_msg.header.stamp = event_timestamp
        camera_msg.header.frame_id = ''
        camera_msg.height = 180
        camera_msg.width = 240
        camera_msg.distortion_model = "plumb_bob"
        # k1, k2, p1, p2, k3
        camera_msg.D = [-0.1385927674081299, 0.09337366641919795, -0.0003355869875320301, 0.0001737201582276446, 0.0]
        # fx, , cx, , fy, , cy, , , 1.0
        camera_msg.K = [335.4194629584808, 0.0, 129.9246633794451, 0.0, 335.3529356120773, 99.18643034473205, 0.0, 0.0, 1.0]
        # Identity matrix
        camera_msg.R =[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        # camera_msg.P = [328.3079223632812, 0.0, 129.7166999252022, 0.0, 0.0, 330.1483459472656, 98.78069942616821, 0.0, 0.0, 0.0, 1.0, 0.0]
        camera_msg.P = [335.4194629584808, 0.0, 129.9246633794451, 0.0, 0.0, 335.3529356120773, 99.18643034473205, 0.0, 0.0, 0.0, 1.0, 0.0]
        camera_msg.binning_x = 0
        camera_msg.binning_y = 0
        camera_msg.roi.x_offset= 0
        camera_msg.roi.y_offset= 0
        camera_msg.roi.height= 0
        camera_msg.roi.width= 0
        camera_msg.roi.do_rectify= False
        bag.write("/dvs/camera_info", camera_msg, event_timestamp)

        
    for row in range(df_ground.shape[0]):
        poses_timestamp = rospy.Time.from_sec(df_ground[0][row])
        poses_msg = PoseStamped()
        poses_msg.header.stamp = poses_timestamp
        poses_msg.pose.position.x = df_ground[1][row]
        poses_msg.pose.position.y = df_ground[2][row]
        poses_msg.pose.position.z = df_ground[3][row]
        poses_msg.pose.orientation.x = df_ground[4][row]
        poses_msg.pose.orientation.y = df_ground[5][row]
        poses_msg.pose.orientation.z = df_ground[6][row]
        poses_msg.pose.orientation.w = df_ground[7][row]
        bag.write('/optitrack/davis', poses_msg, poses_timestamp)
