import rospy
import rosbag
from dvs_msgs.msg import Event,EventArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
import pandas as pd
import numpy as np

def evimoToRosbag():
    # D_file = np.load("./evimo/D.npy")
    # K_file = np.load("./evimo/K.npy")
    polar = np.load("./evimo/dataset_events_p.npy")
    pixels = np.load("./evimo/dataset_events_xy.npy")
    event_ts = np.load("./evimo/dataset_events_t.npy")
    meta = np.load("./evimo/meta.npy",allow_pickle=True).item()

    with rosbag.Bag('evimo.bag', 'w') as bag:
        event_array_msg = EventArray()
        for row in range(pixels.shape[0]):
            event_timestamp = rospy.Time.from_sec(event_ts[row])
            event_msg = Event()
            event_msg.ts = event_timestamp
            event_msg.x = pixels[row][0]
            event_msg.y = pixels[row][1]
            event_msg.polarity = polar[row]

            event_array_msg.events.append(event_msg)

        event_array_msg.header.seq = 10000
        event_array_msg.header.stamp.secs = 0
        event_array_msg.header.stamp.nsecs = 0
        event_array_msg.header.frame_id = ''
        event_array_msg.height = 180    #Needs to be changed
        event_array_msg.width = 240     #Needs to be changed
        bag.write("/dvs/events", event_array_msg, event_timestamp)

        print('Part 1 complete')
        
        seq = 40606
        for row in range(661):
            event_timestamp = rospy.Time.from_sec(row['ts'])
            camera_msg = CameraInfo()
            camera_msg.header.seq = seq
            camera_msg.header.stamp = event_timestamp
            camera_msg.header.frame_id = ''
            camera_msg.height = meta['meta']['res_x']                   #Done
            camera_msg.width = meta['meta']['res_y']                    #Done
            camera_msg.distortion_model = meta['meta']['dist_model']    #Done
            # k1, k2, p1, p2, k3
            camera_msg.D = [meta['meta']['k1'], meta['meta']['k2'], meta['meta']['p1'], meta['meta']['p2'], meta['meta']['k3']]
            # fx, , cx, , fy, , cy, , , 1.0
            camera_msg.K = [meta['meta']['fx'], 0.0, meta['meta']['cx'], 0.0, meta['meta']['fy'], meta['meta']['cy'], 0.0, 0.0, 1.0]
            # Identity matrix
            camera_msg.R =[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            camera_msg.P = [meta['meta']['fx'], 0.0, meta['meta']['cx'], 0.0, 0.0, meta['meta']['fy'], meta['meta']['cy'], 0.0, 0.0, 0.0, 1.0, 0.0]
            camera_msg.binning_x = 0
            camera_msg.binning_y = 0
            camera_msg.roi.x_offset= 0
            camera_msg.roi.y_offset= 0
            camera_msg.roi.height= 0
            camera_msg.roi.width= 0
            camera_msg.roi.do_rectify= False
            bag.write("/dvs/camera_info", camera_msg, event_timestamp)

        print('Part 2 complete')
            
        for row in meta['frames']:
            poses_timestamp = rospy.Time.from_sec(row['ts'])
            poses_msg = PoseStamped()
            poses_msg.header.stamp = poses_timestamp
            poses_msg.pose.position.x = row['cam']['pos']['t']['x']
            poses_msg.pose.position.y = row['cam']['pos']['t']['y']
            poses_msg.pose.position.z = row['cam']['pos']['q']['z']
            poses_msg.pose.orientation.x = row['cam']['pos']['q']['x']
            poses_msg.pose.orientation.y = row['cam']['pos']['q']['y']
            poses_msg.pose.orientation.z = row['cam']['pos']['q']['z']
            poses_msg.pose.orientation.w = row['cam']['pos']['q']['w']
            bag.write('/optitrack/davis', poses_msg, poses_timestamp)
    
    return 0

evimoToRosbag()