import rosbag
from scipy.spatial.transform import Rotation as R
import numpy as np
import argparse
import os
import pandas as pd
from fastparquet import write

def tf_to_mat(transform):
    r = R.from_quat([transform.transform.rotation.x, transform.transform.rotation.y , transform.transform.rotation.z, transform.transform.rotation.w])
    t = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
    mat = np.eye(4)
    mat[:3,:3] = r.as_matrix()
    mat[:3,3] = t
    return mat

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-r", "--rosbag_path", type=str, default="~/Data")
    parser.add_argument("-o", "--output_directory", type=str, default="~/Data/")
    args = parser.parse_args()
    
    ros_timestamps = []
    ros_odom_x = []
    ros_odom_y = []
    ros_odom_yaw = []
    if args.rosbag_path is not None:
        bag = rosbag.Bag(args.rosbag_path, 'r')
        tf_odom_base = None
        tf_map_odom = None
        tf_map_base = None
        ros_timestamp = None
        for topic, msg, t in bag.read_messages(topics=['/tf']):
            if topic == '/tf':
                for transform in msg.transforms:
                    if transform.child_frame_id == 'base_footprint':
                        tf_odom_base = tf_to_mat(transform)
                        if ros_timestamp is None:
                            ros_timestamp = transform.header.stamp.to_sec()
                    elif transform.child_frame_id == 'odom':
                        tf_map_odom = tf_to_mat(transform)
                        if ros_timestamp is None:
                            ros_timestamp = transform.header.stamp.to_sec()
            else:
                print("Unknown topic")
            if tf_odom_base is not None and tf_map_odom is not None:
                tf_map_base = np.matmul(tf_map_odom, tf_odom_base)
                ros_odom_x.append(tf_map_base[0,3])
                ros_odom_y.append(tf_map_base[1,3])
                # ros_odom_yaw.append(np.arctan2(tf_map_base[1,0], tf_map_base[0,0]))
                ros_timestamps.append(ros_timestamp)
                tf_odom_base = None
                tf_map_odom = None
                tf_map_base = None
                ros_timestamp = None
        bag.close()
    else:
        print("No rosbag path provided")

    
    outdir = os.path.join(args.output_directory)
    os.makedirs(outdir, exist_ok=True)

    df = pd.DataFrame({'t': ros_timestamps, 'x': ros_odom_x, 'y': ros_odom_y}) # , 'p': ros_odom_yaw})
    write(f"{outdir}/{os.path.basename(args.rosbag_path).split('.')[0]}.parquet", df)