#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from livox_ros_driver.msg import CustomMsg
from sensor_msgs.msg import PointCloud2
import datetime
import time
import os
# from multiprocessing import Process, Value, Array


LOG_DIRS = "/workspace/bind_data"
LOG_FLAG = True
MSG_TYPE = 0 # [0]=PointCloud2, [1]=CustomMsg


def get_device_time(cmd):
    tmp_time = datetime.datetime.now()

    # datetime.datetime
    date_time = tmp_time + datetime.timedelta(hours=9)
    # float
    unix_time = date_time.timestamp()
    # convert(no-milliseconds)
    conv_str = date_time.strftime("%Y-%m-%d_%H-%M-%S")
    # convert(milliseconds)
    conv_str_milli = date_time.strftime("%Y-%m-%d_%H-%M-%S_%f")

    if cmd == "date_time":
        return date_time
    elif cmd == "unix_time":
        return unix_time
    elif cmd == "conv_str":
        return conv_str
    elif cmd == "conv_str_milli":
        return conv_str_milli


def callback_lidar(point_cloud, args):
    filepass = args[0]

    ros_time_raw = point_cloud.header.stamp # ROS時間
    jetson_time_unix = time.time() # Jetson時間
    
    print("\n========================================")
    ros_time_unix = float(str(ros_time_raw.secs) + "." + str(ros_time_raw.nsecs).zfill(9)[:3])
    ros_time_dt = datetime.datetime.utcfromtimestamp(ros_time_unix)
    ros_time_dt_jst = ros_time_dt + datetime.timedelta(hours=9)
    print("ROS Time: " + ros_time_dt_jst.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3])
    
    jetson_time_dt = datetime.datetime.utcfromtimestamp(jetson_time_unix)
    jetson_time_dt_jst = jetson_time_dt + datetime.timedelta(hours=9)
    print("Jetson Time: " + jetson_time_dt_jst.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3])
    
    if LOG_FLAG == True:
        with open(filepass + "/time_log.csv", mode="a") as f:
            f.write(str(jetson_time_unix))
            f.write("," + str(ros_time_unix) + "\r\n")


def process_ros(log_filepass):
    rospy.init_node("research_subscriber")

    if MSG_TYPE == 0:
        rospy.Subscriber("/livox/lidar", PointCloud2, callback_lidar, (log_filepass, ))
    elif MSG_TYPE == 1:
        rospy.Subscriber("/livox/lidar", CustomMsg, callback_lidar, (log_filepass, ))

    rospy.spin()


def main():
    # Logファイルパスの設定
    log_filepass = LOG_DIRS + "/" + str(get_device_time("conv_str"))
    os.makedirs(log_filepass)
    print("[Debug] Log file save to: {}".format(log_filepass))

    # ROSプロセスの設定
    process_ros(log_filepass)
    # p_ros = Process(target=process_ros, args=(log_filepass,))
    # p_ros.start()



if __name__ == "__main__":
    main()