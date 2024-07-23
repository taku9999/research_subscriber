#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from livox_ros_driver.msg import CustomMsg
import open3d as o3d
import numpy as np
from gps3 import gps3
import datetime
import time
import os
from multiprocessing import Process, Value, Array


DEVICE = o3d.core.Device("CUDA:0")
DTYPE = o3d.core.float32

LOG_DIRS = "/workspace/bind_data"


def init_cuda():
    print("=== [init_cuda] ===")
    print(" :: Start cuda access...")
    temp_time = time.time()

    pcd_dummy = o3d.t.geometry.PointCloud(DEVICE)
    pcd_dummy.point.positions = o3d.core.Tensor(np.empty((1, 3)), DTYPE, DEVICE)

    print(" :: Success! (time: {})".format(time.time() - temp_time))


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


def callback_odometry(message, args):
    ros_time = args[0]
    gps_time = args[1]
    gps_lat = args[2]
    gps_lon = args[3]
    filepass = args[4]

    timestamp = message.header.stamp # ROS時間
    position = message.pose.pose.position # 位置情報
    orientation = message.pose.pose.orientation # 姿勢情報

    # クォータニオンをオイラー角に変換
    euler = tf.transformations.euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))
    
    # 共有メモリにROS_timeを保存
    ros_time.value = float(str(timestamp.secs) + "." + str(timestamp.nsecs).zfill(9)[:3])

    print("=== Odometry data receive ===")
    print(" :: GPS情報: time={}, lat={}, lon={}".format(gps_time.value.decode('utf-8'), gps_lat.value, gps_lon.value))

    with open(filepass + "/odometry_log.csv", mode="a") as f:
        f.write(str(timestamp.secs) + "." + str(timestamp.nsecs).zfill(9)[:3])                                             # ROS時間（genpy.rostime.Time）
        f.write("," + str(position.x) + "," + str(position.y) + "," + str(position.z))                                     # 位置情報（float）
        f.write("," + str(orientation.x) + "," + str(orientation.y) + "," + str(orientation.z) + "," + str(orientation.w)) # 姿勢情報（float）
        f.write("," + str(euler[0]) + "," + str(euler[1]) + "," + str(euler[2]))                                           # オイラー変換後（float）
        f.write("," + gps_time.value.decode('utf-8') + "," + str(gps_lat.value) + "," + str(gps_lon.value))                # GPS情報（str, float）
        f.write("," + str(get_device_time("unix_time")) + "\r\n")                                                          # デバイス時間（float）


def callback_lidar(point_cloud, args):
    filepass = args[0]

    ros_time = point_cloud.header.stamp # ROS時間
    
    num_points = point_cloud.point_num # Pointの数

    # tensorを用いて点群をpcdに保存
    points = np.zeros((num_points, 3), dtype='float32')
    intensities = np.zeros((num_points, 1), dtype='float32')
    for i, point in enumerate(point_cloud.points):
        points[i, 0] = point.x
        points[i, 1] = point.y
        points[i, 2] = point.z
        intensities[i, 0] = np.float32(point.reflectivity)
    
    pcd_t = o3d.t.geometry.PointCloud(DEVICE)
    pcd_t.point.positions = o3d.core.Tensor(points)
    pcd_t.point.intensity = o3d.core.Tensor(intensities)

    o3d.t.io.write_point_cloud(filepass + "/" + str(ros_time.secs) + "-" + str(ros_time.nsecs).zfill(9)[:3] + "_" + get_device_time("conv_str_milli")[:-3] + ".pcd", pcd_t)


def process_ros(g_ros_time, g_gps_time, g_gps_lat, g_gps_lon, log_filepass, pcd_filepass):
    rospy.init_node('research_subscriber')

    rospy.Subscriber('/Odometry', Odometry , callback_odometry, (g_ros_time, g_gps_time, g_gps_lat, g_gps_lon, log_filepass))
    rospy.Subscriber('/livox/lidar', CustomMsg, callback_lidar, (pcd_filepass, ))

    rospy.spin()


def process_gps(g_ros_time, g_gps_time, g_gps_lat, g_gps_lon, log_filepass):
    try:
        gps_socket = gps3.GPSDSocket()
        data_stream = gps3.DataStream()
        gps_socket.connect()
        gps_socket.watch()
    except :
        print("Initialization Error")
    
    date_format = '%Y-%m-%d %H:%M:%S.%f'
    
    for new_data in gps_socket:
        if new_data:
            data_stream.unpack(new_data)
            gps_time = data_stream.TPV['time']
            gps_lat = data_stream.TPV['lat']
            gps_lon = data_stream.TPV['lon']

            # 時刻
            if gps_time == 'n/a':
                gps_time = 0.0
            else:
                tmp_time = str(gps_time).replace('T', ' ')[:-1]
                tmp_time = datetime.datetime.strptime(tmp_time, date_format)
                tmp_time = tmp_time + datetime.timedelta(hours=9)
                gps_time = tmp_time.strftime("%Y-%m-%d_%H-%M-%S")
                g_gps_time.value = bytes(gps_time, 'utf-8') # 共有メモリ(str)

            # 緯度＆経度
            if gps_lat == 'n/a' and gps_lon == 'n/a':
                gps_lat = 0.0
                gps_lon = 0.0
            else:
                g_gps_lat.value = gps_lat # 共有メモリ(float)
                g_gps_lon.value = gps_lon # 共有メモリ(float)
            
            # GPS情報とROS_Time情報をログ記録
            with open(log_filepass + "/gps_log.csv", mode="a") as f:
                f.write(str(g_ros_time.value))                                           # ROS時間 (共有メモリ)
                f.write("," + str(gps_time) + "," + str(gps_lat) + "," + str(gps_lon))   # GPS情報 (str, float)
                f.write("," + str(get_device_time("unix_time")) + "\r\n")                # デバイス時間 (float)


def main():
    # Logファイルパスの設定
    log_filepass = LOG_DIRS + "/" + str(get_device_time("conv_str"))
    os.makedirs(log_filepass)
    print("[Debug] Log file save to: {}".format(log_filepass))

    # PCDファイルパスの設定
    pcd_filepass = log_filepass + "/pcd_data"
    os.makedirs(pcd_filepass)
    print("[Debug] PCD file save to: {}".format(pcd_filepass))

    # CUDAの初期化
    init_cuda()

    # 共有メモリの設定
    m_ros_time = Value('d', 0.0)
    m_gps_time = Array('c', b"0000-00-00_00-00-00")
    m_gps_lat = Value('d', 0.0)
    m_gps_lon = Value('d', 0.0)

    # GPSプロセスの設定
    p_gps = Process(target=process_gps, args=(m_ros_time, m_gps_time, m_gps_lat, m_gps_lon, log_filepass))
    p_gps.start()

    # ROSプロセスの設定
    p_ros = Process(target=process_ros, args=(m_ros_time, m_gps_time, m_gps_lat, m_gps_lon, log_filepass, pcd_filepass))
    p_ros.start()



if __name__ == "__main__":
    main()