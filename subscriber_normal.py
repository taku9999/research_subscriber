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

log_dirs= "/workspace/bind_data"

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

def odometry_callback(message, args):
    gps_time = args[0]
    gps_lat = args[1]
    gps_lon = args[2]
    filepass = args[3]

    timestamp = message.header.stamp # ROS時間
    position = message.pose.pose.position # 位置情報
    orientation = message.pose.pose.orientation # 姿勢情報

    # クォータニオンをオイラー角に変換
    euler = tf.transformations.euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))

    print("======= Odometry data receive =======")
    print("ROS時間: {}".format(timestamp))
    print("位置情報: x={}, y={}, z={}".format(position.x, position.y, position.z))
    print("姿勢情報: x={}, y={}, z={}, w={}".format(orientation.x, orientation.y, orientation.z, orientation.w))
    print("オイラー変換後: x={}, y={}, z={}".format(euler[0], euler[1], euler[2]))
    print("GPS情報: time={}, lat={}, lon={}".format(gps_time.value.decode('utf-8'), gps_lat.value, gps_lon.value))

    with open(filepass + "/odometry_log.csv", mode="a") as f:
        f.write(str(timestamp)) # ROS時間（genpy.rostime.Time）
        f.write("," + str(position.x) + "," + str(position.y) + "," + str(position.z)) # 位置情報（float）
        f.write("," + str(orientation.x) + "," + str(orientation.y) + "," + str(orientation.z) + "," + str(orientation.w)) # 姿勢情報（float）
        f.write("," + str(euler[0]) + "," + str(euler[1]) + "," + str(euler[2])) # オイラー変換後（float）
        f.write("," + gps_time.value.decode('utf-8') + "," + str(gps_lat.value) + "," + str(gps_lon.value)) # GPS情報（str, float）
        f.write("," + str(get_device_time("unix_time")) + "\r\n") # デバイス時間（float）

def imu_callback(message, args):
    gps_time = args[0]
    gps_lat = args[1]
    gps_lon = args[2]
    filepass = args[3]

    timestamp = message.header.stamp # ROS時間
    acceleration = message.linear_acceleration # 加速度
    angular = message.angular_velocity # 角速度

    # print("========== IMU data receive ==========")
    # print("ROS時間: {}".format(timestamp))
    # print("加速度: x={}, y={}, z={}".format(acceleration.x, acceleration.y, acceleration.z))
    # print("角速度: x={}, y={}, z={}".format(angular.x, angular.y, angular.z))
    # print("GPS情報: time={}, lat={}, lon={}".format(gps_time.value.decode('utf-8'), gps_lat.value, gps_lon.value))

    with open(filepass + "/imu_log.csv", mode="a") as f:
        f.write(str(timestamp)) # ROS時間（genpy.rostime.Time）
        f.write("," + str(acceleration.x) + "," + str(acceleration.y) + "," + str(acceleration.z)) # 加速度（float）
        f.write("," + str(angular.x) + "," + str(angular.y) + "," + str(angular.z)) # 角速度（float）
        f.write("," + gps_time.value.decode('utf-8') + "," + str(gps_lat.value) + "," + str(gps_lon.value)) # GPS情報（str, float）
        f.write("," + str(get_device_time("unix_time")) + "\r\n") # デバイス時間（float）

def lidar_callback(point_cloud, args):
    filepass = args[0]

    ros_time = point_cloud.header.stamp
    
    # ポイントクラウドデータの要素数を取得
    num_points = point_cloud.point_num

    # tensorを用いて点群をpcdに保存（反射強度あり）
    points = np.zeros((num_points, 3), dtype='float32')
    intensities = np.zeros((num_points, 1), dtype='float32')
    for i, point in enumerate(point_cloud.points):
        points[i, 0] = point.x
        points[i, 1] = point.y
        points[i, 2] = point.z
        intensities[i, 0] = np.float32(point.reflectivity)
    
    pcd_t = o3d.t.geometry.PointCloud()
    pcd_t.point.positions = o3d.core.Tensor(points)
    pcd_t.point.intensity = o3d.core.Tensor(intensities)
    print(pcd_t)
    o3d.t.io.write_point_cloud(filepass + "/" + str(ros_time) + "_" + get_device_time("conv_str_milli")[:-3] + ".pcd", pcd_t)

    # # tensorを用いずに点群をpcdに保存（反射強度なし）
    # points = np.zeros((num_points, 3))
    # for i, point in enumerate(point_cloud.points):
    #     points[i, 0] = point.x
    #     points[i, 1] = point.y
    #     points[i, 2] = point.z
    
    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(points)
    # o3d.io.write_point_cloud(filepass + "/" + str(ros_time) + "_" + get_device_time("conv_str_milli")[:-3] + ".pcd", pcd)

def ros_process(g_gps_time, g_gps_lat, g_gps_lon, log_filepass, pcd_filepass):
    rospy.init_node('position_sub')

    rospy.Subscriber('/Odometry', Odometry , odometry_callback, (g_gps_time, g_gps_lat, g_gps_lon, log_filepass))
    rospy.Subscriber('/livox/imu', Imu , imu_callback, (g_gps_time, g_gps_lat, g_gps_lon, log_filepass))
    rospy.Subscriber('/livox/lidar', CustomMsg, lidar_callback, (pcd_filepass, ))

    rospy.spin()

def gps_process(g_gps_time, g_gps_lat, g_gps_lon):
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

            # time
            if gps_time == 'n/a':
                gps_time = 0.0
            else:
                tmp_time = str(gps_time).replace('T', ' ')[:-1]
                tmp_time = datetime.datetime.strptime(tmp_time, date_format)
                tmp_time = tmp_time + datetime.timedelta(hours=9)
                gps_time = tmp_time.strftime("%Y-%m-%d_%H-%M-%S")
                g_gps_time.value = bytes(gps_time, 'utf-8') # 共有メモリ(str)

            # lat & lon
            if gps_lat == 'n/a' and gps_lon == 'n/a':
                gps_lat = 0.0
                gps_lon = 0.0
            else:
                g_gps_lat.value = gps_lat # 共有メモリ(float)
                g_gps_lon.value = gps_lon # 共有メモリ(float)

def log_out(g_gps_time, g_gps_lat, g_gps_lon):
    while True:
        print('--------------------------------------')
        print("[Log] gps_time:", g_gps_time.value.decode('utf-8'))
        print("[Log] gps_lat:", g_gps_lat.value)
        print("[Log] gps_lon:", g_gps_lon.value)
        print('')
        time.sleep(1)

def main():
    # Logファイルパスの設定
    log_filepass = log_dirs + "/" + str(get_device_time("conv_str"))
    os.makedirs(log_filepass)
    print("[Debug] Log file save to: {}".format(log_filepass))

    # PCDファイルパスの設定
    pcd_filepass = log_filepass + "/pcd_data"
    os.makedirs(pcd_filepass)
    print("[Debug] PCD file save to: {}".format(pcd_filepass))

    # 共有メモリの設定
    m_gps_time = Array('c', b"0000-00-00_00-00-00")
    m_gps_lat = Value('d', 0.0)
    m_gps_lon = Value('d', 0.0)

    # GPSプロセスの設定
    p_gps = Process(target=gps_process, args=(m_gps_time, m_gps_lat, m_gps_lon))
    p_gps.start()

    # ROSプロセスの設定
    p_ros = Process(target=ros_process, args=(m_gps_time, m_gps_lat, m_gps_lon, log_filepass, pcd_filepass))
    p_ros.start()

    # # デバッグ用
    # p_log = Process(target=log_out, args=(m_gps_time, m_gps_lat, m_gps_lon))
    # p_log.start()

if __name__ == "__main__":
    main()