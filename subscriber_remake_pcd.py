#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from livox_ros_driver.msg import CustomMsg
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import datetime
import time
import os
import torch.multiprocessing as mp
from multiprocessing import Process, Value, Array


DEVICE = o3d.core.Device("CUDA:0")
DTYPE = o3d.core.float32

LOG_DIRS = "/workspace/bind_data"

PUBLISH_HZ = 60
TARGET_HZ = 10

MSG_TYPE = 0 # [0]=PointCloud2, [1]=CustomMsg


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


def callback_lidar(point_cloud, args):
    que = args[0]
    ros_time = args[1]
    
    # 共有メモリにROS_timeを保存
    ros_time_raw = point_cloud.header.stamp
    ros_time_unix = float(str(ros_time_raw.secs) + "." + str(ros_time_raw.nsecs).zfill(9)[:3])
    ros_time_dt = datetime.datetime.utcfromtimestamp(ros_time_unix)
    ros_time_dt_jst = ros_time_dt + datetime.timedelta(hours=9)
    ros_time_str = ros_time_dt_jst.strftime('%Y-%m-%d_%H-%M-%S_%f')[:-3]
    ros_time.value = bytes(ros_time_str, "utf-8") # 共有メモリを更新(str)

    if MSG_TYPE == 0:
        # PointCloud2型の点群データを読み取る
        points = []
        intensities = []
        for point in pc2.read_points(point_cloud, field_names=("x", "y", "z", "intensity"), skip_nans=True):
            points.append([point[0], point[1], point[2]])
            intensities.append([point[3]])
        points = np.array(points, dtype="float32")
        intensities = np.array(intensities, dtype="float32")
    elif MSG_TYPE == 1:
        # CustomMsg型の点群データを読み取る
        num_points = point_cloud.point_num # Pointの数
        points = np.zeros((num_points, 3), dtype="float32")
        intensities = np.zeros((num_points, 1), dtype="float32")
        for i, point in enumerate(point_cloud.points):
            points[i, 0] = point.x
            points[i, 1] = point.y
            points[i, 2] = point.z
            intensities[i, 0] = np.float32(point.reflectivity)
    
    # tensorを用いて点群をpcdに保存
    pcd_t = o3d.t.geometry.PointCloud(DEVICE)
    pcd_t.point.positions = o3d.core.Tensor(points, DTYPE, DEVICE)
    pcd_t.point.intensity = o3d.core.Tensor(intensities, DTYPE, DEVICE)

    # キューに追加
    que.put(pcd_t)
    print(" :: [ADD] Now queue size :" + str(que.qsize()))


def process_ros(q_pointcloud, g_ros_time):
    init_cuda()
    
    rospy.init_node("research_subscriber")

    if MSG_TYPE == 0:
        rospy.Subscriber("/livox/lidar", PointCloud2, callback_lidar, (q_pointcloud, g_ros_time))
    elif MSG_TYPE == 1:
        rospy.Subscriber("/livox/lidar", CustomMsg, callback_lidar, (q_pointcloud, g_ros_time))

    rospy.spin()


def process_pcd(q_pointcloud, g_ros_time, pcd_filepass):
    coupling_num = PUBLISH_HZ / TARGET_HZ
    
    while 1:
        if q_pointcloud.qsize() >= coupling_num:
            pcd_t_save = o3d.t.geometry.PointCloud(DEVICE)
            temp = q_pointcloud.get()
            pcd_t_save = temp
            
            while True:
                if not q_pointcloud.empty():
                    temp = q_pointcloud.get()
                    pcd_t_save = pcd_t_save + temp
                else:
                    break
        
            print("=== Que Get Success !! ===")
            o3d.t.io.write_point_cloud(pcd_filepass + "/" + str(g_ros_time.value)[2:-1] + ".pcd", pcd_t_save)


def main():
    if mp.get_start_method() == "fork":
        mp.set_start_method("spawn", force=True)
    
    # キューの設定
    manager = mp.Manager()
    q_pointcloud = manager.Queue()
    
    # 共有メモリの設定
    m_ros_time = Array("c", b"0000-00-00_00-00-00_000")
    
    # PCDファイルパスの設定
    pcd_filepass = LOG_DIRS + "/" + str(get_device_time("conv_str") + "_" + str(TARGET_HZ) + "Hz")
    os.makedirs(pcd_filepass)
    print(" :: [Debug] PCD file save to: {}".format(pcd_filepass))

    # ROSプロセスの設定
    p_ros = mp.Process(target=process_ros, args=(q_pointcloud, m_ros_time))
    p_ros.start()
    
    # PCDプロセスの設定
    p_pcd = mp.Process(target=process_pcd, args=(q_pointcloud, m_ros_time, pcd_filepass))
    p_pcd.start()
    
    p_ros.join()
    p_pcd.join()



if __name__ == "__main__":
    main()