#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from livox_ros_driver.msg import CustomMsg
import open3d as o3d
import numpy as np
import datetime
import time
import os
import torch.multiprocessing as mp


DEVICE = o3d.core.Device("CUDA:0")
DTYPE = o3d.core.float32

LOG_DIRS = "/workspace/bind_data"

COUPLING_NUM = 6


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
    pcd_t.point.positions = o3d.core.Tensor(points, DTYPE, DEVICE)
    pcd_t.point.intensity = o3d.core.Tensor(intensities, DTYPE, DEVICE)
    

    que.put(pcd_t)
    print(" :: [ADD] Now queue size :" + str(que.qsize()))


def process_ros(q_pointcloud):
    init_cuda()
    
    rospy.init_node('remake_pcd')
    rospy.Subscriber('/livox/lidar', CustomMsg, callback_lidar, (q_pointcloud,))
    rospy.spin()


def process_pcd(q_pointcloud, pcd_filepass):
    while 1:
        if q_pointcloud.qsize() >= COUPLING_NUM:
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
            o3d.t.io.write_point_cloud(pcd_filepass + "/" + get_device_time("conv_str_milli")[:-3] + ".pcd", pcd_t_save)
    


def main():
    if mp.get_start_method() == 'fork':
        mp.set_start_method('spawn', force=True)
        
    manager = mp.Manager()
        
    q_pointcloud = manager.Queue()
    
    # PCDファイルパスの設定
    log_filepass = LOG_DIRS + "/" + str(get_device_time("conv_str"))
    os.makedirs(log_filepass)
    pcd_filepass = log_filepass + "/pcd_data"
    os.makedirs(pcd_filepass)
    print(" :: [Debug] PCD file save to: {}".format(pcd_filepass))


    # ROSプロセスの設定
    p_ros = mp.Process(target=process_ros, args=(q_pointcloud, ))
    p_ros.start()
    
    # PCDプロセスの設定
    p_pcd = mp.Process(target=process_pcd, args=(q_pointcloud, pcd_filepass))
    p_pcd.start()
    
    p_ros.join()
    p_pcd.join()



if __name__ == "__main__":
    main()