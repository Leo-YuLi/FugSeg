// Copyright (c) 2025 Yu Li

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <string>
#include <chrono>
#include <numeric>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "fugseg/fugseg.h"
#include "fugseg/cloud.h"
#include "fugseg/dataset.h"
#include "utils.h"

// global variables
ros::Publisher _pub_pc_det_grd;
ros::Publisher _pub_pc_det_nongrd;
ros::Publisher _pub_pc_raw;

void rosLoop(const float rate, fugseg::GroundExtractor *ground_extractor, fugseg::Cloud *cloud, fugseg::KITTI *kitti)
{
    sensor_msgs::PointCloud2 pc_det_grd = utils::NewPointCloud(std::string("lidar"));
    sensor_msgs::PointCloud2 pc_det_nongrd = utils::NewPointCloud(std::string("lidar"));
    sensor_msgs::PointCloud2 pc_raw = utils::NewPointCloud(std::string("lidar"));
    // processing loop
    size_t epoch = 0;
    size_t MaxEpochs = kitti->MaxEpochs();
    ros::Rate loop_rate(rate);
    while (ros::ok() && epoch<MaxEpochs) {
        // 1. open new frame
        if (!kitti->OpenFrame(epoch, *cloud)) {
            break;
        }
        // 2. ground segmentation
        fugseg::Cluster3D cluster_ground;
        fugseg::Cluster3D cluster_others;
        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
        if (!ground_extractor->ExtractGround(*cloud, cluster_ground, cluster_others)) {
            break;
        }
        double runtime_s = (std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now()-t0)).count();
        // 3. publisher update
        fugseg::Cluster3D cluster_raw(cloud->points().rows());
        std::iota(cluster_raw.begin(), cluster_raw.end(), 0);
        utils::BuildPointCloud(pc_raw, cloud->points(), cluster_raw);
        utils::BuildPointCloud(pc_det_grd, cloud->points(), cluster_ground);
        utils::BuildPointCloud(pc_det_nongrd, cloud->points(), cluster_others);
        _pub_pc_det_grd.publish(pc_det_grd);
        _pub_pc_det_nongrd.publish(pc_det_nongrd);
        _pub_pc_raw.publish(pc_raw);
        ros::spinOnce();
        // 4. logging
        ROS_INFO("Process epoch: %d / %d. Runtime: %.3f (ms/frame)", epoch+1, MaxEpochs, runtime_s*1e3f);
        loop_rate.sleep();
        epoch++;
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "viewer");
	ros::NodeHandle nh("~");
    _pub_pc_det_grd = nh.advertise<sensor_msgs::PointCloud2>("pc_det_grd",1);
    _pub_pc_det_nongrd = nh.advertise<sensor_msgs::PointCloud2>("pc_det_nongrd",1);
    _pub_pc_raw = nh.advertise<sensor_msgs::PointCloud2>("pc_raw",1);
    // global control params
    const std::string glb_lidar_dir = nh.param<std::string>("glb_lidar_dir","");
    const float glb_rate = nh.param<float>("glb_rate",10);
    // FugSeg related params
    const int grd_NumSegs = nh.param<int>("grd_NumSegs",120);
    const float grd_BinRes = nh.param<float>("grd_BinRes",1.0);
    const float grd_ThreInlier = nh.param<float>("grd_ThreInlier",0.15);
    const float grd_DslopeDegMax = nh.param<float>("grd_DslopeDegMax",7.0);
    const float grd_MaxGapRadial = nh.param<float>("grd_MaxGapRadial",10.0);
    if (glb_lidar_dir.empty()) {
        ROS_ERROR("Please specify the directory to the KITTI odometry dataset using: glb_lidar_dir");
        ros::shutdown();
        return -1;
    }
    // start the processing loop
    fugseg::Cloud cloud;
    fugseg::KITTI kitti(glb_lidar_dir);
    fugseg::GroundExtractor ground_extractor(grd_NumSegs, grd_BinRes, grd_ThreInlier, grd_DslopeDegMax, grd_MaxGapRadial);
    rosLoop(glb_rate, &ground_extractor, &cloud, &kitti);
    // field cleanup
    ros::shutdown();
    return 0;
}