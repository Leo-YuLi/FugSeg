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

#pragma once

#include <string>
#include <Eigen/Dense>
#include "sensor_msgs/PointCloud2.h"
#include "fugseg/utils.h"

namespace utils {

/**
 * @brief create a new PointCloud2 message
 */
sensor_msgs::PointCloud2 NewPointCloud(const std::string &frame_id);

/**
 * @brief fill a PointCloud2 using the provided information
 */
void BuildPointCloud(sensor_msgs::PointCloud2 &pc, const Eigen::ArrayX4f &cloud, const fugseg::Cluster3D &cluster,
                        const uint32_t sequence = 0, const ros::Time &timestamp = ros::Time::now());

}   // namespace utils
