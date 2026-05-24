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

#include "utils.h"

namespace utils {

sensor_msgs::PointCloud2 NewPointCloud(const std::string &frame_id)
{
    sensor_msgs::PointCloud2 pc;
    pc.header.frame_id = frame_id;
    pc.is_bigendian = 1;
    pc.is_dense = 1;
    pc.height = 1;
    sensor_msgs::PointField field;
    field.count = 1;
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.offset = 0;
    field.name = 'x';
    pc.fields.push_back(field);
    field.offset = 4;
    field.name = 'y';
    pc.fields.push_back(field);
    field.offset = 8;
    field.name = 'z';
    pc.fields.push_back(field);
    field.offset = 12;
    field.name = "intensity";
    pc.fields.push_back(field);
    pc.point_step = 16;
    return pc;
}

void BuildPointCloud(sensor_msgs::PointCloud2 &pc, const Eigen::ArrayX4f &cloud, const fugseg::Cluster3D &cluster, const uint32_t sequence, const ros::Time &timestamp)
{
    pc.data.clear();
    size_t num_points = cluster.size();
    pc.data.resize(num_points*pc.point_step, 0);
    size_t offset = 0;
    for (size_t i=0; i<num_points; i++) {
        offset = i*pc.point_step;
        float *X = (float*)(&pc.data[offset]);
        float *Y = X + 1;
        float *Z = X + 2;
        float *R = X + 3;
        *X = cloud(cluster[i], 0);
        *Y = cloud(cluster[i], 1);
        *Z = cloud(cluster[i], 2);
        *R = cloud(cluster[i], 3);
    }
    pc.header.stamp = timestamp;
    pc.header.seq = sequence;
    pc.width = num_points;
    pc.row_step = pc.width * pc.point_step;
}

}   // namespace utils