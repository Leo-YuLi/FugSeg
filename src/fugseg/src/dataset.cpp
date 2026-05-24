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

#include "fugseg/dataset.h"
#include <iostream>
#include <fstream>
#include "fugseg/utils.h"

namespace fugseg {

KITTI::KITTI(const std::string &directory)
: _buff_points(134400)
{
    Reset(directory);
}

KITTI::~KITTI()
{
}

void KITTI::Reset(const std::string &directory)
{
    _directory = directory;
    _scan_names = fugseg::getFilesWithExt(_directory, ".bin");
    std::cout << "find " << _scan_names.size() << " scans under: " << _directory << std::endl;
}

bool KITTI::parseScan(const std::string &scan_dir, Cloud &cloud)
{
    // 1. working variables for point cloud
    Eigen::ArrayX4f& points = cloud.points();
    // 2. open the Lidar scan
    // allocate 4 MB buffer (only ~130*4*4 KB are needed)
    float *data = (float*)malloc(_buff_points*4*sizeof(float));
    float *px = data+0;
    float *py = data+1;
    float *pz = data+2;
    float *pr = data+3;
    FILE *stream;
    stream = fopen(scan_dir.c_str(),"rb");
    if (stream == nullptr) {
        std::cerr << "Can not open the file: " << scan_dir << std::endl;
        free(data);
        fclose(stream);
        return false;
    }
    size_t num_points = fread(data,sizeof(float),_buff_points*4,stream)/4;
    // 3. load point cloud
    cloud.Reset(num_points);
    for (size_t i=0; i<num_points; i++) {
        points.row(i) << *px, *py, *pz, *pr;
        px+=4; py+=4; pz+=4; pr+=4;
    }
    // field cleanup
    fclose(stream);
    free(data);
    return true;
}

bool KITTI::OpenFrame(const size_t epoch, Cloud &cloud)
{
    if (epoch >= _scan_names.size()) {
        std::cerr << "Invalid epoch: " << epoch << ". Valid range is [0, " << _scan_names.size()-1 << "]." << std::endl;
        assert(0);
        return false;
    }
    // parse and load the scan
    std::string scan_dir = _directory + "/" + _scan_names[epoch];
    if(!parseScan(scan_dir, cloud)) {
        std::cerr << "Failed to load the scan: " << scan_dir << std::endl;
        assert(0);
        return false;
    }
    return true;
}

}  // namespace fugseg