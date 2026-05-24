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
#include <vector>
#include "fugseg/cloud.h"

namespace fugseg {

class KITTI {
	public:
		KITTI(const std::string &directory);
		~KITTI();

	private:
        std::string _directory;                     // directory to the KITTI odometry dataset
        std::vector<std::string> _scan_names;       // list of scans within this sequence, e.g., "000000.bin", "000001.bin", ...
        const size_t _buff_points;                  // max number of points in a scan

	private:
        /**
         * @brief parse the given Lidar scan
         */
        bool parseScan(const std::string &scan_dir, Cloud &cloud);

	public:
        /**
         * @brief reset all working variables
         */
        void Reset(const std::string &directory);

        /**
         * @brief open the given epoch, and load the scan into cloud
         */
        bool OpenFrame(const size_t epoch, Cloud &cloud);

        size_t MaxEpochs() const { return _scan_names.size(); }
};

} // namespace fugseg
