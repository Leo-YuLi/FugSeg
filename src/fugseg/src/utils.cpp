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

#include "fugseg/utils.h"
#include <boost/filesystem.hpp>
#include <algorithm>
#include <iostream>

namespace fugseg {

std::vector<std::string> getFilesWithExt(const std::string& directory, const std::string& extension)
{
    std::vector<std::string> extFiles;
    // Check if the directory exists
    boost::filesystem::path dirPath(directory);
    if (!boost::filesystem::exists(dirPath) || !boost::filesystem::is_directory(dirPath)) {
        std::cerr << "Directory does not exist or is not a directory: " << directory << std::endl;
        assert(0);
        return extFiles;
    }
    // Iterate through the directory
    for (const auto& entry : boost::filesystem::directory_iterator(dirPath)) {
        if (boost::filesystem::is_regular_file(entry.path()) && entry.path().extension() == extension) {
            extFiles.push_back(entry.path().filename().string());
        }
    }
    // Sort the file names alphabetically
    std::sort(extFiles.begin(), extFiles.end());
    return extFiles;
}

}   // namespace fugseg