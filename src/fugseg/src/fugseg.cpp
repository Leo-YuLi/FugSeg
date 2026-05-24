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

#include "fugseg/fugseg.h"
#include <iostream>
#include <cmath>

namespace fugseg {

GroundExtractor::GroundExtractor(int numSegs, float binRes, float groundInlierThre, float maxDslopeDeg, float maxGapRadialLength, float rangeMin, float rangeMax)
:_numSegs(numSegs), _binRes(binRes), _groundInlierThre(groundInlierThre), _maxDslopeRad(maxDslopeDeg*PI/180.0), _maxGapRadialLength(maxGapRadialLength), _rangeMin(rangeMin), _rangeMax(rangeMax)
{
    // initialize radial bins
    iniBinUniform();
    // initialize angular segments
    _segBounds.clear();
    _segBounds.emplace_back(0.0f);
    float resSeg = 2*PI/numSegs;
    for (size_t i=0; i<numSegs; i++) {
        _segBounds.emplace_back(_segBounds.back() + resSeg);
    }
    _segBounds.back() = 2*PI;
    // dataset related parameters
    // KITTI dataset
    _m2_theta = 0.009*PI/180.0;
    _m2_theta *= _m2_theta;
    _m2_phi = 0.033*PI/180.0;
    _m2_phi *= _m2_phi;
    _m2_ra = 0.02*0.02;
    _buff_points = 134400;
    _lidarHeight = 1.73f;
    _maxHeightSeed = 0.3f - _lidarHeight;
    // ego vehicle parameters of KITTI dataset, taken from DipgSeg
    _close_region_boundary_x_neg = -1.6f;
    _close_region_boundary_x_pos = 2.6f;
    _close_region_boundary_y = 1.5f;
    // initialize other variables
    _ptID = Eigen::ArrayXXi::Zero(_numBins, _numSegs);
    _groundImg = Eigen::ArrayXXi::Zero(_numBins, _numSegs);
    _range2d = Eigen::ArrayXf::Zero(_buff_points);
    _yaws = Eigen::ArrayXf::Zero(_buff_points);
    _IDmapping = zIDss3D(_numSegs, zIDs3D(_numBins));
    _eleNodes = EleNodess(_numSegs, EleNodes(_numBins+1));
}

GroundExtractor::~GroundExtractor()
{
}

void GroundExtractor::reset(const size_t& num_pts)
{
    _ptID.fill(-1);
    _groundImg.fill(0);
    for (int col=0; col<_numSegs; col++) {
        for (int row=0; row<_numBins; row++) {
            _IDmapping[col][row].reset();
            _eleNodes[col][row].reset();
        }
        _eleNodes[col][_numBins].reset();
    }
}

bool GroundExtractor::ExtractGround(const fugseg::Cloud& cloud, fugseg::Cluster3D& cluster_ground, fugseg::Cluster3D& cluster_others)
{
    try {
        // 1. prepare working variables
        cluster_ground.clear();
        cluster_others.clear();
        _IDs_ground = &cluster_ground;
        _IDs_nonground = &cluster_others;
        _points = &cloud.points();
        reset(_points->rows());
        // 2. PGM
        projectPoints();
        // 3. UGL (SGL + CGP)
        LabelGroundVertical();
        LabelGroundHorizontalJoint();
        // 4. EGE + PGS
        SegmentGroundFine();
    } catch (const std::exception& e) {
        std::cerr << "Exception in ExtractGround(): " << e.what() << std::endl;
        return false;
    }
    return true;
}

void GroundExtractor::LabelGroundVertical()
{
    for (int col = 0; col < _groundImg.cols(); col++) {
        // 1. start from bottom of each column to find the first ground seed
        int row = 0;
        float groundSlope = getSlopeAngle0(row,col);   // the ground slope determined from r_last
        while (row < _groundImg.rows()-1 &&
                (_ptID(row, col) == -1                                      // row non-empty
                || (*_points)(_ptID(row, col),2) > _maxHeightSeed           // abs. height check
                || std::fabs(groundSlope) > _maxDslopeRad                   // initial slope check
                || _ptID(row+1, col) == -1                                  // row+1 non-empty
                || std::fabs(groundSlope-getSlopeAngleExt(_ptID(row, col), _ptID(row+1, col))) > _maxDslopeRad    // subsequent slope check
                )
        ) {
            row++;
            groundSlope = getSlopeAngle0(row,col);
        }
        // if no seed can be found: continue for the next segment
        if (row == _groundImg.rows()-1) {
            continue;
        }
        // otherwise, label it as ground
        else {
            _groundImg(row,col) = 1;
        }
        int row_seed = row;
        // 2.1 Forward labeling: start from this seed, try to find more ground points along radial direction
        int row_last = row_seed;
        float newSlope = groundSlope;
        row++;
        while (row < _groundImg.rows() && ((_ptID(row, col)==-1) || (_range2d(_ptID(row, col))-_range2d(_ptID(row_last, col))) < _maxGapRadialLength)) {
            if (_ptID(row, col) != -1) {
                newSlope = getSlopeAngleExt(_ptID(row_last, col), _ptID(row, col));
                // if the change in slope is moderate, label this cell as ground
                if (std::fabs(newSlope-groundSlope) < _maxDslopeRad) {
                    _groundImg(row,col) = 1;
                    groundSlope = newSlope;
                    row_last = row;
                }
                // otherwise, check for potential reflection noise below the ground plane
                else if (newSlope < 0.0f) {
                    _groundImg(row,col) -= 1;
                }
            }
            row++;
        }
        // 2.2 Backward labeling: try to find more ground points in the opposite radial direction (towards the sensor origin)
        row_last--;
        // find a ground seed which has non-ground neighbor down and ground neighbor up
        while (row_last>0 && !(_groundImg(row_last-1,col)!=1 && _groundImg(row_last,col)==1 && _groundImg(row_last+1,col)==1)) {
            row_last--;
        }
        if (row_last > 0) {
            groundSlope = getSlopeAngleExt(_ptID(row_last+1,col),_ptID(row_last,col));
            row = row_last-1;
            while (row > 0) {
                if (std::abs(row-row_last) < 2) {
                    if (_ptID(row, col) != -1) {
                        newSlope = getSlopeAngleExt(_ptID(row_last, col), _ptID(row, col));
                        if (_groundImg(row,col) == 1 || std::fabs(newSlope-groundSlope) < _maxDslopeRad) {
                            _groundImg(row,col) = 1;
                            groundSlope = newSlope;
                            row_last = row;
                        }
                        // otherwise, check for potential reflection noise below the ground plane
                        else if (newSlope < 0.0f) {
                            _groundImg(row,col) -= 1;
                        }
                    }
                }
                // otherwise, update the row_last to the next ground cell
                else {
                    row_last = row;
                    while (row_last>0 && !(_groundImg(row_last-1,col)!=1 && _groundImg(row_last,col)==1 && _groundImg(row_last+1,col)==1)) {
                        row_last--;
                    }
                    if (row_last > 0) {
                        groundSlope = getSlopeAngleExt(_ptID(row_last+1,col),_ptID(row_last,col));
                    }
                    row = row_last;
                }
                row--;
            }
        }
    }
}

void GroundExtractor::LabelGroundHorizontalJoint()
{
    ////////////////////////////////////////////////////////////////// Part I: near to far //////////////////////////////////////////////////////////////////
    // I-1. ground propagate from left to right (clock-wise)
    for (int row = 1; row < _groundImg.rows()-1; row++) {
        int col_last = 1;
        // find a ground seed which has left ground neighbor and right non-ground neighbor
        while (col_last<_groundImg.cols()-1 && !(_groundImg(row,col_last)==1 && _ptID(row,col_last+1)!=-1 && _groundImg(row,col_last+1)!=1)) {
            col_last++;
        }
        int col_seed = col_last;
        if (col_last != _groundImg.cols()-1) {
            // start from this seed, try to find more ground points clock-wise
            int col = col_last+1;
            float groundSlope;
            float newSlope;
            bool isProcessed = false;   // true: this cell has already been propagated in first step i.e. using horizontal slope vector
            while (col < _groundImg.cols()) {
                isProcessed = false;
                // try 1: propagation using horizontal slope vector
                if (_groundImg(row,col-2)==1 && _groundImg(row,col-1)==1 && _groundImg(row,col)!=1) {
                    groundSlope = getSlopeAngleExt(_ptID(row,col-2),_ptID(row,col-1));
                    newSlope = getSlopeAngleExt(_ptID(row,col-1),_ptID(row,col));
                    if (std::fabs(newSlope-groundSlope) < _maxDslopeRad) {
                        _groundImg(row,col) = 1;
                        isProcessed = true;
                    }
                    else if (newSlope < 0.0f) {
                        _groundImg(row,col) -= 1;
                        // pass noise
                        col_last = col-1;
                        while ((++col)<_groundImg.cols() && _groundImg(row,col)!=1 && _ptID(row,col)!=-1 && std::fabs(getSlopeAngleExt(_ptID(row,col_last),_ptID(row,col))-groundSlope)>_maxDslopeRad && getSlopeAngleExt(_ptID(row,col_last),_ptID(row,col))<0.0f) {
                            _groundImg(row,col) -= 1;
                        }
                        col--;
                    }
                }
                // try 2: propagation using vertical slope vector
                if (!isProcessed && _groundImg(row,col-1)==1 && _ptID(row,col)!=-1 && _groundImg(row,col)!=1 && (_groundImg(row-1,col-1)==1 || _groundImg(row+1,col-1)==1) && (_groundImg(row-1,col)==1 || _groundImg(row+1,col)==1)) {
                    groundSlope = getRadialSlopeAngleExt01(row,col-1);
                    newSlope = getRadialSlopeAngleExt01(row,col);
                    if (std::fabs(newSlope-groundSlope) < _maxDslopeRad) {
                        _groundImg(row,col) = 1;
                    }
                }
                // update col to the next ground-nonground cell
                col++;
                while (col<_groundImg.cols() && !(_groundImg(row,col-1)==1 && _ptID(row,col)!=-1 && _groundImg(row,col)!=1)) {
                    col++;
                }
            }
        }
        // cross-border handling
        if (_groundImg(row,_groundImg.cols()-1)==1 && _groundImg(row,0)!=1 && _ptID(row,0)!=-1) {
            int col0 = _groundImg.cols()-2;
            int col1 = _groundImg.cols()-1;
            int col = 0;
            float groundSlope;
            float newSlope;
            bool isProcessed = false;
            while (col < col_seed) {
                isProcessed = false;
                // try 1: propagation using horizontal slope vector
                if (_groundImg(row,col0)==1 && _groundImg(row,col1)==1 && _groundImg(row,col)!=1) {
                    groundSlope = getSlopeAngleExt(_ptID(row,col0),_ptID(row,col1));
                    newSlope = getSlopeAngleExt(_ptID(row,col1),_ptID(row,col));
                    if (std::fabs(newSlope-groundSlope) < _maxDslopeRad) {
                        _groundImg(row,col) = 1;
                        isProcessed = true;
                    }
                    else if (newSlope < 0.0f) {
                        _groundImg(row,col) -= 1;
                        // pass noise
                        while ((++col)<col_seed && _groundImg(row,col)!=1 && _ptID(row,col)!=-1 && std::fabs(getSlopeAngleExt(_ptID(row,col1),_ptID(row,col))-groundSlope)>_maxDslopeRad && getSlopeAngleExt(_ptID(row,col1),_ptID(row,col))<0.0f) {
                            _groundImg(row,col) -= 1;
                        }
                        col--;
                    }
                }
                // try 2: propagation using vertical slope vector
                if (!isProcessed && _groundImg(row,col1)==1 && _ptID(row,col)!=-1 && _groundImg(row,col)!=1 && (_groundImg(row-1,col1)==1 || _groundImg(row+1,col1)==1) && (_groundImg(row-1,col)==1 || _groundImg(row+1,col)==1)) {
                    groundSlope = getRadialSlopeAngleExt01(row,col1);
                    newSlope = getRadialSlopeAngleExt01(row,col);
                    if (std::fabs(newSlope-groundSlope) < _maxDslopeRad) {
                        _groundImg(row,col) = 1;
                    }
                }
                // update col to the next ground-nonground pixel
                col++;
                while (col<col_seed && !(_groundImg(row,col-1)==1 && _ptID(row,col)!=-1 && _groundImg(row,col)!=1)) {
                    col++;
                }
                col0 = (_groundImg.cols() + col-2) % _groundImg.cols();
                col1 = col-1;
            }
        }
    }
    // I-2. propagate from right to left (counter clock-wise)
    for (int row = 1; row < _groundImg.rows()-1; row++) {
        int col_last = _groundImg.cols()-2;
        // find a ground seed which has right ground neighbor and left non-ground neighbor
        while (col_last>0 && !(_groundImg(row,col_last-1)!=1 && _ptID(row,col_last-1)!=-1 && _groundImg(row,col_last)==1)) {
            col_last--;
        }
        int col_seed = col_last;
        if (col_last != 0) {
            // start from this seed, try to find more ground points counter clock-wise
            int col = col_last-1;
            float groundSlope;
            float newSlope;
            bool isProcessed = false;
            while (col >= 0) {
                isProcessed = false;
                // try 1: propagation using horizontal slope vector
                if (_groundImg(row,col+2)==1 && _groundImg(row,col+1)==1 && _groundImg(row,col)!=1) {
                    groundSlope = getSlopeAngleExt(_ptID(row,col+2),_ptID(row,col+1));
                    newSlope = getSlopeAngleExt(_ptID(row,col+1),_ptID(row,col));
                    if (std::fabs(newSlope-groundSlope) < _maxDslopeRad) {
                        _groundImg(row,col) = 1;
                        isProcessed = true;
                    }
                    else if (newSlope < 0.0f) {
                        _groundImg(row,col) -= 1;
                        // pass noise
                        col_last = col+1;
                        while ((--col)>=0 && _groundImg(row,col)!=1 && _ptID(row,col)!=-1 && std::fabs(getSlopeAngleExt(_ptID(row,col_last),_ptID(row,col))-groundSlope)>_maxDslopeRad && getSlopeAngleExt(_ptID(row,col_last),_ptID(row,col))<0.0f) {
                            _groundImg(row,col) -= 1;
                        }
                        col++;
                    }
                }
                // try 2: propagation using vertical slope vector
                if (!isProcessed && _groundImg(row,col+1)==1 && _ptID(row,col)!=-1 && _groundImg(row,col)!=1 && (_groundImg(row-1,col+1)==1 || _groundImg(row+1,col+1)==1) && (_groundImg(row-1,col)==1 || _groundImg(row+1,col)==1)) {
                    groundSlope = getRadialSlopeAngleExt01(row,col+1);
                    newSlope = getRadialSlopeAngleExt01(row,col);
                    if (std::fabs(newSlope-groundSlope) < _maxDslopeRad) {
                        _groundImg(row,col) = 1;
                    }
                }
                // update col to the next ground-nonground pixel
                col--;
                while (col>=0 && !(_groundImg(row,col+1)==1 && _ptID(row,col)!=-1 && _groundImg(row,col)!=1)) {
                    col--;
                }
            }
        }
        // cross-border handling
        if (_groundImg(row,0)==1 && _groundImg(row,_groundImg.cols()-1)!=1 && _ptID(row,_groundImg.cols()-1)!=-1) {
            int col0 = 1;
            int col1 = 0;
            int col = _groundImg.cols()-1;
            float groundSlope;
            float newSlope;
            bool isProcessed = false;
            while (col > col_seed) {
                isProcessed = false;
                // try 1: propagation using horizontal slope vector
                if (_groundImg(row,col0)==1 && _groundImg(row,col1)==1 && _groundImg(row,col)!=1) {
                    groundSlope = getSlopeAngleExt(_ptID(row,col0),_ptID(row,col1));
                    newSlope = getSlopeAngleExt(_ptID(row,col1),_ptID(row,col));
                    if (std::fabs(newSlope-groundSlope) < _maxDslopeRad) {
                        _groundImg(row,col) = 1;
                        isProcessed = true;
                    }
                    else if (newSlope < 0.0f) {
                        _groundImg(row,col) -= 1;
                        // pass noise
                        while ((--col)>col_seed && _groundImg(row,col)!=1 && _ptID(row,col)!=-1 && std::fabs(getSlopeAngleExt(_ptID(row,col1),_ptID(row,col))-groundSlope)>_maxDslopeRad && getSlopeAngleExt(_ptID(row,col1),_ptID(row,col))<0.0f) {
                            _groundImg(row,col) -= 1;
                        }
                        col++;
                    }
                }
                // try 2: propagation using vertical slope vector
                if (!isProcessed && _groundImg(row,col1)==1 && _ptID(row,col)!=-1 && _groundImg(row,col)!=1 && (_groundImg(row-1,col1)==1 || _groundImg(row+1,col1)==1) && (_groundImg(row-1,col)==1 || _groundImg(row+1,col)==1)) {
                    groundSlope = getRadialSlopeAngleExt01(row,col1);
                    newSlope = getRadialSlopeAngleExt01(row,col);
                    if (std::fabs(newSlope-groundSlope) < _maxDslopeRad) {
                        _groundImg(row,col) = 1;
                    }
                }
                // update col to the next ground-nonground pixel
                col--;
                while (col>col_seed && !(_groundImg(row,col+1)==1 && _ptID(row,col)!=-1 && _groundImg(row,col)!=1)) {
                    col--;
                }
                col0 = (col+2) % _groundImg.cols();
                col1 = col+1;
            }
        }
    }

    ////////////////////////////////////////////////////////////////// Part II: far to near //////////////////////////////////////////////////////////////////
    // II-1. propagate from left to right (clock-wise)
    for (int row = _groundImg.rows()-2; row > 0; row--) {
        int col_last = 1;
        // find a ground seed which has left ground neighbor and right non-ground neighbor
        while (col_last<_groundImg.cols()-1 && !(_groundImg(row,col_last)==1 && _ptID(row,col_last+1)!=-1 && _groundImg(row,col_last+1)!=1)) {
            col_last++;
        }
        int col_seed = col_last;
        if (col_last != _groundImg.cols()-1) {
            // start from this seed, try to find more ground points clock-wise
            int col = col_last+1;
            float groundSlope;
            float newSlope;
            bool isProcessed = false;
            while (col < _groundImg.cols()) {
                isProcessed = false;
                // try 1: propagation using horizontal slope vector
                if (_groundImg(row,col-2)==1 && _groundImg(row,col-1)==1 && _groundImg(row,col)!=1) {
                    groundSlope = getSlopeAngleExt(_ptID(row,col-2),_ptID(row,col-1));
                    newSlope = getSlopeAngleExt(_ptID(row,col-1),_ptID(row,col));
                    if (std::fabs(newSlope-groundSlope) < _maxDslopeRad) {
                        _groundImg(row,col) = 1;
                        isProcessed = true;
                    }
                    else if (newSlope < 0.0f) {
                        _groundImg(row,col) -= 1;
                        // pass noise
                        col_last = col-1;
                        while ((++col)<_groundImg.cols() && _groundImg(row,col)!=1 && _ptID(row,col)!=-1 && std::fabs(getSlopeAngleExt(_ptID(row,col_last),_ptID(row,col))-groundSlope)>_maxDslopeRad && getSlopeAngleExt(_ptID(row,col_last),_ptID(row,col))<0.0f) {
                            _groundImg(row,col) -= 1;
                        }
                        col--;
                    }
                }
                // try 2: propagation using vertical slope vector
                if (!isProcessed && _groundImg(row,col-1)==1 && _ptID(row,col)!=-1 && _groundImg(row,col)!=1 && (_groundImg(row-1,col-1)==1 || _groundImg(row+1,col-1)==1) && (_groundImg(row-1,col)==1 || _groundImg(row+1,col)==1)) {
                    groundSlope = getRadialSlopeAngleExt10(row,col-1);
                    newSlope = getRadialSlopeAngleExt10(row,col);
                    if (std::fabs(newSlope-groundSlope) < _maxDslopeRad) {
                        _groundImg(row,col) = 1;
                    }
                }
                // update col to the next ground-nonground pixel
                col++;
                while (col<_groundImg.cols() && !(_groundImg(row,col-1)==1 && _ptID(row,col)!=-1 && _groundImg(row,col)!=1)) {
                    col++;
                }
            }
        }
        // cross-border handling
        if (_groundImg(row,_groundImg.cols()-1)==1 && _groundImg(row,0)!=1 && _ptID(row,0)!=-1) {
            int col0 = _groundImg.cols()-2;
            int col1 = _groundImg.cols()-1;
            int col = 0;
            float groundSlope;
            float newSlope;
            bool isProcessed = false;
            while (col < col_seed) {
                isProcessed = false;
                // try 1: propagation using horizontal slope vector
                if (_groundImg(row,col0)==1 && _groundImg(row,col1)==1 && _groundImg(row,col)!=1) {
                    groundSlope = getSlopeAngleExt(_ptID(row,col0),_ptID(row,col1));
                    newSlope = getSlopeAngleExt(_ptID(row,col1),_ptID(row,col));
                    if (std::fabs(newSlope-groundSlope) < _maxDslopeRad) {
                        _groundImg(row,col) = 1;
                        isProcessed = true;
                    }
                    else if (newSlope < 0.0f) {
                        _groundImg(row,col) -= 1;
                        // pass noise
                        while ((++col)<col_seed && _groundImg(row,col)!=1 && _ptID(row,col)!=-1 && std::fabs(getSlopeAngleExt(_ptID(row,col1),_ptID(row,col))-groundSlope)>_maxDslopeRad && getSlopeAngleExt(_ptID(row,col1),_ptID(row,col))<0.0f) {
                            _groundImg(row,col) -= 1;
                        }
                        col--;
                    }
                }
                // try 2: propagation using vertical slope vector
                if (!isProcessed && _groundImg(row,col1)==1 && _ptID(row,col)!=-1 && _groundImg(row,col)!=1 && (_groundImg(row-1,col1)==1 || _groundImg(row+1,col1)==1) && (_groundImg(row-1,col)==1 || _groundImg(row+1,col)==1)) {
                    groundSlope = getRadialSlopeAngleExt10(row,col1);
                    newSlope = getRadialSlopeAngleExt10(row,col);
                    if (std::fabs(newSlope-groundSlope) < _maxDslopeRad) {
                        _groundImg(row,col) = 1;
                    }
                }
                // update col to the next ground-nonground pixel
                col++;
                while (col<col_seed && !(_groundImg(row,col-1)==1 && _ptID(row,col)!=-1 && _groundImg(row,col)!=1)) {
                    col++;
                }
                col0 = (_groundImg.cols() + col-2) % _groundImg.cols();
                col1 = col-1;
            }
        }
    }
    // II-2. propagate from right to left (counter clock-wise)
    for (int row = _groundImg.rows()-2; row > 0; row--) {
        int col_last = _groundImg.cols()-2;
        // find a ground seed which has right ground neighbor and left non-ground neighbor
        while (col_last>0 && !(_groundImg(row,col_last-1)!=1 && _ptID(row,col_last-1)!=-1 && _groundImg(row,col_last)==1)) {
            col_last--;
        }
        int col_seed = col_last;
        if (col_last != 0) {
            // start from this seed, try to find more ground points counter clock-wise
            int col = col_last-1;
            float groundSlope;
            float newSlope;
            bool isProcessed = false;
            while (col >= 0) {
                isProcessed = false;
                // try 1: propagation using horizontal slope vector
                if (_groundImg(row,col+2)==1 && _groundImg(row,col+1)==1 && _groundImg(row,col)!=1) {
                    groundSlope = getSlopeAngleExt(_ptID(row,col+2),_ptID(row,col+1));
                    newSlope = getSlopeAngleExt(_ptID(row,col+1),_ptID(row,col));
                    if (std::fabs(newSlope-groundSlope) < _maxDslopeRad) {
                        _groundImg(row,col) = 1;
                        isProcessed = true;
                    }
                    else if (newSlope < 0.0f) {
                        _groundImg(row,col) -= 1;
                        // pass noise
                        col_last = col+1;
                        while ((--col)>=0 && _groundImg(row,col)!=1 && _ptID(row,col)!=-1 && std::fabs(getSlopeAngleExt(_ptID(row,col_last),_ptID(row,col))-groundSlope)>_maxDslopeRad && getSlopeAngleExt(_ptID(row,col_last),_ptID(row,col))<0.0f) {
                            _groundImg(row,col) -= 1;
                        }
                        col++;
                    }
                }
                // try 2: propagation using vertical slope vector
                if (!isProcessed && _groundImg(row,col+1)==1 && _ptID(row,col)!=-1 && _groundImg(row,col)!=1 && (_groundImg(row-1,col+1)==1 || _groundImg(row+1,col+1)==1) && (_groundImg(row-1,col)==1 || _groundImg(row+1,col)==1)) {
                    groundSlope = getRadialSlopeAngleExt10(row,col+1);
                    newSlope = getRadialSlopeAngleExt10(row,col);
                    if (std::fabs(newSlope-groundSlope) < _maxDslopeRad) {
                        _groundImg(row,col) = 1;
                    }
                }
                // update col to the next ground-nonground pixel
                col--;
                while (col>=0 && !(_groundImg(row,col+1)==1 && _ptID(row,col)!=-1 && _groundImg(row,col)!=1)) {
                    col--;
                }
            }
        }
        // cross-border handling
        if (_groundImg(row,0)==1 && _groundImg(row,_groundImg.cols()-1)!=1 && _ptID(row,_groundImg.cols()-1)!=-1) {
            int col0 = 1;
            int col1 = 0;
            int col = _groundImg.cols()-1;
            float groundSlope;
            float newSlope;
            bool isProcessed = false;
            while (col > col_seed) {
                isProcessed = false;
                // try 1: propagation using horizontal slope vector
                if (_groundImg(row,col0)==1 && _groundImg(row,col1)==1 && _groundImg(row,col)!=1) {
                    groundSlope = getSlopeAngleExt(_ptID(row,col0),_ptID(row,col1));
                    newSlope = getSlopeAngleExt(_ptID(row,col1),_ptID(row,col));
                    if (std::fabs(newSlope-groundSlope) < _maxDslopeRad) {
                        _groundImg(row,col) = 1;
                        isProcessed = true;
                    }
                    else if (newSlope < 0.0f) {
                        _groundImg(row,col) -= 1;
                        // pass noise
                        while ((--col)>col_seed && _groundImg(row,col)!=1 && _ptID(row,col)!=-1 && std::fabs(getSlopeAngleExt(_ptID(row,col1),_ptID(row,col))-groundSlope)>_maxDslopeRad && getSlopeAngleExt(_ptID(row,col1),_ptID(row,col))<0.0f) {
                            _groundImg(row,col) -= 1;
                        }
                        col++;
                    }
                }
                // try 2: propagation using vertical slope vector
                if (!isProcessed && _groundImg(row,col1)==1 && _ptID(row,col)!=-1 && _groundImg(row,col)!=1 && (_groundImg(row-1,col1)==1 || _groundImg(row+1,col1)==1) && (_groundImg(row-1,col)==1 || _groundImg(row+1,col)==1)) {
                    groundSlope = getRadialSlopeAngleExt10(row,col1);
                    newSlope = getRadialSlopeAngleExt10(row,col);
                    if (std::fabs(newSlope-groundSlope) < _maxDslopeRad) {
                        _groundImg(row,col) = 1;
                    }
                }
                // update col to the next ground-nonground pixel
                col--;
                while (col>col_seed && !(_groundImg(row,col+1)==1 && _ptID(row,col)!=-1 && _groundImg(row,col)!=1)) {
                    col--;
                }
                col0 = (col+2) % _groundImg.cols();
                col1 = col+1;
            }
        }
    }
}

void GroundExtractor::SegmentGroundFine()
{
    // 1. [EGE: 1/2] elevation estimation for ground nodes (using cells with _groundImg==1)
    for (int r=0; r<_groundImg.rows(); r++) {
        for (int c=0; c<_groundImg.cols(); c++) {
            if (_groundImg(r, c) == 1) {
                float dYaw02 = (_yaws(_ptID(r,c)) - _segBounds.at(c)) * (_yaws(_ptID(r,c)) - _segBounds.at(c));
                float dYaw12 = (_yaws(_ptID(r,c)) - _segBounds.at(c+1)) * (_yaws(_ptID(r,c)) - _segBounds.at(c+1));
                float dRange02 = (_range2d(_ptID(r,c)) - _binBounds.at(r)) * (_range2d(_ptID(r,c)) - _binBounds.at(r));
                float dRange12 = (_range2d(_ptID(r,c)) - _binBounds.at(r+1)) * (_range2d(_ptID(r,c)) - _binBounds.at(r+1));
                // propagate to lower-left
                _eleNodes[c][r].addElevation((*_points)(_ptID(r,c), 2), std::exp(-1.0 * std::sqrt(dYaw02*_binBounds.at(r)*_binBounds.at(r) + dRange02)));
                // propagate to lower-right
                _eleNodes[(c+1)%_numSegs][r].addElevation((*_points)(_ptID(r,c), 2), std::exp(-1.0 * std::sqrt(dYaw12*_binBounds.at(r)*_binBounds.at(r) + dRange02)));
                // propagate to upper-left
                _eleNodes[c][r+1].addElevation((*_points)(_ptID(r,c), 2), std::exp(-1.0 * std::sqrt(dYaw02*_binBounds.at(r+1)*_binBounds.at(r+1) + dRange12)));
                // propagate to upper-right
                _eleNodes[(c+1)%_numSegs][r+1].addElevation((*_points)(_ptID(r,c), 2), std::exp(-1.0 * std::sqrt(dYaw12*_binBounds.at(r+1)*_binBounds.at(r+1) + dRange12)));
            }
            // pre-set the state for all potential noise reflection cells
            else if (_groundImg(r, c) < 0) {
                _eleNodes[c][r].state = 2;
                _eleNodes[(c+1)%_numSegs][r].state = 2;
                _eleNodes[c][r+1].state = 2;
                _eleNodes[(c+1)%_numSegs][r+1].state = 2;
            }
        }
    }
    for (int r=0; r<_numBins+1; r++) {
        for (int c=0; c<_numSegs; c++) {
            _eleNodes[c][r].iniElevation(1);
        }
    }

    // 2. [EGE: 2/2] elevation estimation for noisy ground nodes (using cells with _groundImg < 0, through four-pass height propagation)
    // 2.1 horizontal direction (two rounds)
    for (int row=0; row<_numBins+1; row++) {
        // round 1: 0 -> Inf
        float res = _binBounds.at(row) * (_segBounds.at(1)-_segBounds.at(0));   // angular resolution (in meter) on this row
        float distance = 0.0;       // accumulated distance used to weight the propagated elevation
        int col_seed = -1;          // used to complete the cross-over propagation
        for (int col=0; col<_numSegs; col++) {
            if ((_eleNodes[col][row].state==1 || _eleNodes[col][row].state==4) && _eleNodes[(col+1)%_numSegs][row].state==2) {
                distance += res;
                _eleNodes[(col+1)%_numSegs][row].addElevation(_eleNodes[col][row].elevation, std::exp(-1.0 * distance));
                _eleNodes[(col+1)%_numSegs][row].state = 4;      // [4]: horizontal left to right
                if (col_seed == -1) {
                    col_seed = col;
                }
            }
            // reset the accumulated distance
            else {
                distance = 0.0;
            }
        }
        // complete the cross-over propagation
        if (col_seed!=-1 && _eleNodes[0][row].state == 4) {
            for (int col=0; col<col_seed; col++) {
                if ((_eleNodes[col][row].state==1 || _eleNodes[col][row].state==4) && _eleNodes[col+1][row].state==2) {
                    distance += res;
                    _eleNodes[col+1][row].addElevation(_eleNodes[col][row].elevation, std::exp(-1.0 * distance));
                    _eleNodes[col+1][row].state = 4;     // [4]: horizontal left to right
                }
                else {
                    distance = 0.0;
                }
            }
        }
        // round 2: Inf -> 0
        distance = 0.0;
        col_seed = -1;
        for (int col=_numSegs-1; col>=0; col--) {
            if ((_eleNodes[col][row].state==1 || _eleNodes[col][row].state==14) && (_eleNodes[(col-1+_numSegs)%_numSegs][row].state==2 || _eleNodes[(col-1+_numSegs)%_numSegs][row].state==4)) {
                distance += res;
                _eleNodes[(col-1+_numSegs)%_numSegs][row].addElevation(_eleNodes[col][row].elevation, std::exp(-1.0 * distance));
                _eleNodes[(col-1+_numSegs)%_numSegs][row].state = 14;    // [14]: horizontal right to left
                if (col_seed == -1) {
                    col_seed = col;
                }
            }
            else {
                distance = 0.0;
            }
        }
        // complete the cross-over propagation
        if (col_seed != -1 && _eleNodes[_numSegs-1][row].state == 14) {
            for (int col=_numSegs-1; col>col_seed; col--) {
                if ((_eleNodes[col][row].state==1 || _eleNodes[col][row].state==14) && (_eleNodes[col-1][row].state==2 || _eleNodes[col-1][row].state==4)) {
                    distance += res;
                    _eleNodes[col-1][row].addElevation(_eleNodes[col][row].elevation, std::exp(-1.0 * distance));
                    _eleNodes[col-1][row].state = 14;    // [14]: horizontal right to left
                }
                else {
                    distance = 0.0;
                }
            }
        }
    }
    // 2.2 vertical direction (two rounds)
    for (int col=0; col<_numSegs; col++) {
        // round 1: 0 -> Inf
        float distance = 0.0;
        for (int row=1; row<_numBins+1; row++) {
            if ((_eleNodes[col][row-1].state==1 || _eleNodes[col][row-1].state==3) && (_eleNodes[col][row].state==2 || _eleNodes[col][row].state==4 || _eleNodes[col][row].state==14)) {
                distance += _binBounds.at(row)-_binBounds.at(row-1);
                _eleNodes[col][row].addElevation(_eleNodes[col][row-1].elevation, std::exp(-1.0 * distance));
                _eleNodes[col][row].state = 3;   // [3]: vertical forward
            }
            else {
                distance = 0.0;
            }
        }
        // round 2: Inf -> 0
        distance = 0.0;
        for (int row=_numBins-1; row>=0; row--) {
            if ((_eleNodes[col][row+1].state==1 || _eleNodes[col][row+1].state==13) && (_eleNodes[col][row].state==2 || _eleNodes[col][row].state==3 || _eleNodes[col][row].state==4 || _eleNodes[col][row].state==14)) {
                distance += _binBounds.at(row+1)-_binBounds.at(row);
                _eleNodes[col][row].addElevation(_eleNodes[col][row+1].elevation, std::exp(-1.0 * distance));
                _eleNodes[col][row].state = 13;  // [13]: vertical backward
            }
            // reset the accumulated distance
            else {
                distance = 0.0;
            }
        }
    }
    // 2.3 elevation determination for all noisy nodes with state 3, 13, 4 or 14
    for (int r=0; r<_numBins+1; r++) {
        for (int c=0; c<_numSegs; c++) {
            if (_eleNodes[c][r].state == 13 || _eleNodes[c][r].state == 3 || _eleNodes[c][r].state == 14 || _eleNodes[c][r].state == 4) {
                _eleNodes[c][r].iniElevation(5);
            }
        }
    }

    // 3. [PGS] point-level ground segmentation
    for (int r=0; r<_groundImg.rows(); r++) {
        for (int c=0; c<_groundImg.cols(); c++) {
            if (_groundImg(r, c)!=0 && hasValidNeighbors(r, c)) {
                // case 1: above ground cells
                if (_groundImg(r, c) == 1) {
                    for (const auto& ptID : _IDmapping[c][r].IDs) {
                        if ((*_points)(ptID,2) > (interpolateGrdElevation(r, c, ptID) + _groundInlierThre)) {
                            _IDs_nonground->emplace_back(ptID);
                        }
                        else {
                            _IDs_ground->emplace_back(ptID);
                        }
                    }
                }
                // case 2: potential noisy reflections + neighboring obstacle cells
                else if (_groundImg(r, c) < 0) {
                    for (const auto& ptID : _IDmapping[c][r].IDs) {
                        if (std::fabs((*_points)(ptID,2)-interpolateGrdElevation(r, c, ptID)) > _groundInlierThre) {
                            _IDs_nonground->emplace_back(ptID);
                        }
                        else {
                            _IDs_ground->emplace_back(ptID);
                        }
                    }
                }
                // case 3: obstacle cells
                else{
                    for (const auto& ptID : _IDmapping[c][r].IDs) {
                        _IDs_nonground->emplace_back(ptID);
                    }
                }
            }
            // case 4: all other cells without valid neighboring _eleNodes
            else{
                for (const auto& ptID : _IDmapping[c][r].IDs) {
                    _IDs_nonground->emplace_back(ptID);
                }
            }
        }
    }
}

inline bool GroundExtractor::hasValidNeighbors(const int& row, const int& col)
{
    _validity[0] = (float) (_eleNodes[col][row].weight_sum != 0.0f);
    _validity[1] = (float) (_eleNodes[(col+1)%_numSegs][row].weight_sum != 0.0f);
    _validity[2] = (float) (_eleNodes[col][row+1].weight_sum != 0.0f);
    _validity[3] = (float) (_eleNodes[(col+1)%_numSegs][row+1].weight_sum != 0.0f);
    return _validity[0]!=0.0f || _validity[1]!=0.0f || _validity[2]!=0.0f || _validity[3]!=0.0f;
}

inline float GroundExtractor::interpolateGrdElevation(const int& row, const int& col, const int& ptID)
{
    float dYaw0 = std::abs(_yaws(ptID) - _segBounds.at(col));
    float dYaw1 = std::abs(_yaws(ptID) - _segBounds.at(col+1));
    dYaw0 = std::max(0.01f, dYaw1/(dYaw0+dYaw1));
    dYaw1 = std::max(0.01f, 1.0f-dYaw0);
    float dRange0 = std::abs(_range2d(ptID) - _binBounds.at(row));
    float dRange1 = std::abs(_range2d(ptID) - _binBounds.at(row+1));
    dRange0 = std::max(0.01f, dRange1/(dRange0+dRange1));
    dRange1 = std::max(0.01f, 1.0f-dRange0);
    float weight_sum = (dYaw0+dRange0)*_validity[0] + (dYaw1+dRange0)*_validity[1] + (dYaw0+dRange1)*_validity[2] + (dYaw1+dRange1)*_validity[3];
    float height_sum = _eleNodes[col][row].elevation * (dYaw0+dRange0)*_validity[0]
                       + _eleNodes[(col+1)%_numSegs][row].elevation * (dYaw1+dRange0)*_validity[1]
                       + _eleNodes[col][row+1].elevation * (dYaw0+dRange1)*_validity[2]
                       + _eleNodes[(col+1)%_numSegs][row+1].elevation * (dYaw1+dRange1)*_validity[3];
    return height_sum / weight_sum;
}

inline float GroundExtractor::getSlopeAngle0(const int& row, const int& col)
{
    const int id = _ptID(row, col);
    if (id == -1) {
        return PI/2.0;
    }
    float dR = _range2d(id);
    float dZ = (*_points)(id,2) + _lidarHeight;
    return std::atan2(dZ, dR);
}

inline float GroundExtractor::getSlopeAngleExt(const int& id0, const int& id1)
{
    if (id0==-1 || id1==-1) {
        return PI/2.0;
    }
    // dZ
    float dZ = (*_points)(id1,2) - (*_points)(id0,2);
    float Rxy2_id0 = _range2d(id0)*_range2d(id0);
    float Rxy2_id1 = _range2d(id1)*_range2d(id1);
    float R2_id0 = Rxy2_id0 + (*_points)(id0,2)*(*_points)(id0,2);
    float R2_id1 = Rxy2_id1 + (*_points)(id1,2)*(*_points)(id1,2);
    float sinPhi2_id0 = (*_points)(id0,2)*(*_points)(id0,2) / R2_id0;
    float sinPhi2_id1 = (*_points)(id1,2)*(*_points)(id1,2) / R2_id1;
    float cosPhi2_id0 = 1-sinPhi2_id0;
    float cosPhi2_id1 = 1-sinPhi2_id1;
    float mZ2_id0 = (sinPhi2_id0*_m2_ra + R2_id0*cosPhi2_id0*_m2_phi);
    float mZ2_id1 = (sinPhi2_id1*_m2_ra + R2_id1*cosPhi2_id1*_m2_phi);
    float m_dZ = std::sqrt(mZ2_id0 + mZ2_id1);
    // if dZ is below noise, then return directly 0.0 as slope angle
    if (std::fabs(dZ) <= m_dZ) {
        return 0.0f;
    }
    // otherwise, compensate the dZ and dR with measurement noise (in a conservative manner)
    if (dZ > 0) {
        dZ = dZ - m_dZ;
    } else {
        dZ = dZ + m_dZ;
    }
    // dRxy
    float dX2 = (*_points)(id1,0) - (*_points)(id0,0);
    dX2 = dX2*dX2;
    float dY2 = (*_points)(id1,1) - (*_points)(id0,1);
    dY2 = dY2*dY2;
    float dxy2 = dX2 + dY2;
    float sinTheta2_id0 = (*_points)(id0,1)*(*_points)(id0,1) / Rxy2_id0;
    float sinTheta2_id1 = (*_points)(id1,1)*(*_points)(id1,1) / Rxy2_id1;
    float cosTheta2_id0 = 1 - sinTheta2_id0;
    float cosTheta2_id1 = 1 - sinTheta2_id1;
    float m2_x_id0 = cosTheta2_id0*cosPhi2_id0*_m2_ra + R2_id0*(sinTheta2_id0*cosPhi2_id0*_m2_theta + cosTheta2_id0*sinPhi2_id0*_m2_phi);
    float m2_x_id1 = cosTheta2_id1*cosPhi2_id1*_m2_ra + R2_id1*(sinTheta2_id1*cosPhi2_id1*_m2_theta + cosTheta2_id1*sinPhi2_id1*_m2_phi);
    float m2_y_id0 = sinTheta2_id0*cosPhi2_id0*_m2_ra + R2_id0*(cosTheta2_id0*cosPhi2_id0*_m2_theta + sinTheta2_id0*sinPhi2_id0*_m2_phi);
    float m2_y_id1 = sinTheta2_id1*cosPhi2_id1*_m2_ra + R2_id1*(cosTheta2_id1*cosPhi2_id1*_m2_theta + sinTheta2_id1*sinPhi2_id1*_m2_phi);
    float m_dxy = std::sqrt((dX2/dxy2)*(m2_x_id0+m2_x_id1) + (dY2/dxy2)*(m2_y_id0+m2_y_id1));
    return std::atan2(dZ, std::sqrt(dxy2)+m_dxy);
}

inline float GroundExtractor::getRadialSlopeAngleExt01(const int& row, const int& col)
{
    if (_groundImg(row-1,col) == 1) {
        return getSlopeAngleExt(_ptID(row-1,col),_ptID(row,col));
    }
    else {
        return getSlopeAngleExt(_ptID(row,col),_ptID(row+1,col));
    }
}

inline float GroundExtractor::getRadialSlopeAngleExt10(const int& row, const int& col)
{
    if (_groundImg(row+1,col) == 1) {
        return getSlopeAngleExt(_ptID(row+1,col),_ptID(row,col));
    }
    else {
        return getSlopeAngleExt(_ptID(row,col),_ptID(row-1,col));
    }
}

void GroundExtractor::projectPoints()
{
    // 1. initialize IDmapping
    for (size_t i = 0; i < _points->rows(); i++) {
        _range2d(i) = std::sqrt((*_points)(i,0)*(*_points)(i,0) + (*_points)(i,1)*(*_points)(i,1));
        int row = getBinIndexUniform(_range2d(i));
        // if this point is out of scope, add it directly to the non ground list and continue
        if (row == -1 || (std::fabs((*_points)(i,1)) < _close_region_boundary_y && (*_points)(i,0) < _close_region_boundary_x_pos && (*_points)(i,0) > _close_region_boundary_x_neg)) {
            _IDs_nonground->emplace_back(i);
        }
        // otherwise, determine its segment index and add it to the corresponding cell
        else {
            double yaw = PI - std::atan2((*_points)(i,1), (*_points)(i,0));     // [0, 2PI] clockwise starting from -X axis (X axis: driving direction)
            int col = std::floor((0.5*(yaw/PI))*_numSegs);                      // in [0, _numSegs] clockwise starting from -X axis (X axis: driving direction)
            if (col < 0) {
                col = 0;
                yaw = 0.0;
            }
            else if (col >= _numSegs) {
                col = _numSegs-1;
                yaw = 2*PI;
            }
            _IDmapping[col][row].addPoint(i, (*_points)(i,2));
            _yaws(i) = yaw;
        }
    }
    // 2. determine representative point for each cell
    for (size_t col=0; col<_numSegs; col++) {
        for (size_t row=0; row<_numBins; row++) {
            if (!_IDmapping[col][row].IDs.empty()) {
                _ptID(row, col) = _IDmapping[col][row].ptID_Zmin;
            }
        }
    }
}

inline int GroundExtractor::getBinIndexUniform(const float& range2d)
{
    if (range2d<_binBounds.front() || range2d>=_binBounds.back()) {
        return -1;
    }
    return int((range2d-_rangeMin)/_binRes);
}

void GroundExtractor::iniBinUniform()
{
    _binBounds.clear();
    _binBounds.emplace_back(_rangeMin);
    while ((_binBounds.back()+_binRes) < _rangeMax) {
        _binBounds.emplace_back(_binBounds.back() + _binRes);
    }
    _binBounds.emplace_back(_rangeMax);
    _numBins = _binBounds.size()-1;
}

} // namespace fugseg