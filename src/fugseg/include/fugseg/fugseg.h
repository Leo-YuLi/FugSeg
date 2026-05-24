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

#include <vector>
#include <Eigen/Dense>
#include "fugseg/cloud.h"
#include "fugseg/utils.h"

namespace fugseg {

class GroundExtractor
{
    public:
        /**
         * @brief Container to store the points (indices) projected to a cell
         */
        struct zIDs{
            zIDs() {}
            zIDs(const zIDs& other) {
                ptID_Zmin = other.ptID_Zmin;
                Zmin = other.Zmin;
                IDs = other.IDs;
            }
            void addPoint(const int ID, const float& Z) {
                if (Z < Zmin) {
                    Zmin = Z;
                    ptID_Zmin = ID;
                }
                IDs.emplace_back(ID);
            }
            void reset() {
                ptID_Zmin = -1;
                Zmin = 1e6f;
                IDs.clear();
            }
            // variables
            int ptID_Zmin{-1};                      // global 3D point ID of lowest point
            float Zmin{1e6f};                       // Z coordinate of lowest point
            std::vector<int> IDs;                   // IDs of all points projected to one pixel
        };
        typedef std::vector<zIDs> zIDs3D;           // e.g., points along a single column
        typedef std::vector<zIDs3D> zIDss3D;        // e.g., points matrix for the whole image

        /**
         * @brief Ground elevation node, which holds the weighted elevation from its neighbors
         */
        struct EleNode{
            EleNode() {}
            void addElevation(const float& height, const float& weight) {
                height_sum += weight*height;
                weight_sum += weight;
                // temporarily hold the latest height using elevation, used in vertical & horizontal noise propagation
                elevation = height;
            }
            void iniElevation(const int& state_) {
                if (weight_sum != 0.0f) {
                    elevation = height_sum/weight_sum;
                    state = state_;
                }
                else{
                    elevation = -1e6f;
                }
            }
            void reset() {
                weight_sum = 0.0f;
                height_sum = 0.0f;
                elevation = -1e6f;
                state = 0;
            }
            // variables
            float weight_sum{0.0f};                 // sum of all given weight
            float height_sum{0.0f};                 // sum of all weight*height pair, use height_sum/weight_sum to get the final elevation of this node
            float elevation{-1e6f};                 // final elevation of this node. Default: -1e6 means no valid ground elevation can be propagated to this node; otherwise: height_sum/weight_sum
            int state{0};                           // state of this node. 0: default, uninitialized; 1: elevation fully initialized by _groundImg==1 cells; 2: unprocessed _groundImg<0 cells (vertical); 3: _groundImg<0 cells to be propagated (vertical); 4: _groundImg<0 cells to be propagated (Horizontal); 5: valid elevation propagated to _groundImg<0 cells
        };
        typedef std::vector<EleNode> EleNodes;      // e.g., points along a single column
        typedef std::vector<EleNodes> EleNodess;    // e.g., points matrix for the whole image

    public:
        GroundExtractor(int numSegs, float binRes, float groundInlierThre, float maxDslopeDeg, float maxGapRadialLength, float rangeMin=0.5f, float rangeMax=80.0f);
        ~GroundExtractor();

    public:
        bool ExtractGround(const fugseg::Cloud& cloud, fugseg::Cluster3D& cluster_ground, fugseg::Cluster3D& cluster_others);

    private:
        // control variables
        float _close_region_boundary_x_neg{0.0f};   // rear boundary
        float _close_region_boundary_x_pos{0.0f};   // front boundary
        float _close_region_boundary_y{0.0f};       // left and right boundary
        std::vector<float> _binBounds;              // size equals num_bins + 1, with: bin_bounds_.front()=r_min and bin_bounds_.back()=r_max
        std::vector<float> _segBounds;              // size equals num_segs + 1, with: seg_bounds_.front()=0 and seg_bounds_.back()=2PI
        float _rangeMin;                            // minimum range of the projected cell, in meter
        float _rangeMax;                            // maximum range of the projected cell, in meter
        float _binRes;                              // only relevant if bin is created with uniform resolution
        int _numBins;                               // Number of radial bins in each segment, height of projected cell image
        int _numSegs;                               // Number of angular segments, width of projected cell image
        float _groundInlierThre;                    // height threshold used during fine-grained ground segmentation, in meter
        float _maxDslopeRad;                        // maximum allowed slope change, in rad
        float _maxGapRadialLength{10.0f};           // maximum radial gap length to be tolerated during vertical propagation, in meter
        // sensor related variables
        size_t _buff_points;                        // max number of points in a scan. KITTI: 360/0.18*64 < 134400
        float _lidarHeight;                         // mounting height of LiDAR sensor (>0), in meter
        float _maxHeightSeed;                       // height threshold used to select ground seed point, in meter
        float _m2_theta;                            // horizontal angular accuracy, e.g., 0.01 deg => (0.01 * pi/180)^2
        float _m2_phi;                              // vertical angular accuracy, e.g., 0.01 deg => (0.01 * pi/180)^2
        float _m2_ra;                               // range accuracy, e.g., 0.02m => 0.020^2
        // working variables
        const Eigen::ArrayX4f* _points;             // [_num_points, 4]: X, Y, Z, remission
        Eigen::ArrayXf _range2d;                    // [_num_points, 1]: 2D depth from XY
        Eigen::ArrayXf _yaws;                       // [_num_points, 1], yaw angle of the lowest point at each cell. [0, 2PI] clockwise starting from -X axis (X axis: driving direction)
        zIDss3D _IDmapping;                         // the map from each cell to the original points (as 0-based global ID)
        EleNodess _eleNodes;                        // elevation node that holds the weighted elevation from its neighbors
        Eigen::ArrayXXi _ptID;                      // [_numBins, _numSegs], look up table from pixel to its corresponding 3D point in _points list. default -1 means empty. Row-0: near
        Eigen::ArrayXXi _groundImg;                 // [_numBins, _numSegs], ground image, row-0: near. 0: default, empty + unprocessed cell + non-ground pixel; 1: ground pixel; <0: potential false reflections
        float _validity[4] {0.0f, 0.0f, 0.0f, 0.0f};// used in SegmentGroundFine(). lower-left, lower-right, upper-left, upper-right
        fugseg::Cluster3D* _IDs_ground;             // resulting list of ground points
        fugseg::Cluster3D* _IDs_nonground;          // resulting list of non-ground points

    private:
        /**
         * @brief PGM: project _points into polar cell, and initialize _ptID look-up table
         */
        void projectPoints();

        /**
         * @brief SGL: segment-wise ground labelling
         */
        void LabelGroundVertical();

        /**
         * @brief CGP: cross-segment ground propagation
         */
        void LabelGroundHorizontalJoint();

        /**
         * @brief EGE (estimation of ground elevation) + PGS (point-level ground segmentation)
         */
        void SegmentGroundFine();

        /**
         * @brief interpolate the ground elevation for point ptID within the given cell
         */
        inline float interpolateGrdElevation(const int& row, const int& col, const int& ptID);

        /**
         * @brief initialize bins using the bin resolution _binRes
         */
        void iniBinUniform();

        /**
         * @brief determine the bin index of the given 2D range. Return -1 if the given rawRange is out of scope
         */
        inline int getBinIndexUniform(const float& rawRange);

        /**
         * @brief reset working variables
         */
        void reset(const size_t& num_pts);

        /**
         * @brief check the elevation stats around the given cell, used in SegmentGroundFine()
         */
        inline bool hasValidNeighbors(const int& row, const int& col);

        /**
         * @brief calculate the 3D slope angle formed by the point vector _points(id0)->_points(id1). Return angle in rad
         */
        // getSlopeAngle0 is used for ground seeding in each segment, slope is calculated between (0,0,-lidarHeight) and the given point
        inline float getSlopeAngle0(const int& row, const int& col);
        // extended version, which compensates the calculated slope using measurement uncertainty
        inline float getSlopeAngleExt(const int& id0, const int& id1);
        // similar function used to find the radial slope between neighbor bins in the direction: near to far (i.e. slope vector towards outside)
        inline float getRadialSlopeAngleExt01(const int& row, const int& col);
        // similar function used to find the radial slope between neighbor bins in the direction: far to near (i.e. slope vector towards center)
        inline float getRadialSlopeAngleExt10(const int& row, const int& col);
};

}  // namespace fugseg

