#pragma once

#include "loamFeature.h"
#include "laggedMatcher.h"

namespace loam
{

class LOAM
{
public:
    void match(const pcl::PointCloud<pcl::PointXYZI> &scan)
    {
        curScan_ = KittiDataPreprocessing().convertScanToVectorOfLinesAzi(scan);

        // LidarFeatures features = LidarFeatureExtraction().extractFeatures(lines);
        // LidarFeatures features = LidarFeatureExtraction().extractFeatures2(lines);
        curfeatures_ = LidarFeatureExtraction().extractFeaturesSimple(curScan_);

        matcher_.match(curfeatures_);


    }

    void getGlobalPose(Eigen::Quaterniond & q, Eigen::Vector3d &t)
    {
        matcher_.getPose(q, t);
    }


    std::vector<pcl::PointCloud<PointType>> curScan_;
    LidarFeatures curfeatures_;

    // Has internal states
    lagged::LaggedScanMatching matcher_;

};

}