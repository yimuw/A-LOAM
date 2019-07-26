#pragma once

#include <cmath>
#include <vector>
#include <string>
#include <list>
#include <memory>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>


#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include "loamFeature.h"
#include "lidarFactor.hpp"



namespace lagged
{

#define DISTORTION 0
constexpr double SCAN_PERIOD = 0.1;
constexpr double DISTANCE_SQ_THRESHOLD = 25;
constexpr double NEARBY_SCAN = 2.5;
double DUMMY_PTR[4];


// undistort lidar point
void TransformToStart(const Eigen::Quaterniond &q_last_curr,
                      const Eigen::Vector3d & t_last_curr,
                      PointType const *const pi, 
                      PointType *const po)
{
    //interpolation ratio
    double s;
    if (DISTORTION)
        s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD;
    else
        s = 1.0;
    //s = 1;
    Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
    Eigen::Vector3d t_point_last = s * t_last_curr;
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    Eigen::Vector3d un_point = q_point_last * point + t_point_last;

    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->intensity = pi->intensity;
}





class LaggedScanMatching
{
public:

    struct LaggedData
    {
        LaggedData()
        {
            pointCloudKdtree = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr{new pcl::KdTreeFLANN<pcl::PointXYZI>()};
        }

        void getPose(Eigen::Quaterniond & q, Eigen::Vector3d &t)
        {
            Eigen::Map<Eigen::Quaterniond> q_map(para_q);
            Eigen::Map<Eigen::Vector3d> t_map(para_t);

            q = q_map;
            t = t_map;
        }

        size_t id = 0;

        // parameters
        double para_q[4] = {0, 0, 0, 1};
        double para_t[3] = {0, 0, 0};

        Eigen::Quaterniond q_w{1, 0, 0, 0};
        Eigen::Vector3d t_w{0, 0, 0};

        pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr pointCloudKdtree;
        pcl::PointCloud<PointType>::Ptr pointCloud = nullptr;
    };

    LaggedScanMatching()
    {
        kdtreeCornerLast = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr{new pcl::KdTreeFLANN<pcl::PointXYZI>()};
        kdtreeSurfLast = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr{new pcl::KdTreeFLANN<pcl::PointXYZI>()};
    }

    // void associateEdges(const LidarFeatures &features, ceres::Problem &problem)
    // {
    //     Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);
    //     Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);

    //     auto cornerPointsSharp = features.cornerPointsSharp;
    //     int cornerPointsSharpNum = cornerPointsSharp->points.size();

    //     int corner_correspondence = 0;
        
    //     pcl::PointXYZI pointSel;
    //     std::vector<int> pointSearchInd;
    //     std::vector<float> pointSearchSqDis;

    //     // find correspondence for corner features
    //     for (int i = 0; i < cornerPointsSharpNum; ++i)
    //     {
    //         // break;

    //         TransformToStart(q_last_curr, t_last_curr, &(cornerPointsSharp->points[i]), &pointSel);
    //         kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

    //         int closestPointInd = -1, minPointInd2 = -1;
    //         if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
    //         {
    //             closestPointInd = pointSearchInd[0];
    //             int closestPointScanID = int(laserCloudCornerLast->points[closestPointInd].intensity);

    //             double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
    //             // search in the direction of increasing scan line
    //             for (int j = closestPointInd + 1; j < (int)laserCloudCornerLast->points.size(); ++j)
    //             {
    //                 // if in the same scan line, continue
    //                 if (int(laserCloudCornerLast->points[j].intensity) <= closestPointScanID)
    //                     continue;

    //                 // if not in nearby scans, end the loop
    //                 if (int(laserCloudCornerLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
    //                     break;

    //                 double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
    //                                         (laserCloudCornerLast->points[j].x - pointSel.x) +
    //                                     (laserCloudCornerLast->points[j].y - pointSel.y) *
    //                                         (laserCloudCornerLast->points[j].y - pointSel.y) +
    //                                     (laserCloudCornerLast->points[j].z - pointSel.z) *
    //                                         (laserCloudCornerLast->points[j].z - pointSel.z);

    //                 if (pointSqDis < minPointSqDis2)
    //                 {
    //                     // find nearer point
    //                     minPointSqDis2 = pointSqDis;
    //                     minPointInd2 = j;
    //                 }
    //             }

    //             // search in the direction of decreasing scan line
    //             for (int j = closestPointInd - 1; j >= 0; --j)
    //             {
    //                 // if in the same scan line, continue
    //                 if (int(laserCloudCornerLast->points[j].intensity) >= closestPointScanID)
    //                     continue;

    //                 // if not in nearby scans, end the loop
    //                 if (int(laserCloudCornerLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
    //                     break;

    //                 double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
    //                                         (laserCloudCornerLast->points[j].x - pointSel.x) +
    //                                     (laserCloudCornerLast->points[j].y - pointSel.y) *
    //                                         (laserCloudCornerLast->points[j].y - pointSel.y) +
    //                                     (laserCloudCornerLast->points[j].z - pointSel.z) *
    //                                         (laserCloudCornerLast->points[j].z - pointSel.z);

    //                 if (pointSqDis < minPointSqDis2)
    //                 {
    //                     // find nearer point
    //                     minPointSqDis2 = pointSqDis;
    //                     minPointInd2 = j;
    //                 }
    //             }
    //         }
    //         if (minPointInd2 >= 0) // both closestPointInd and minPointInd2 is valid
    //         {
    //             Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x,
    //                                         cornerPointsSharp->points[i].y,
    //                                         cornerPointsSharp->points[i].z);
    //             Eigen::Vector3d last_point_a(laserCloudCornerLast->points[closestPointInd].x,
    //                                             laserCloudCornerLast->points[closestPointInd].y,
    //                                             laserCloudCornerLast->points[closestPointInd].z);
    //             Eigen::Vector3d last_point_b(laserCloudCornerLast->points[minPointInd2].x,
    //                                             laserCloudCornerLast->points[minPointInd2].y,
    //                                             laserCloudCornerLast->points[minPointInd2].z);

    //             double s;
    //             if (DISTORTION)
    //                 s = (cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) / SCAN_PERIOD;
    //             else
    //                 s = 1.0;

    //             ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
    //             ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
    //             problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
    //             corner_correspondence++;
    //         }
    //     }

    //     printf("coner_correspondance %d\n", corner_correspondence);
    // }

    void plainAssociation(
        const LidarFeatures &features, 
        ceres::Problem &problem)
    {
        Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);
        Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);

        // Eigen::Quaterniond q_curr2last = q_last_curr.inverse();
        // Eigen::Vector3d t_curr2last = -t_last_curr;

        auto surfPointsFlat = features.surfPointsFlat;
        int surfPointsFlatNum = surfPointsFlat->points.size();

        // find correspondence for plane features
        int plane_correspondence = 0;

        pcl::PointXYZI pointSel;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        for (int i = 0; i < surfPointsFlatNum; ++i)
        {
            TransformToStart(q_last_curr, t_last_curr, &(surfPointsFlat->points[i]), &pointSel);
            kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

            int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
            if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
            {
                closestPointInd = pointSearchInd[0];

                // get closest point's scan ID
                int closestPointScanID = int(laserCloudSurfLast->points[closestPointInd].intensity);
                double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

                // search in the direction of increasing scan line
                for (int j = closestPointInd + 1; j < (int)laserCloudSurfLast->points.size(); ++j)
                {
                    // if not in nearby scans, end the loop
                    if (int(laserCloudSurfLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                        break;

                    double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                            (laserCloudSurfLast->points[j].x - pointSel.x) +
                                        (laserCloudSurfLast->points[j].y - pointSel.y) *
                                            (laserCloudSurfLast->points[j].y - pointSel.y) +
                                        (laserCloudSurfLast->points[j].z - pointSel.z) *
                                            (laserCloudSurfLast->points[j].z - pointSel.z);

                    // if in the same or lower scan line
                    if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
                    {
                        minPointSqDis2 = pointSqDis;
                        minPointInd2 = j;
                    }
                    // if in the higher scan line
                    else if (int(laserCloudSurfLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)
                    {
                        minPointSqDis3 = pointSqDis;
                        minPointInd3 = j;
                    }
                }

                // search in the direction of decreasing scan line
                for (int j = closestPointInd - 1; j >= 0; --j)
                {
                    // if not in nearby scans, end the loop
                    if (int(laserCloudSurfLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                        break;

                    double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                            (laserCloudSurfLast->points[j].x - pointSel.x) +
                                        (laserCloudSurfLast->points[j].y - pointSel.y) *
                                            (laserCloudSurfLast->points[j].y - pointSel.y) +
                                        (laserCloudSurfLast->points[j].z - pointSel.z) *
                                            (laserCloudSurfLast->points[j].z - pointSel.z);

                    // if in the same or higher scan line
                    if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
                    {
                        minPointSqDis2 = pointSqDis;
                        minPointInd2 = j;
                    }
                    else if (int(laserCloudSurfLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
                    {
                        // find nearer point
                        minPointSqDis3 = pointSqDis;
                        minPointInd3 = j;
                    }
                }

                if (minPointInd2 >= 0 && minPointInd3 >= 0)
                {

                    Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,
                                                surfPointsFlat->points[i].y,
                                                surfPointsFlat->points[i].z);
                    Eigen::Vector3d last_point_a(laserCloudSurfLast->points[closestPointInd].x,
                                                    laserCloudSurfLast->points[closestPointInd].y,
                                                    laserCloudSurfLast->points[closestPointInd].z);
                    Eigen::Vector3d last_point_b(laserCloudSurfLast->points[minPointInd2].x,
                                                    laserCloudSurfLast->points[minPointInd2].y,
                                                    laserCloudSurfLast->points[minPointInd2].z);
                    Eigen::Vector3d last_point_c(laserCloudSurfLast->points[minPointInd3].x,
                                                    laserCloudSurfLast->points[minPointInd3].y,
                                                    laserCloudSurfLast->points[minPointInd3].z);

                    double s;
                    if (DISTORTION)
                        s = (surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) / SCAN_PERIOD;
                    else
                        s = 1.0;
                    
                    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                    ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                    problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                    plane_correspondence++;
                }
            }
        }
        printf("plane_correspondence %d \n", plane_correspondence);
    }


    void plainAssociationLagged(
        const LidarFeatures &features, 
        const LaggedData &lag_data,
        ceres::Problem &problem)
    {
        // curr to last curr
        Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);
        Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);

        // last curr to origin
        Eigen::Matrix4d T_lcur2w = qtToMat(q_w_curr, t_w_curr);
        // cur to last_cur
        Eigen::Matrix4d T_cur2lcur = qtToMat(q_last_curr, t_last_curr);
        
        // curr to origin
        const Eigen::Matrix4d T_cur2w = T_lcur2w * T_cur2lcur;

        Eigen::Matrix4d T_lagged2w = qtToMat(lag_data.q_w, lag_data.t_w);

        // T(cur to lagged) = T(origin to lagged) * T(curr to origin)
        Eigen::Matrix4d T_cur2lagged = T_lagged2w.inverse() * T_cur2w;
        // T(lagged to last_cur) = T(o to last_cur) * T(lagged to o)
        Eigen::Matrix4d T_lagged2lcur = T_lcur2w.inverse() * T_lagged2w;

        std::cout << "T_cur2lagged: " << T_cur2lagged << std::endl;
        std::cout << "T_lagged2lcur: " << T_lagged2lcur << std::endl;

        Eigen::Quaterniond q_cur2lagged;
        Eigen::Vector3d t_cur2lagged;
        MatToqt(T_cur2lagged, q_cur2lagged, t_cur2lagged);

        auto surfPointsFlat = features.surfPointsFlat;
        int surfPointsFlatNum = surfPointsFlat->points.size();

        // find correspondence for plane features
        int plane_correspondence = 0;

        pcl::PointXYZI pointSel;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        for (int i = 0; i < surfPointsFlatNum; ++i)
        {
            TransformToStart(q_cur2lagged, t_cur2lagged, &(surfPointsFlat->points[i]), &pointSel);
            lag_data.pointCloudKdtree->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

            int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;

            constexpr double DISTANCE_SQ_THRESHOLD = 1.;
            
            if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
            {
                closestPointInd = pointSearchInd[0];

                // get closest point's scan ID
                int closestPointScanID = int(lag_data.pointCloud->points[closestPointInd].intensity);
                double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

                // search in the direction of increasing scan line
                for (int j = closestPointInd + 1; j < (int)lag_data.pointCloud->points.size(); ++j)
                {
                    // if not in nearby scans, end the loop
                    if (int(lag_data.pointCloud->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                        break;

                    double pointSqDis = (lag_data.pointCloud->points[j].x - pointSel.x) *
                                            (lag_data.pointCloud->points[j].x - pointSel.x) +
                                        (lag_data.pointCloud->points[j].y - pointSel.y) *
                                            (lag_data.pointCloud->points[j].y - pointSel.y) +
                                        (lag_data.pointCloud->points[j].z - pointSel.z) *
                                            (lag_data.pointCloud->points[j].z - pointSel.z);

                    // if in the same or lower scan line
                    if (int(lag_data.pointCloud->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
                    {
                        minPointSqDis2 = pointSqDis;
                        minPointInd2 = j;
                    }
                    // if in the higher scan line
                    else if (int(lag_data.pointCloud->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)
                    {
                        minPointSqDis3 = pointSqDis;
                        minPointInd3 = j;
                    }
                }

                // search in the direction of decreasing scan line
                for (int j = closestPointInd - 1; j >= 0; --j)
                {
                    // if not in nearby scans, end the loop
                    if (int(lag_data.pointCloud->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                        break;

                    double pointSqDis = (lag_data.pointCloud->points[j].x - pointSel.x) *
                                            (lag_data.pointCloud->points[j].x - pointSel.x) +
                                        (lag_data.pointCloud->points[j].y - pointSel.y) *
                                            (lag_data.pointCloud->points[j].y - pointSel.y) +
                                        (lag_data.pointCloud->points[j].z - pointSel.z) *
                                            (lag_data.pointCloud->points[j].z - pointSel.z);

                    // if in the same or higher scan line
                    if (int(lag_data.pointCloud->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
                    {
                        minPointSqDis2 = pointSqDis;
                        minPointInd2 = j;
                    }
                    else if (int(lag_data.pointCloud->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
                    {
                        // find nearer point
                        minPointSqDis3 = pointSqDis;
                        minPointInd3 = j;
                    }
                }

                if (minPointInd2 >= 0 && minPointInd3 >= 0)
                {

                    Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,
                                                surfPointsFlat->points[i].y,
                                                surfPointsFlat->points[i].z);

                    auto get_point_in_lcurr_frame = [T_lagged2lcur](const Eigen::Vector3d &v)
                    {
                        Eigen::Matrix3d R = T_lagged2lcur.block(0,0,3,3);
                        Eigen::Vector3d t = T_lagged2lcur.block(0,3,3,1);
                        Eigen::Vector3d res = R * v + t;

                        return res;
                    };

                    Eigen::Vector3d last_point_a(lag_data.pointCloud->points[closestPointInd].x,
                                                    lag_data.pointCloud->points[closestPointInd].y,
                                                    lag_data.pointCloud->points[closestPointInd].z);
                    last_point_a = get_point_in_lcurr_frame(last_point_a);

                    Eigen::Vector3d last_point_b(lag_data.pointCloud->points[minPointInd2].x,
                                                    lag_data.pointCloud->points[minPointInd2].y,
                                                    lag_data.pointCloud->points[minPointInd2].z);
                    last_point_b = get_point_in_lcurr_frame(last_point_b);

                    Eigen::Vector3d last_point_c(lag_data.pointCloud->points[minPointInd3].x,
                                                    lag_data.pointCloud->points[minPointInd3].y,
                                                    lag_data.pointCloud->points[minPointInd3].z);
                    last_point_c = get_point_in_lcurr_frame(last_point_c);


                    double s;
                    if (DISTORTION)
                        s = (surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) / SCAN_PERIOD;
                    else
                        s = 1.0;
                    
                    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                    ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                    problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                    plane_correspondence++;
                }
            }
        }
        printf("plane_correspondence %d \n", plane_correspondence);
    }

    void associatedLaggedPoints(const LidarFeatures &features, ceres::Problem &problem)
    {
        if(lagged_data_.size() < 10)
        {
            return;
        }

        // curr to last curr
        Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);
        Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);

        // last curr to origin
        Eigen::Matrix4d T_lcur2w = qtToMat(q_w_curr, t_w_curr);
        // cur to last_cur
        Eigen::Matrix4d T_cur2lcur = qtToMat(q_last_curr, t_last_curr);
        
        // curr to origin
        const Eigen::Matrix4d T_cur2w = T_lcur2w * T_cur2lcur;


        pcl::PointXYZI pointSel;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        auto &surfPointsFlat = features.surfPointsFlat;
        int surfPointsFlatNum = surfPointsFlat->points.size();

        size_t point_correspondence = 0;

        constexpr size_t MAX_I = 5;

        std::cout << "lagged_data_.size():" << lagged_data_.size() << std::endl;
        size_t i = 0;
        auto data_iter = lagged_data_.begin();
        for(; i < MAX_I && data_iter != lagged_data_.end(); ++i, ++data_iter)
        {
            auto &lagged_data = *data_iter;
            std::cout << "i: " << i << std::endl;
            std::cout << "lagged_data->t_w: " << lagged_data->t_w << std::endl;
            // lagged to origin
            Eigen::Matrix4d T_lagged2w = qtToMat(lagged_data->q_w, lagged_data->t_w);

            // T(cur to lagged) = T(origin to lagged) * T(curr to origin)
            Eigen::Matrix4d T_cur2lagged = T_lagged2w.inverse() * T_cur2w;
            // T(lagged to last_cur) = T(o to last_cur) * T(lagged to o)
            Eigen::Matrix4d T_lagged2lcur = T_lcur2w.inverse() * T_lagged2w;

            Eigen::Quaterniond q_cur2lagged;
            Eigen::Vector3d t_cur2lagged;
            MatToqt(T_cur2lagged, q_cur2lagged, t_cur2lagged);

            std::cout << "T_cur2lagged: " << T_cur2lagged << std::endl;
            std::cout << "T_lagged2lcur: " << T_lagged2lcur << std::endl;

            for (int i = 0; i < surfPointsFlatNum; ++i)
            {
                TransformToStart(q_cur2lagged, t_cur2lagged, &(surfPointsFlat->points[i]), &pointSel);
                lagged_data->pointCloudKdtree->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                constexpr double DISTANCE_SQ_THRESHOLD_POINT_TO_POINT = 0.2 * 0.2;
                if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD_POINT_TO_POINT)
                {
                    int closestPointInd = pointSearchInd[0];

                    Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,
                                                surfPointsFlat->points[i].y,
                                                surfPointsFlat->points[i].z);

                    
                    Eigen::Vector4d close_lagged_frame(lagged_data->pointCloud->points[closestPointInd].x,
                                                    lagged_data->pointCloud->points[closestPointInd].y,
                                                    lagged_data->pointCloud->points[closestPointInd].z,
                                                    1.);
                    Eigen::Vector4d close_lcurr_frame_homo = T_lagged2lcur * close_lagged_frame;
                    Eigen::Vector3d close_lcurr_frame = close_lcurr_frame_homo.head<3>();
                    
                    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                    ceres::CostFunction *cost_function = LidarPointFactor::Create(curr_point, close_lcurr_frame);
                    problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                    point_correspondence++;
                    
                }

            }
        }

        std::cout << "point_correspondence : " << point_correspondence << std::endl;
    }

    void match(const LidarFeatures &features)
    {
        if(not isInit_)
        {
            isInit_ = true;
            std::cout << "LOAMScanMatching init " << std::endl;
        }
        else
        {
            Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);
            Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);

            TicToc t_opt;

            for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter)
            {
                //ceres::LossFunction *loss_function = NULL;
                ceres::LocalParameterization *q_parameterization =
                    new ceres::EigenQuaternionParameterization();
                ceres::Problem::Options problem_options;

                ceres::Problem problem(problem_options);
                problem.AddParameterBlock(para_q, 4, q_parameterization);
                problem.AddParameterBlock(para_t, 3);


                TicToc t_data;
                // associateEdges(features, problem);
                // plainAssociation(features, problem);
                // associatedLaggedPoints(features, problem);

                size_t lagged_count = 0;
                for(const auto &lagged: lagged_data_)
                {
                    std::cout << "lagged_count:" << lagged_count << std::endl;
                    plainAssociationLagged(features, *lagged, problem);
                    lagged_count++;
                    if(lagged_count == 1)
                    {
                        break;
                    }
                }

                //printf("coner_correspondance %d, plane_correspondence %d \n", corner_correspondence, plane_correspondence);
                printf("data association time %f ms \n", t_data.toc());


                TicToc t_solver;
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.max_num_iterations = 5;
                options.minimizer_progress_to_stdout = false;
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);
                printf("solver time %f ms \n", t_solver.toc());
                std::cout << " para_t: " <<  para_t[0] << ", " <<  para_t[1] << ", " <<  para_t[2] << std::endl;
            }
            
            printf("optimization twice time %f \n", t_opt.toc());
            t_w_curr = t_w_curr + q_w_curr * t_last_curr;
            q_w_curr = q_w_curr * q_last_curr;
        }

        lastFeature_ = features;
        kdtreeCornerLast->setInputCloud(lastFeature_.cornerPointsSharp);
        kdtreeSurfLast->setInputCloud(lastFeature_.surfPointsFlat);

        laserCloudCornerLast = lastFeature_.cornerPointsSharp;
        laserCloudSurfLast = lastFeature_.surfPointsFlat;

        std::shared_ptr<LaggedData> lagged = std::make_shared<LaggedData>();
        static size_t frame_id = 0;

        lagged->id = frame_id++;
        lagged->q_w = q_w_curr;
        lagged->t_w = t_w_curr;
        // // TODO: point to
        lagged->pointCloudKdtree = kdtreeSurfLast;
        lagged->pointCloud = lastFeature_.surfPointsFlat;

        lagged_data_.push_front(lagged);

        if(lagged_data_.size() > 30)
        {
            lagged_data_.pop_back();
        }
    }

    void getPose(Eigen::Quaterniond & q, Eigen::Vector3d &t)
    {
        q = q_w_curr;
        t = t_w_curr;
    }
    

private:
    // Transformation from current frame to world frame
    Eigen::Quaterniond q_w_curr {1, 0, 0, 0};
    Eigen::Vector3d t_w_curr {0, 0, 0};

    // q_curr_last(x, y, z, w), t_curr_last
    double para_q[4] = {0, 0, 0, 1};
    double para_t[3] = {1., 0, 0};

    bool isInit_ = false;
    LidarFeatures lastFeature_;

    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast = nullptr;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast = nullptr;

    std::list<std::shared_ptr<LaggedData>> lagged_data_;
};

}


