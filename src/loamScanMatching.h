#pragma once

#include <cmath>
#include <vector>
#include <string>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>


#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include "loamFeature.h"
#include "lidarFactor.hpp"


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

// // transform all lidar points to the start of the next frame

// void TransformToEnd(PointType const *const pi, PointType *const po)
// {
//     // undistort point first
//     pcl::PointXYZI un_point_tmp;
//     TransformToStart(pi, &un_point_tmp);

//     Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
//     Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);

//     po->x = point_end.x();
//     po->y = point_end.y();
//     po->z = point_end.z();

//     //Remove distortion time info
//     po->intensity = int(pi->intensity);
// }



class LOAMScanMatching
{
public:
    LOAMScanMatching()
    {
        kdtreeCornerLast = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr{new pcl::KdTreeFLANN<pcl::PointXYZI>()};
        kdtreeSurfLast = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr{new pcl::KdTreeFLANN<pcl::PointXYZI>()};
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
            
            auto cornerPointsSharp = features.cornerPointsSharp;
            auto surfPointsFlat = features.surfPointsFlat;

            int cornerPointsSharpNum = cornerPointsSharp->points.size();
            int surfPointsFlatNum = surfPointsFlat->points.size();

            TicToc t_opt;
            int corner_correspondence = 0;
            int plane_correspondence = 0;
            for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter)
            {

                //ceres::LossFunction *loss_function = NULL;
                ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                ceres::LocalParameterization *q_parameterization =
                    new ceres::EigenQuaternionParameterization();
                ceres::Problem::Options problem_options;

                ceres::Problem problem(problem_options);
                problem.AddParameterBlock(para_q, 4, q_parameterization);
                problem.AddParameterBlock(para_t, 3);

                pcl::PointXYZI pointSel;
                std::vector<int> pointSearchInd;
                std::vector<float> pointSearchSqDis;

                TicToc t_data;
                // find correspondence for corner features
                for (int i = 0; i < cornerPointsSharpNum; ++i)
                {
                    // break;

                    TransformToStart(q_last_curr, t_last_curr, &(cornerPointsSharp->points[i]), &pointSel);
                    kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                    int closestPointInd = -1, minPointInd2 = -1;
                    if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                    {
                        closestPointInd = pointSearchInd[0];
                        int closestPointScanID = int(laserCloudCornerLast->points[closestPointInd].intensity);

                        double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
                        // search in the direction of increasing scan line
                        for (int j = closestPointInd + 1; j < (int)laserCloudCornerLast->points.size(); ++j)
                        {
                            // if in the same scan line, continue
                            if (int(laserCloudCornerLast->points[j].intensity) <= closestPointScanID)
                                continue;

                            // if not in nearby scans, end the loop
                            if (int(laserCloudCornerLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                break;

                            double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                    (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                    (laserCloudCornerLast->points[j].z - pointSel.z);

                            if (pointSqDis < minPointSqDis2)
                            {
                                // find nearer point
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                        }

                        // search in the direction of decreasing scan line
                        for (int j = closestPointInd - 1; j >= 0; --j)
                        {
                            // if in the same scan line, continue
                            if (int(laserCloudCornerLast->points[j].intensity) >= closestPointScanID)
                                continue;

                            // if not in nearby scans, end the loop
                            if (int(laserCloudCornerLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                break;

                            double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                    (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                    (laserCloudCornerLast->points[j].z - pointSel.z);

                            if (pointSqDis < minPointSqDis2)
                            {
                                // find nearer point
                                minPointSqDis2 = pointSqDis;
                                minPointInd2 = j;
                            }
                        }
                    }
                    if (minPointInd2 >= 0) // both closestPointInd and minPointInd2 is valid
                    {
                        Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x,
                                                    cornerPointsSharp->points[i].y,
                                                    cornerPointsSharp->points[i].z);
                        Eigen::Vector3d last_point_a(laserCloudCornerLast->points[closestPointInd].x,
                                                        laserCloudCornerLast->points[closestPointInd].y,
                                                        laserCloudCornerLast->points[closestPointInd].z);
                        Eigen::Vector3d last_point_b(laserCloudCornerLast->points[minPointInd2].x,
                                                        laserCloudCornerLast->points[minPointInd2].y,
                                                        laserCloudCornerLast->points[minPointInd2].z);

                        double s;
                        if (DISTORTION)
                            s = (cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) / SCAN_PERIOD;
                        else
                            s = 1.0;
                        ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
                        problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                        corner_correspondence++;
                    }
                }

                // find correspondence for plane features
                assert(laserCloudSurfLast != nullptr);
                assert(laserCloudCornerLast != nullptr);
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
                            ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                            plane_correspondence++;
                        }
                    }
                }

                //printf("coner_correspondance %d, plane_correspondence %d \n", corner_correspondence, plane_correspondence);
                printf("data association time %f ms \n", t_data.toc());

                if ((corner_correspondence + plane_correspondence) < 10)
                {
                    printf("less correspondence! *************************************************\n");
                }

                TicToc t_solver;
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.max_num_iterations = 4;
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
    double para_t[3] = {0, 0, 0};

    bool isInit_ = false;
    LidarFeatures lastFeature_;

    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast = nullptr;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast = nullptr;


};

