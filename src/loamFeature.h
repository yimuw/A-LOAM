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


// using PointType = pcl::PointXYZ;
constexpr int N_SCANS = 64;


class KittiDataPreprocessing
{
public:
    template <typename PointT>
    void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                                pcl::PointCloud<PointT> &cloud_out, 
                                float thres)
    {
        if (&cloud_in != &cloud_out)
        {
            cloud_out.header = cloud_in.header;
            cloud_out.points.resize(cloud_in.points.size());
        }

        size_t j = 0;

        for (size_t i = 0; i < cloud_in.points.size(); ++i)
        {
            if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
                continue;
            cloud_out.points[j] = cloud_in.points[i];
            j++;
        }
        if (j != cloud_in.points.size())
        {
            cloud_out.points.resize(j);
        }

        cloud_out.height = 1;
        cloud_out.width = static_cast<uint32_t>(j);
        cloud_out.is_dense = true;
    }

    // make a copy of pc
    std::vector<pcl::PointCloud<PointType>> convertScanToVectorOfLines(pcl::PointCloud<PointType> laserCloudIn)
    {    
        TicToc t_prepare;
        std::vector<int> scanStartInd(N_SCANS, 0);
        std::vector<int> scanEndInd(N_SCANS, 0);
        std::vector<int> indices;

        pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);

        constexpr double MINIMUM_RANGE = 1.;
        this->removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);


        int cloudSize = laserCloudIn.points.size();
        // float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
        // float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
        //                     laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI;

        // std::cout << "startOri:" << atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x) << std::endl;
        // std::cout << "endOri:" << atan2(laserCloudIn.points[cloudSize - 1].y,
        //                     laserCloudIn.points[cloudSize - 1].x) << std::endl;


        // if (endOri - startOri > 3 * M_PI)
        // {
        //     endOri -= 2 * M_PI;
        // }
        // else if (endOri - startOri < M_PI)
        // {
        //     endOri += 2 * M_PI;
        // }
        //printf("end Ori %f\n", endOri);

        // bool halfPassed = false;
        int count = cloudSize;
        PointType point;
        std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
        for (int i = 0; i < cloudSize; i++)
        {
            point.x = laserCloudIn.points[i].x;
            point.y = laserCloudIn.points[i].y;
            point.z = laserCloudIn.points[i].z;

            float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
            int scanID = 0;

            if (N_SCANS == 16)
            {
                scanID = int((angle + 15) / 2 + 0.5);
                if (scanID > (N_SCANS - 1) || scanID < 0)
                {
                    count--;
                    continue;
                }
            }
            else if (N_SCANS == 32)
            {
                scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
                if (scanID > (N_SCANS - 1) || scanID < 0)
                {
                    count--;
                    continue;
                }
            }
            else if (N_SCANS == 64)
            {   
                if (angle >= -8.83)
                    scanID = int((2 - angle) * 3.0 + 0.5);
                else
                    scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

                // use [0 50]  > 50 remove outlies 
                if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
                {
                    count--;
                    continue;
                }
            }
            else
            {
                printf("wrong scan number\n");
                ROS_BREAK();
            }
            //printf("angle %f scanID %d \n", angle, scanID);

            // float ori = -atan2(point.y, point.x);
            // if (!halfPassed)
            // { 
            //     if (ori < startOri - M_PI / 2)
            //     {
            //         ori += 2 * M_PI;
            //     }
            //     else if (ori > startOri + M_PI * 3 / 2)
            //     {
            //         ori -= 2 * M_PI;
            //     }

            //     if (ori - startOri > M_PI)
            //     {
            //         halfPassed = true;
            //     }
            // }
            // else
            // {
            //     ori += 2 * M_PI;
            //     if (ori < endOri - M_PI * 3 / 2)
            //     {
            //         ori += 2 * M_PI;
            //     }
            //     else if (ori > endOri + M_PI / 2)
            //     {
            //         ori -= 2 * M_PI;
            //     }
            // }

            float relTime = 0; // (ori - startOri) / (endOri - startOri);

            const double scanPeriod = 0.1;
            point.intensity = scanID + scanPeriod * relTime;

            // std::cout << "id:" << scanID << std::endl;
            // std::cout << "azu: " << atan2(laserCloudIn.points[i].y, laserCloudIn.points[i].x) << std::endl;
            
            laserCloudScans[scanID].push_back(point); 
        }
        
        cloudSize = count;
        printf("points size %d \n", cloudSize);
        printf("prepare time %f \n", t_prepare.toc());

        return laserCloudScans;
    }

    // make a copy of pc
    std::vector<pcl::PointCloud<PointType>> convertScanToVectorOfLinesAzi(pcl::PointCloud<PointType> laserCloudIn)
    {    
        TicToc t_prepare;
        std::vector<int> scanStartInd(N_SCANS, 0);
        std::vector<int> scanEndInd(N_SCANS, 0);
        std::vector<int> indices;

        pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);

        constexpr double MINIMUM_RANGE = 1.;
        this->removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);


        int cloudSize = laserCloudIn.points.size();

        PointType point;
        std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);

        float last_azi = 1e-8;
        double azi_accu = 0;
        int scanID = 0;
        for (int i = 0; i < cloudSize; i++)
        {

            point.x = laserCloudIn.points[i].x;
            point.y = laserCloudIn.points[i].y;
            point.z = laserCloudIn.points[i].z;

            float azi = std::atan2(point.y, point.x);
            if(azi < 0)
            {
                azi += 2 * M_PI;
            }
  
            if( (M_PI < last_azi && last_azi < 2 * M_PI) && (azi > 0 && azi < M_PI) && azi_accu > 2 * M_PI - 1)
            {
                scanID++;
                // std::cout << "i:" << i  << " azi_accu:" << azi_accu << std::endl;
                azi_accu = 0;
                last_azi = 1e-8;
            }

            azi_accu += azi - last_azi;

            // std::cout << "azi:" << azi << " last_azi:" << last_azi << " azi_accu:" << azi_accu << std::endl;
            last_azi = azi;
            

            point.intensity = scanID;

            if(scanID == N_SCANS)
            {
                std::cout << "WARNING!!! scanID > N_SCANS" << "  i:" << i << std::endl;
                break;
            }

            laserCloudScans.at(scanID).push_back(point); 
        }

        std::cout << " scanID end:" << scanID << std::endl;
        printf("prepare time %f \n", t_prepare.toc());

        return laserCloudScans;
    }
};


struct LidarFeatures
{
    LidarFeatures()
    {
        cornerPointsSharp = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
        cornerPointsLessSharp = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
        surfPointsFlat = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
        surfPointsLessFlat = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    }


    pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
    pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
    pcl::PointCloud<PointType>::Ptr surfPointsFlat;
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;
};


// can't put huge data in stack
float cloudCurvature[400000];
int cloudSortInd[400000];
int cloudNeighborPicked[400000];
int cloudLabel[400000];


class LidarFeatureExtraction
{
public:

    LidarFeatures extractFeatures(const std::vector<pcl::PointCloud<PointType>> &laserCloudScans)
    {
        std::vector<int> scanStartInd(N_SCANS, 0);
        std::vector<int> scanEndInd(N_SCANS, 0);

        pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());

        assert(laserCloudScans.size() == N_SCANS);
        std::cout << "laserCloudScans.size(): " << laserCloudScans.size() << std::endl;
        
        for (int i = 0; i < N_SCANS; i++)
        { 
            scanStartInd[i] = laserCloud->size() + 5;
            *laserCloud += laserCloudScans[i];
            scanEndInd[i] = laserCloud->size() - 6;
        }
        const size_t cloudSize = laserCloud->size();
        std::cout << "cloudSize: " << cloudSize << std::endl;

        // Confusing. Compute curv for all point, but use it within a scan
        for (size_t i = 5; i < cloudSize - 5; i++)
        { 
            float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
            float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
            float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

            cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
            cloudSortInd[i] = i;
            cloudNeighborPicked[i] = 0;
            cloudLabel[i] = 0;
        }

        // continuous checks
        for (size_t i = 5; i < cloudSize - 4; i++)
        { 
            Eigen::Vector3d p1{laserCloud->points[i].x, laserCloud->points[i].y, laserCloud->points[i].z};
            Eigen::Vector3d p2{laserCloud->points[i + 1].x, laserCloud->points[i + 1].y, laserCloud->points[i + 1].z};

            if( (p1 - p2).norm() > 0.3)
            {
                for (int l = 0; l <= 5; l++)
                {
                    cloudNeighborPicked[i + l] = 1;
                    cloudNeighborPicked[i - l] = 1;
                }
            }
        }

        std::cout << "compute curv done." << std::endl;

        TicToc t_pts;

        LidarFeatures features;
        float t_q_sort = 0;
        for (int i = 0; i < N_SCANS; i++)
        {
            if( scanEndInd[i] - scanStartInd[i] < 6)
            {
                // std::cout << "warning: scanEndInd[i] - scanStartInd[i] < 6" << std::endl;
                continue;
            }
            pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);

            // 6 sub-region for feature extration
            for (int j = 0; j < 6; j++)
            {
                // sp: start position   ep: end position
                int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6; 
                int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

                TicToc t_tmp;
                std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, [](int i,int j)
                {
                    return (::cloudCurvature[i]<::cloudCurvature[j]);
                });

                t_q_sort += t_tmp.toc();

                // edge points
                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--)
                {
                    int ind = cloudSortInd[k]; 

                    if (cloudNeighborPicked[ind] == 0 &&
                        cloudCurvature[ind] > 0.1)
                    {

                        largestPickedNum++;
                        if (largestPickedNum <= 2)
                        {                        
                            cloudLabel[ind] = 2;
                            features.cornerPointsSharp->push_back(laserCloud->points[ind]);
                            features.cornerPointsLessSharp->push_back(laserCloud->points[ind]);
                        }
                        else if (largestPickedNum <= 20)
                        {                        
                            cloudLabel[ind] = 1; 
                            features.cornerPointsLessSharp->push_back(laserCloud->points[ind]);
                        }
                        else
                        {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1; 

                        for (int l = 1; l <= 5; l++)
                        {
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                            {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                            {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                // surf points
                int smallestPickedNum = 0;
                for (int k = sp; k <= ep; k++)
                {
                    int ind = cloudSortInd[k];

                    if (cloudNeighborPicked[ind] == 0 &&
                        cloudCurvature[ind] < 0.1)
                    {

                        cloudLabel[ind] = -1; 
                        features.surfPointsFlat->push_back(laserCloud->points[ind]);

                        smallestPickedNum++;
                        if (smallestPickedNum >= 7)
                        { 
                            break;
                        }

                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++)
                        { 
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                            {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                            {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++)
                {
                    // all other points???
                    if (cloudLabel[k] <= 0)
                    {
                        surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                    }
                }
            }

            pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
            pcl::VoxelGrid<PointType> downSizeFilter;
            downSizeFilter.setInputCloud(surfPointsLessFlatScan);
            downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
            downSizeFilter.filter(surfPointsLessFlatScanDS);

            *features.surfPointsLessFlat += surfPointsLessFlatScanDS;
        }
        printf("sort q time %f \n", t_q_sort);
        printf("seperate points time %f \n", t_pts.toc());

        std::cout << "surfPointsFlat.size()        :" << features.surfPointsFlat->size() << std::endl;
        std::cout << "surfPointsLessFlat.size()    :" << features.surfPointsLessFlat->size() << std::endl;
        std::cout << "cornerPointsSharp.size()     :" << features.cornerPointsSharp->size() << std::endl;
        std::cout << "cornerPointsLessSharp.size() :" << features.cornerPointsLessSharp->size() << std::endl;

        return features;
    }


    LidarFeatures extractFeatures2(std::vector<pcl::PointCloud<PointType>> &laserCloudScans)
    {

        TicToc t_pts;

        LidarFeatures features;

        for (int idx_s = 0; idx_s < N_SCANS; idx_s++)
        {
            // std::cout << "idx_s: " << idx_s << std::endl;
            // if(idx_s != 20)
            // {
            //     continue;
            // }

            if(laserCloudScans[idx_s].points.size() < 10)
            {
                continue;
            }

            auto & laserCloud = laserCloudScans[idx_s];

            for(size_t i = 5; i < laserCloud.points.size() - 5; ++i)
            {
                Eigen::Vector3f p0{laserCloud.points[i].x, laserCloud.points[i].y, laserCloud.points[i].z};

                // const auto point = laserCloud.points[i];
                // float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
                // std::cout << "angle: " << angle << std::endl;

                Eigen::Vector3f curv_sum {0., 0, 0};
                bool toFar = false;

                Eigen::Vector3f pr_prev{laserCloud.points.at(i).x, laserCloud.points[i].y, laserCloud.points[i].z};
                Eigen::Vector3f pl_prev{laserCloud.points.at(i).x, laserCloud.points[i].y, laserCloud.points[i].z};

                for(size_t j = 1; j <= 5; ++j)
                {
                    Eigen::Vector3f pr{laserCloud.points.at(i + j).x, laserCloud.points[i + j].y, laserCloud.points[i + j].z};
                    Eigen::Vector3f pl{laserCloud.points.at(i - j).x, laserCloud.points[i - j].y, laserCloud.points[i - j].z};

                    if( (pr - pr_prev).norm() > 0.3 || (pl - pl_prev).norm() > 0.3)
                    {
                        toFar = true;
                        break;
                    }

                    pr_prev = pr;
                    pl_prev = pl;

                    Eigen::Vector3f dr = (pr - p0);
                    Eigen::Vector3f dl = (pl - p0);
                    curv_sum += dr + dl; 
                }

                if(toFar)
                {
                    continue;
                }

                float curv = curv_sum.norm(); // / p0.norm();

                if(curv > 0.001)
                {
                    features.cornerPointsSharp->push_back(laserCloud.points[i]);
                    features.cornerPointsSharp->points.back().intensity = curv * 50;
                }
            }
            

        }
        printf("seperate points time %f \n", t_pts.toc());

        std::cout << "surfPointsFlat.size()        :" << features.surfPointsFlat->size() << std::endl;
        std::cout << "surfPointsLessFlat.size()    :" << features.surfPointsLessFlat->size() << std::endl;
        std::cout << "cornerPointsSharp.size()     :" << features.cornerPointsSharp->size() << std::endl;
        std::cout << "cornerPointsLessSharp.size() :" << features.cornerPointsLessSharp->size() << std::endl;

        return features;
    }


    LidarFeatures extractFeaturesSimple(const std::vector<pcl::PointCloud<PointType>> &laserCloudScans)
    {
        std::vector<int> scanStartInd(N_SCANS, 0);
        std::vector<int> scanEndInd(N_SCANS, 0);

        pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());

        assert(laserCloudScans.size() == N_SCANS);
        std::cout << "laserCloudScans.size(): " << laserCloudScans.size() << std::endl;

        LidarFeatures features;
        
        for (int i = 0; i < N_SCANS; i++)
        { 
            for (size_t j = 0; j < laserCloudScans[i].size(); j++)
            {
                const auto &p = laserCloudScans[i].points[j];
                const double dist = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
                const double r = ((double) rand() / (RAND_MAX));

                // std::cout << "dist:" << dist << "  " << r << std::endl;
                constexpr double DIST_MAX = 500;

                // if(dist > 10)
                // {
                //     // features.surfPointsFlat->push_back(p);
                //     all_points.push_back(p);
                // }
                // else 
                if(dist / DIST_MAX > r)
                {
                    features.surfPointsFlat->push_back(p);
                }
                
            }
        }

        if(false)
        {
            pcl::VoxelGrid<PointType> downSizeFilter;
            downSizeFilter.setInputCloud(features.surfPointsFlat);
            downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
            downSizeFilter.filter(*features.surfPointsFlat);
        }


        std::cout << "surfPointsFlat.size()        :" << features.surfPointsFlat->size() << std::endl;
        std::cout << "surfPointsLessFlat.size()    :" << features.surfPointsLessFlat->size() << std::endl;
        std::cout << "cornerPointsSharp.size()     :" << features.cornerPointsSharp->size() << std::endl;
        std::cout << "cornerPointsLessSharp.size() :" << features.cornerPointsLessSharp->size() << std::endl;
        
        return features;
    }
    

};
