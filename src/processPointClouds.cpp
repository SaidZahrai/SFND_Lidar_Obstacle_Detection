// PCL lib Functions for processing point clouds 

// Update and cleaned version for the course project - Dec 30, 2022


#include "processPointClouds.h"
#include "projectImplementations.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

// FilterCloud: a. Voxel filter, b. Corp the cloud and c. Remove the roof points
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Completed during the course according to helps and instructions
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered {new pcl::PointCloud<PointT>};

    // Create and apply the voxel filter

    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion {new pcl::PointCloud<PointT>};

    // Create and apply a filter to 1) remove the points outside the region of interest and 
    // 2) remove the points that are inside a box on the roof of the car. 

    pcl::CropBox<PointT> roi(true);
    roi.setInputCloud(cloudFiltered);
    roi.setMin(minPoint);
    roi.setMax(maxPoint);
    roi.filter(*cloudRegion);

    // Remove roof points
    roi.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roi.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roi.setInputCloud(cloudRegion);
    roi.setNegative(true);
    roi.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}

// SeparateCloud: Separated the point cloud to a plane cloud and the remaining parts 
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // Completed during the course according to helps and instructions
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT> ());

    for (int index: inliers->indices)
    {
        planeCloud->points.push_back(cloud->points[index]);
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

// SegmentPlane: Finds a plane and uses SeparateCloud to present the results as two separate point clouds
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    std::cout<<"Print - Starting RANSAC" << std::endl;
    std::unordered_set<int> planarSegmentIndices = Ransac<PointT>(cloud, maxIterations, distanceThreshold);
    if (planarSegmentIndices.size() == 0)
    {
        std::cout<<"Could not estimate planar interfaces..." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    typename pcl::PointIndices::Ptr inliers {new pcl::PointIndices()};
    for (auto itr = planarSegmentIndices.begin(); itr != planarSegmentIndices.end(); itr++)
        inliers->indices.push_back(*itr);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

// Clustering: Uses KdTree to segment the point cloud and returns the result as a vector of point clouds of the segments
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    EuclideanClustering<PointT> ec(cloud);

    auto cluster_indices = ec.euclideanCluster(clusterTolerance, minSize, maxSize);

    for (auto indices : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_Cluster (new pcl::PointCloud<PointT>);

        for (int i : indices)
        {
            cloud_Cluster->points.push_back(cloud->points[i]);
        }

        cloud_Cluster->width = cloud_Cluster->points.size();
        cloud_Cluster->height = 1;
        cloud_Cluster->is_dense = true;

        clusters.push_back(cloud_Cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

// BoundingBox: Returns the bounding box for a point cloud
template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

// SavePcd: Saves point clouds
template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}

// LoadPcd: Loads point clouds
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}

// streamPcd: Streams point clouds
template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}