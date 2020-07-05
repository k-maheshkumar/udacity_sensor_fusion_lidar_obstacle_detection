// PCL lib Functions for processing point clouds

#include "processPointClouds.h"

//constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    typename pcl::PointCloud<PointT>::Ptr filteredCloud{new pcl::PointCloud<PointT>()};
    
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    // Create the filtering object
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*filteredCloud);

    pcl::CropBox<PointT> cropBox;

    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);
    cropBox.setInputCloud(filteredCloud);
    cropBox.filter(*filteredCloud);

    pcl::CropBox<PointT> egoVehicleRoof(true);

    egoVehicleRoof.setMin(Eigen::Vector4f(-1.5, -1.7, -1.0, 1.0));
    egoVehicleRoof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1.0));
    egoVehicleRoof.setInputCloud(filteredCloud);

    std::vector<int> inlierIndices;
    egoVehicleRoof.filter(inlierIndices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};

    for (int index : inlierIndices)
    {
        inliers->indices.push_back(index);
    }

    pcl::ExtractIndices<PointT> extractedIndices;

    extractedIndices.setInputCloud(filteredCloud);
    extractedIndices.setIndices(inliers);
    extractedIndices.setNegative(true);
    extractedIndices.filter(*filteredCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filteredCloud;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstaclesCloud(new pcl::PointCloud<PointT>());

    for (int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstaclesCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstaclesCloud, planeCloud);

    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // pcl::SACSegmentation<PointT> seg;
    // seg.setOptimizeCoefficients(true);
    // seg.setModelType(pcl::SACMODEL_PLANE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setMaxIterations(maxIterations);
    // seg.setDistanceThreshold(distanceThreshold);
    // seg.setInputCloud(cloud);
    // seg.segment(*inliers, *coefficients);

    ransacPlane<PointT>(cloud, maxIterations, distanceThreshold, *inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        std::cout << "Couldn't estimate a planar model for the given dataset" << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

template <typename PointT>
void ProcessPointClouds<PointT>::getClusterInProximity(const std::vector<std::vector<float>> &points, KdTree *tree, std::vector<bool> &processed_cluster, int index, float distanceTol, std::vector<int> &cluster)
{

    processed_cluster[index] = true;

    cluster.push_back(index);

    std::vector<int> nearby = tree->search(points[index], distanceTol);

    for (int idx : nearby)
    {
        if (!processed_cluster[idx])
            getClusterInProximity(points, tree, processed_cluster, idx, distanceTol, cluster);
    }
}

template <typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol)
{
    std::cout << "using custom euclideanCluster implementation" << std::endl;

    std::vector<std::vector<int>> clusters;

    std::vector<bool> processed_cluster(points.size(), false);

    for (int index = 0; index < points.size(); index++)
    {
        if (!processed_cluster[index])
        {
            std::vector<int> cluster;

            getClusterInProximity(points, tree, processed_cluster, index, distanceTol, cluster);

            clusters.push_back(cluster);
        }
    }

    return clusters;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    // // Creating the KdTree object for the search method of the extraction
    // typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    // tree->setInputCloud(cloud);

    // std::vector<pcl::PointIndices> clusterIndices;
    // pcl::EuclideanClusterExtraction<PointT> ec;
    // ec.setClusterTolerance(clusterTolerance); // 2cm
    // ec.setMinClusterSize(minSize);
    // ec.setMaxClusterSize(maxSize);
    // ec.setSearchMethod(tree);
    // ec.setInputCloud(cloud);
    // ec.extract(clusterIndices);

    // for (pcl::PointIndices pointIndices : clusterIndices)
    // {
    //     typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);

    //     for (int index : pointIndices.indices)
    //         cloudCluster->points.push_back(cloud->points[index]);

    //     cloudCluster->width = cloudCluster->points.size();
    //     cloudCluster->height = 1;
    //     cloudCluster->is_dense = true;

    //     clusters.push_back(cloudCluster);
    // }

    KdTree *tree{new KdTree()};

    std::vector<std::vector<float>> points;

    for (int index = 0; index < cloud->size(); index++)
    {
        const std::vector<float> point{cloud->points[index].x, cloud->points[index].y, cloud->points[index].z};
        tree->insert(point, index);

        points.push_back(point);
    }

    std::vector<std::vector<int>> clustersIndices = euclideanCluster(points, tree, clusterTolerance);

    for (auto indices : clustersIndices)
    {

        if (indices.size() < minSize || indices.size() > maxSize)
        {
            continue;
        }

        typename pcl::PointCloud<PointT>::Ptr cluster{new pcl::PointCloud<PointT>};

        for (auto index : indices)
        {
            cluster->points.push_back(cloud->points[index]);
        }

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template <typename PointT>
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

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}
