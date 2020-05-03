/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Add inliers
    float scatter = 0.6;
    for (int i = -5; i < 5; i++)
    {
        double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        pcl::PointXYZ point;
        point.x = i + scatter * rx;
        point.y = i + scatter * ry;
        point.z = 0;

        cloud->points.push_back(point);
    }
    // Add outliers
    int numOutliers = 10;
    while (numOutliers--)
    {
        double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        pcl::PointXYZ point;
        point.x = 5 * rx;
        point.y = 5 * ry;
        point.z = 0;

        cloud->points.push_back(point);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
    viewer->addCoordinateSystem(1.0);
    return viewer;
}

// reference: https://en.wikipedia.org/wiki/Random_sample_consensus
std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> bestInliersResult;
    srand(time(NULL));

    float x1 = 0;
    float y1 = 0;

    float x2 = 0;
    float y2 = 0;

    float bestError = 10000;

    std::vector<float> bestModel;

    /*
    For variables x and y and coefficients A, B, and C, the general equation of a line is:
        Ax+By+C=0
    Given two points: point1 (x1, y1) and point2 (x2, y2), the line through point1 and point2 has the specific form:

        (y1−y2)x+(x2−x1)y+(x1∗y2−x2∗y1)=0
        
    Line formula Ax + By + C = 0Ax+By+C=0
    Distance d = |Ax+By+C|/sqrt(A^2+B^2)
    */

    // Randomly sample subset and fit line

    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier

    // Return indicies of inliers from fitted line with most inliers

    // For max iterations
    while (maxIterations--)
    {
        std::unordered_set<int> inliers;

        int index = 0;

        index = rand() % cloud->size();
        pcl::PointXYZ point1 = cloud->at(index);
        inliers.insert(index);

        index = rand() % cloud->size();
        pcl::PointXYZ point2 = cloud->at(rand() % cloud->size());
        inliers.insert(index);

        x1 = point1.x;
        y1 = point1.y;

        x2 = point2.x;
        y2 = point2.y;

        float A = y1 - y2;
        float B = x2 - x1;
        float C = x1 * y2 - x2 * y1;

        float yError = 0;

        std::vector<float> model{A, B, C};

        int inliersThresh = 5;

        for (int index = 0; index < cloud->size(); index++)
        {
            if (inliers.count(index))
                continue;

            pcl::PointXYZ point = cloud->at(index);
            float x = point.x;
            float y = point.y;

            float distance = fabs(A * x + B * y + C) / sqrt(A * A + B * B);

            yError += (y2 - y);

            if (distance < distanceTol)
            {
                inliers.insert(index);
            }
        }
        if (inliers.size() >= bestInliersResult.size())
        {
            float rmse = sqrt(yError * yError / cloud->size());

            if (rmse < bestError)
            {
                bestModel = model;
                bestError = rmse;
                bestInliersResult = inliers;
            }
        }
    }

    return bestInliersResult;
}

    // Randomly sample subset and fit line

    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier

    // Return indicies of inliers from fitted line with most inliers

    return inliersResult;
}

int main()
{

    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

    // Create data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();

    // TODO: Change the max iteration and distance tolerance arguments for Ransac function
    std::unordered_set<int> inliers = Ransac(cloud, 0, 0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

    for (int index = 0; index < cloud->points.size(); index++)
    {
        pcl::PointXYZ point = cloud->points[index];
        if (inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    // Render 2D point cloud with inliers and outliers
    if (inliers.size())
    {
        renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
        renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
    }
    else
    {
        renderPointCloud(viewer, cloud, "data");
    }

    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
}
