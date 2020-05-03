#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <vector>
#include <unordered_set>
#include <pcl/common/common.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>

template <typename PointT>
void ransacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol, pcl::PointIndices &planeInliers, pcl::ModelCoefficients &modelCoeff)
{
    std::cout << "using custom RANSAC implementation" << std::endl;

    std::unordered_set<int> bestInliersResult;
    srand(time(NULL));

    float x1 = 0;
    float y1 = 0;
    float z1 = 0;

    float x2 = 0;
    float y2 = 0;
    float z2 = 0;

    float x3 = 0;
    float y3 = 0;
    float z3 = 0;

    float bestError = 10000;

    std::vector<float> bestModel;

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

        index = rand() % cloud->size();
        pcl::PointXYZ point3 = cloud->at(rand() % cloud->size());
        inliers.insert(index);

        x1 = point1.x;
        y1 = point1.y;
        z1 = point1.z;

        x2 = point2.x;
        y2 = point2.y;
        z2 = point2.z;

        x3 = point3.x;
        y3 = point3.y;
        z3 = point3.z;

        float i = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        float j = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        float k = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);

        float A = i;
        float B = j;
        float C = k;
        float D = -(i * x1 + j * y1 + k * z1);

        // float zError = 0;

        std::vector<float> model{A, B, C, D};

        int inliersThresh = 5;

        for (int index = 0; index < cloud->size(); index++)
        {
            if (inliers.count(index))
                continue;

            pcl::PointXYZ point = cloud->at(index);
            float x = point.x;
            float y = point.y;
            float z = point.z;

            float distance = fabs(A * x + B * y + C * z + D) / sqrt(A * A + B * B + C * C);

            // reference: https://stackoverflow.com/questions/59919008/calculate-root-mean-square-of-3d-deviation-after-surface-fitting-in-python
            // float zModel = -(((x - B) / A) * ((x - B) / A) + ((y - D) / C) * ((y - D) / C)) + 1;
            // zError += zModel - z;

            if (distance < distanceTol)
            {
                inliers.insert(index);
            }
        }
        if (inliers.size() >= bestInliersResult.size())
        {
            // rmse is not used as it spoils RANSAC because of random generator
            // float rmse = sqrt(zError * zError / cloud->size());

            // if (rmse < bestError)
            // {
            bestModel = model;
            // bestError = rmse;
            bestInliersResult = inliers;
            // }
        }
    }
    std::copy(bestInliersResult.begin(), bestInliersResult.end(), std::back_inserter(planeInliers.indices));
    std::copy(bestModel.begin(), bestModel.end(), std::back_inserter(modelCoeff.values));
}
#endif /* UTILS_H */
