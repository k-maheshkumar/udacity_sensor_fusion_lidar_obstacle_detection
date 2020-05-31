/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node
{
    std::vector<float> point;
    int id;
    Node *left;
    Node *right;

    Node(std::vector<float> arr, int setId)
        : point(arr), id(setId), left(NULL), right(NULL)
    {
    }
};

struct KdTree
{
    Node *root;

    KdTree()
        : root(NULL)
    {
    }

    void insertRecursively(Node **node, const std::vector<float> &point, const int &id, const int &depth)
    {

        if (*node == NULL)
        {
            *node = new Node(point, id);
            return;
        }

        else
        {
            int index = depth % 3;

            if ((*node)->point[index] < point[index])
                insertRecursively(&(*node)->right, point, id, depth + 1);
            else
                insertRecursively(&(*node)->left, point, id, depth + 1);
        }
    }

    void insert(std::vector<float> point, int id)
    {
        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root

        insertRecursively(&root, point, id, 0);
    }

    void searchRecursively(Node *node, std::vector<float> target, int depth, float distanceTol, std::vector<int> &ids)
    {
        if (node == NULL)
            return;

        float x1 = node->point[0];
        float y1 = node->point[1];
        float z1 = node->point[2];

        float x2 = target[0];
        float y2 = target[1];
        float z2 = target[2];

        // bool inRange = (fabs(x1 - x2) <= distanceTol) && (fabs(y1 - y2) <= distanceTol);
        bool inRange = x1 > x2 - distanceTol && x1 < x2 + distanceTol && y1 > y2 - distanceTol && y1 < y2 + distanceTol && z1 > z2 - distanceTol && z1 < z2 + distanceTol;

        if (inRange)
        {
            float distance = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1));
            if (distance <= distanceTol)
                ids.push_back(node->id);
        }

        int index = depth % 3;

        if (target[index] - distanceTol < node->point[index])
            searchRecursively(node->left, target, depth + 1, distanceTol, ids);
        if (target[index] + distanceTol > node->point[index])
            searchRecursively(node->right, target, depth + 1, distanceTol, ids);
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol)
    {
        std::vector<int> ids;
        searchRecursively(root, target, 0, distanceTol, ids);
        return ids;
    }
};
