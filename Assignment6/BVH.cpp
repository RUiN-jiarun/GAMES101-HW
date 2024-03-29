#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

// TODO: using SAH acceleration algorithm
BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        // Get the largest bbox
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        // Compute the total area
        float Sn = centroidBounds.SurfaceArea();
        // Set the bucket number B (B < 32)
        int B = 25;
        float minCost = std::numeric_limits<float>::infinity();
        int minCostCoor = 0;
        int minCostIndex = 0;
        
        // Split according to each axis
        for (int i = 0; i < 3; i++)     
        {   
            switch (i)
            {
                case 0:  // sorting x
                    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {return f1->getBounds().Centroid().x < f2->getBounds().Centroid().x;});
                    break;
                case 1:  // sorting y
                    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {return f1->getBounds().Centroid().y < f2->getBounds().Centroid().y;});
                    break;
                case 2:  // sorting z
                    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {return f1->getBounds().Centroid().z < f2->getBounds().Centroid().z;});
                    break;
            }
            for (int j = 1; j < B; j++)
            {
                auto beginning = objects.begin();
                auto middling = objects.begin() + (objects.size() * j / B);
                auto ending = objects.end();
                auto leftshapes = std::vector<Object*>(beginning, middling);
                auto rightshapes = std::vector<Object*>(middling, ending);
                // Computing left bbox and right bbox
                Bounds3 leftBounds, rightBounds;
                for (int k = 0; k < leftshapes.size(); ++k)
                    leftBounds = Union(leftBounds, leftshapes[k]->getBounds().Centroid());
                for (int k = 0; k < rightshapes.size(); ++k)
                    rightBounds = Union(rightBounds, rightshapes[k]->getBounds().Centroid());
                float SA = leftBounds.SurfaceArea(); 
                float SB = rightBounds.SurfaceArea(); 
                float cost = 1 + (leftshapes.size() * SA + rightshapes.size() * SB) / Sn; //计算花费
                // Optimize and get the minCost
                if (cost < minCost) 
                {
                    minCost = cost;
                    minCostIndex = j;
                    minCostCoor = i;
                }
            }
        }


        // int dim = centroidBounds.maxExtent();
        // switch (dim) {
        // case 0:
        //     std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
        //         return f1->getBounds().Centroid().x <
        //                f2->getBounds().Centroid().x;
        //     });
        //     break;
        // case 1:
        //     std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
        //         return f1->getBounds().Centroid().y <
        //                f2->getBounds().Centroid().y;
        //     });
        //     break;
        // case 2:
        //     std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
        //         return f1->getBounds().Centroid().z <
        //                f2->getBounds().Centroid().z;
        //     });
        //     break;
        // }


        switch (minCostCoor)
        {
            case 0:  // sorting x
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {return f1->getBounds().Centroid().x < f2->getBounds().Centroid().x;});
                break;
            case 1:  // sorting y
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {return f1->getBounds().Centroid().y < f2->getBounds().Centroid().y;});
                break;
            case 2:  // sorting z
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {return f1->getBounds().Centroid().z < f2->getBounds().Centroid().z;});
                break;
        }

        auto beginning = objects.begin();
        // auto middling = objects.begin() + (objects.size() / 2);
        auto middling = objects.begin()+ (objects.size() * minCostIndex / B);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Vector3f invDir(1.0/ray.direction.x, 1.0/ray.direction.y, 1.0/ray.direction.z);
    std::array<int, 3> dirisNeg = {ray.direction.x > 0, ray.direction.y > 0, ray.direction.z > 0};
    if (!node->bounds.IntersectP(ray, invDir, dirisNeg))
    {
        // Ray misses bounding box
        return {};
    }
    if (node->left == nullptr && node->right == nullptr)
    {
        // Node is a leaf node
        // Test intersection with all objects and return the closest one
        return node->object->getIntersection(ray);
    }
    // recursive
    Intersection hit1 = BVHAccel::getIntersection(node->left, ray);
    Intersection hit2 = BVHAccel::getIntersection(node->right, ray);
    return hit1.distance > hit2.distance ? hit2 : hit1;

    // return {};
}