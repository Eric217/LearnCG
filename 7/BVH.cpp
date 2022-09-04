#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    auto start = std::chrono::system_clock::now();

    if (primitives.empty())
        return;
    switch (splitMethod) {
        case SplitMethod::NAIVE:
            root = recursiveBuild(primitives);
            break;
        case SplitMethod::SAH:
            root = recursiveBuildWithSAH(primitives);
            break;
    }
    
    auto stop = std::chrono::system_clock::now();
    
    const char *methodName = splitMethod == SplitMethod::NAIVE ? "BVH-NoSAH" : "BVH-SAH";
    std::cout << methodName << " constructed: \n";
     
    auto s = std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() % 60;
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() % 1000;

    std::cout << "            " << s << " seconds\n";
    std::cout << "            " << ms << " millseconds\n";
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        node->area = objects[0]->getArea();
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
    }

    return node;
}

struct SAHBucket {
    Bounds3 box;
    int N = 0;
    std::vector<Object *> objects;
    float min = 0;
    float max = 0;
};

BVHBuildNode* BVHAccel::recursiveBuildWithSAH(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        node->area = objects[0]->getArea();
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuildWithSAH(std::vector{objects[0]});
        node->right = recursiveBuildWithSAH(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
        return node;
    }
    Bounds3 *cacheBounds = (Bounds3 *)calloc(objects.size(), sizeof(Bounds3));
    Vector3f *cacheCenter = new Vector3f[objects.size()];
     
    for (int i = 0; i < objects.size(); ++i) {
        cacheBounds[i] = std::move(objects[i]->getBounds());
        cacheCenter[i] = std::move(cacheBounds[i].Centroid());
    }
     
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds.unionBounds(cacheBounds[i]);
    
    const auto BucketCount = 18;
    SAHBucket *buckets = new SAHBucket[BucketCount];
    auto cost = kInfinity;
    std::vector<Object *> v1, v2;
    
    for (int axis = 0; axis < 3; axis++) {
        // 用任一个轴初始化桶空间
        const auto min = bounds.pMin[axis];
        const auto max = bounds.pMax[axis];
        
        auto share = (max - min) / BucketCount;
        auto base = min;
        for (int i = 0; i < BucketCount; i++) {
            buckets[i].min = base;
            buckets[i].max = (base += share);
            buckets[i].box = Bounds3();
            buckets[i].N = 0;
            buckets[i].objects.clear();
        }
        // 把 obj 放到相应的桶里
        for (int i = 0; i < objects.size(); ++i) {
            auto center = cacheCenter[i][axis];

            for (int b = 0; b < BucketCount; b++) {
                if (center >= buckets[b].min && center <= buckets[b].max) {
                    // found bucket for obj
                    buckets[b].N ++;
                    buckets[b].box.unionBounds(cacheBounds[i]);
                    buckets[b].objects.push_back(objects[i]);
                    break;
                }
            }
        }
        // 划分 b - 1 次得出最优
        for (int i = 0; i < BucketCount - 1; i++) {
            // 下标 <= i 的桶是一组，剩下的在另一组
            Bounds3 left, right;
            int Nl = 0, Nr = 0;
            for (int b = 0; b <= i; b++) {
                left.unionBounds(buckets[b].box);
                Nl += buckets[b].N;
            }
            for (int b = i + 1; b < BucketCount; b++) {
                right.unionBounds(buckets[b].box);
                Nr += buckets[b].N;
            }
            auto costL = left.SurfaceArea() * Nl;
            auto costR = right.SurfaceArea() * Nr;
            float _cost = costL + costR; // real is: kCost + Ct
            if (_cost < cost) {
                cost = _cost;
                v1.clear();
                v2.clear();
                for (int b = 0; b <= i; b++) {
                    v1.insert(v1.end(), buckets[b].objects.begin(), buckets[b].objects.end());
                }
                for (int b = i + 1; b < BucketCount; b++) {
                    v2.insert(v2.end(), buckets[b].objects.begin(), buckets[b].objects.end());
                }
            }
        }
    }
    assert(v1.size() && v2.size() &&
           objects.size() == (v1.size() + v2.size()));

    node->left = recursiveBuild(v1);
    node->right = recursiveBuild(v2);

    node->bounds = Union(node->left->bounds, node->right->bounds);
    node->area = node->left->area + node->right->area;

    delete [] buckets;
    free(cacheBounds);
    delete [] cacheCenter;
    
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
    std::array<int, 3> dirDir = {
        ray.direction.x > 0, ray.direction.y > 0, ray.direction.z > 0
    };
    if (!node->bounds.IntersectP(ray, ray.direction_inv, dirDir)) {
        return Intersection();
    }
    // case leaf
    if (node->object) {
        return node->object->getIntersection(ray);
    } else {

    }
    auto result1 = getIntersection(node->left, ray);
    auto result2 = getIntersection(node->right, ray);
    if (!result1.happened) {
        return result2;
    }
    if (!result2.happened) {
        return result1;
    }
    return result1.distance < result2.distance ? result1 : result2;
}


void BVHAccel::getSample(BVHBuildNode* node, float p, Intersection &pos, float &pdf){
    if(node->left == nullptr || node->right == nullptr){
        node->object->Sample(pos, pdf);
        pdf *= node->area;
        return;
    }
    if(p < node->left->area) getSample(node->left, p, pos, pdf);
    else getSample(node->right, p - node->left->area, pos, pdf);
}

void BVHAccel::Sample(Intersection &pos, float &pdf){
    float p = std::sqrt(get_random_float()) * root->area;
    getSample(root, p, pos, pdf);
    pdf /= root->area;
}
