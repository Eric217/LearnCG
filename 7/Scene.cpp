//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include <math.h>

void Scene::buildBVH() {
    printf(" - Scene ");
    this->bvh = new BVHAccel(objects);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                // 让 mesh/球 去 sample
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

/// 物体往 outRay 方向反射的光
static Vector3f shade(const Scene& scene, const Intersection& intersection, const Ray& outRay) {
    Vector3f total, fr;
    const Vector3f& hitPoint = intersection.coords;
    const Vector3f& N = intersection.normal;
    
    // 1 从光源采样得到的光
    float pdf = 0, cos;
    Intersection pos; // 用来取 coor，norm，emi
    scene.sampleLight(pos, pdf);
    auto wi = pos.coords - hitPoint;
    auto wi_n = wi.normalized();
    
    auto r2 = dotProduct(wi, wi);
    auto hitPointDelta = ((dotProduct(wi_n, N) < 0) ? -1 : 1) * N * EPSILON;
    Vector3f modifiedOrigin = hitPoint + hitPointDelta;
    Ray customRay(modifiedOrigin, wi_n);
    Intersection pTest = scene.bvh->Intersect(customRay);
    
    // 如果 modifiedOrigin 仍打到自己，应该让 ray 再沿着 N 偏移一部分重新打，不过影响较小可以忽略
    while (pTest.obj == intersection.obj) {
        modifiedOrigin += hitPointDelta * 5;
        customRay = Ray(modifiedOrigin, wi_n);
        pTest = scene.bvh->Intersect(customRay);
    }
    
    // 光源中间有物体，或者 pTest.happened 为 false 也不加光照(这一项影响较小可忽略）
    if (!pTest.happened || (pTest.happened && pTest.distance < std::sqrt(r2) && pTest.obj != pos.obj)) {

    } else {
        cos = std::max(0.f, dotProduct(N, wi_n));
        float cos1 = std::max(0.f, dotProduct(pos.normal, -wi_n));
        auto _v = cos * cos1;
        if (_v > 0 && pdf > 0) {
//            TODO: 应用 texture
//            intersection.tcoords
            fr = intersection.m->eval(outRay.direction, wi_n, N);
            total = _v * pos.emit * fr / (r2 * pdf);
        }
    }
    
    // 2 从物体反射的光
    if (get_random_float() > scene.RussianRoulette) {
        return total;
    }
    wi = intersection.m->sample(outRay.direction, N).normalized();
    cos = std::max(0.f, dotProduct(wi, N));

    if (cos > 0) {
        modifiedOrigin = (dotProduct(wi, N) < 0)
            ? hitPoint - N * EPSILON : hitPoint + N * EPSILON;
        customRay = Ray(modifiedOrigin, wi);
        pTest = scene.bvh->Intersect(customRay);
        if (pTest.happened && !pTest.m->hasEmission()) {
            assert(pTest.obj != intersection.obj);
            fr = intersection.m->eval(outRay.direction, wi, N);
            pdf = intersection.m->pdf(outRay.direction, wi, N);
            
            auto _v = shade(scene, pTest, customRay);
            total += cos * fr * _v / (pdf * scene.RussianRoulette);
        }
    }
    return total;
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const {
    auto inte = bvh->Intersect(ray);
    if (!inte.happened)
        return Vector3f();
    if (inte.obj->hasEmit()) {
        return Vector3f(1, 1, 1);
    }
    return shade(*this, inte, ray);
}

