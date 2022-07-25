// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include "global.hpp"

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

static Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
 
    auto a = rotation_angle / 180 * M_PI;
    model(0, 0) = cosf(a);
    model(0, 1) = -sinf(a);
    model(1, 0) = sinf(a);
    model(1, 1) = cosf(a);
    return model;
}

static float min3(float a, float b, float c) {
    return std::min(std::min(a, b), c);
}

static float max3(float a, float b, float c) {
    return std::max(std::max(a, b), c);
}
static bool pointInside(float x, float y, const Triangle& t);


void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;
    
    for (auto& i : ind)
    {
        auto model = this->model;

        if (i.y() == 1) {
            model = get_model_matrix(angleForTriangle2) * model;
        }
        Eigen::Matrix4f mvp = projection * view * model;
        
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec.x() /= vec.w();
            vec.y() /= vec.w();
            vec.z() /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            // 重新把 Z 映射到 n-f 区间
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.w[i] = v[i].w();
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}
 
//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    auto minX = (int)std::floor(std::min(std::min(v.at(0).x(), v[1].x()), v[2].x()));
    auto maxX = (int)std::ceil(std::max(std::max(v.at(0).x(), v[1].x()), v[2].x()));
    auto minY = (int)std::floor(min3(v[0].y(), v[1].y(), v[2].y()));
    auto maxY = (int)std::ceil(max3(v[0].y(), v[1].y(), v[2].y()));
    
    float targetX;
    float targetY;
    for (int i = minX; i <= maxX; i++) {
        for (int j = minY; j <= maxY; j++) {
            // [i][j]
            targetX = i + 0.5;
            targetY = j + 0.5;
            
            auto isInside = pointInside(targetX, targetY, t);
            if (!isInside) {
                continue;
            }
            auto [alpha, beta, gamma] = computeBarycentric2D(targetX, targetY, t.v);
            // 前面齐次空间变屏幕空间 重心坐标被除以 w，所以现在屏幕空间的重心坐标按新的插值
          
            float z_interpolated = alpha * v[0].z() + beta * v[1].z() + gamma * v[2].z();
         
            if (setDepth(i, j, z_interpolated)) {
                Eigen::Vector3f point(i, j, 1.0f);
                set_pixel(point, t.getColor());
            }
        }
    }
}

static bool pointInside(float x, float y, const Triangle& t) {
    auto& a = t.v[0].topRows(2);
    auto& b = t.v[1].topRows(2);
    auto& c = t.v[2].topRows(2);
    
    Vector2f p = {x, y};
    
    auto ac = (c - a);
    auto cb = (b - c);
    auto ba = (a - b);
    
    auto ap = (p - a);
    auto bp = (p - b);
    auto cp = (p - c);
    
    auto product1 = ap.x() * ac.y() - ac.x() * ap.y();
    auto product2 = cp.x() * cb.y() - cb.x() * cp.y();
    auto product3 = bp.x() * ba.y() - ba.x() * bp.y();
    assert(product1 != 0 && product2 != 0 && product3 != 0);
    
    if (product1 > 0) {
        return !(product2 < 0 || product3 < 0);
    } else {
        return !(product2 > 0 || product3 > 0);
    }
}


bool rst::rasterizer::setDepth(int x, int y, float z) {
    auto p = width * y + x;
    if (depth_buf[p] >= z) {
        depth_buf[p] = z;
        return true;
    }
    return false;
}
void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on
