#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

static bool usingBunny = false;
static bool usingCrate = false;

Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
                0, 1, 0, 0,
                -sin(angle), 0, cos(angle), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
              0, 2.5, 0, 0,
              0, 0, 2.5, 0,
              0, 0, 0, 1;
    if (usingBunny) {
        scale.topRows(3) *= 10;
    } else if (usingCrate) {
        scale.topRows(3) *= 0.4;
    }

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    translate(1, 3) = usingBunny ? -2 : 0;
    return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    
    Eigen::Matrix4f p2o = projection;
    auto fov = eye_fov / 180 * M_PI;
    auto top = tanf(fov / 2) * zNear;
    p2o(0, 0) = zNear;
    p2o(1, 1) = zNear;
    p2o(2, 2) = zNear + zFar;
    p2o(2, 3) = zNear * zFar;
    p2o(3, 2) = -1;
    p2o(3, 3) = 0;
    
    auto h = top * 2;
    auto w = aspect_ratio * h ;
    projection(0, 0) = 2 / w;
    projection(1, 1) = 2 / h;
    projection(2, 2) = -2 / (zFar - zNear);
    projection(2, 3) = -(zFar + zNear) / (zFar - zNear);
    
    return projection * p2o;
}

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        return_color = payload.texture->getColor(payload.tex_coords.x(),
                                                 payload.tex_coords.y());
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    auto v = eye_pos - point;

    for (auto& light : lights)
    {
        result_color += ka.cwiseProduct(amb_light_intensity);
        
        Vector3f D(0,0,0), S(0,0,0);
        auto l = light.position - point;
        auto r2 = l.squaredNorm();
        auto ln = l.normalized();
        auto nn = normal.normalized();
        float costheta = ln.dot(nn);
        if (costheta > 0) {
            D = light.intensity.cwiseProduct(kd) / r2 * costheta;
            float h = (ln + v.normalized()).normalized().dot(nn);
            S = light.intensity.cwiseProduct(ks) / r2 * std::pow(h > 0 ? h : 0, p);
            result_color += D + S;
        }
    }

    return result_color * 255.f;
}
 
Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    ka *= usingBunny ? 3.5 : 1;
    Eigen::Vector3f kd = usingBunny ? Vector3f{1, 1, 1} : payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;
    
    auto v = eye_pos - point;
    
    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto& light : lights)
    {
        Vector3f A = ka.cwiseProduct(amb_light_intensity);
        
        // ambient 是每个光线加一次，吧
        result_color += A;
        
        Vector3f D(0,0,0), S(0,0,0);
        auto l = light.position - point;
        auto r2 = l.squaredNorm();
        auto ln = l.normalized();
        auto nn = normal.normalized();
        float costheta = ln.dot(nn);
        if (costheta > 0) {
            D = light.intensity.cwiseProduct(kd) / r2 * costheta;
            float h = (ln + v.normalized()).normalized().dot(nn);
            S = light.intensity.cwiseProduct(ks) / r2 * std::pow(h > 0 ? h : 0, p);
        }
        result_color += D + S;
    }

    return result_color * 255.f;
}


Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f normal = payload.normal;
    Eigen::Vector2f coor = payload.tex_coords;

    float kh = 0.2, kn = 0.1;

    const auto& n = normal;
    
    auto _d = std::sqrt(n.x() * n.x() + n.z() * n.z());
    Eigen::Vector3f t = {
        n.x() * n.y() / _d,
        _d,
        n.z() * n.y() / _d
    };
    auto b = n.cross(t);
    Matrix3f TBN;
    TBN << t.x(), b.x(), normal.x(),
        t.y(), b.y(), normal.y(),
        t.z(), b.z(), normal.z();
    
    auto& tex = payload.texture;
    auto w_i = 1.f / tex->width;
    auto h_i = 1.f / tex->height;
    
    auto huv = tex->getColor(coor.x(), coor.y());
    auto huvn = huv.norm();
    
    float dU = kh * kn * (tex->getColor(coor.x() + w_i, coor.y()).norm() - huvn);
    float dV = kh * kn * (tex->getColor(coor.x(), coor.y() + h_i).norm() - huvn);
    Vector3f ln { -dU, -dV, 1 };
    
    
    
    Eigen::Vector3f point = payload.view_pos;


    point += kn * huvn * n;
    auto v = eye_pos - point;


    auto newN = (TBN * ln).normalized();
    
    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        Vector3f A = ka.cwiseProduct(amb_light_intensity);
        
        // ambient 是每个光线加一次，吧?
        result_color += A;
        
        Vector3f D(0,0,0), S(0,0,0);
        auto l = light.position - point;
        auto r2 = l.squaredNorm();
        auto ln = l.normalized();
        auto& nn = newN;
        float costheta = ln.dot(nn);
        if (costheta > 0) {
            D = light.intensity.cwiseProduct(kd) / r2 * costheta;
            float h = (ln + v.normalized()).normalized().dot(nn);
            S = light.intensity.cwiseProduct(ks) / r2 * std::pow(h > 0 ? h : 0, p);
        }
        result_color += D + S;

    }

    return result_color * 255.f;
}


Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    if (!payload.texture) {
        return Eigen::Vector3f {0};
    }
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;
    Eigen::Vector2f coor = payload.tex_coords;

    float kh = 0.2, kn = 0.1;

    const auto& n = normal;
    
    auto _d = std::sqrt(n.x() * n.x() + n.z() * n.z());
    Eigen::Vector3f t = {
        n.x() * n.y() / _d,
        _d,
        n.z() * n.y() / _d
    };
    auto b = n.cross(t);
    Matrix3f TBN;
    TBN << t.x(), b.x(), normal.x(),
        t.y(), b.y(), normal.y(),
        t.z(), b.z(), normal.z();
    
    auto& tex = payload.texture;
    auto w_i = 1.f / tex->width;
    auto h_i = 1.f / tex->height;
    
    auto huv = tex->getColor(coor.x(), coor.y()).norm();
    
    float dU = kh * kn * (tex->getColor(coor.x() + w_i, coor.y()).norm() - huv);
    float dV = kh * kn * (tex->getColor(coor.x(), coor.y() + h_i).norm() - huv);
    Vector3f ln { -dU, -dV, 1 };
    Eigen::Vector3f result_color = (TBN * ln).normalized();
     
    return result_color * 255.f;
}

int main(int argc, const char** argv)
{
    int frameC = 0;
    std::vector<Triangle*> TriangleList;
    std::string modelDir = MODEL_DIR;
    float angle = 140.0;
    bool command_line = false;

    std::string filename = "output.png";
    objl::Loader Loader;
     std::string obj_path = modelDir + "/spot/";
//    std::string obj_path = modelDir + "/Crate/";
//    std::string obj_path = modelDir + "/bunny/";
//    std::string obj_path = modelDir + "/cube/";
//    std::string obj_path = modelDir + "/rock/";
    
    std::string obj_name = "spot_triangulated_good.obj";
//    std::string obj_name = "Crate1.obj";
//    std::string obj_name = "bunny.obj";
//    std::string obj_name = "cube.obj";
//    std::string obj_name = "rock.obj";
    // Load .obj File
    bool loadout = Loader.LoadFile(obj_path + obj_name);
    usingBunny = obj_name == "bunny.obj";
    usingCrate = obj_name == "Crate1.obj" || obj_name == "cube.obj" || obj_name == "rock.obj";
    
    const char* texture_path = 0;
    if (obj_name == "rock.obj") {
        texture_path = "rock.png";
    } else if (obj_name == "spot_triangulated_good.obj") {
        texture_path = "spot_texture.png";
    }
    
    for(auto mesh:Loader.LoadedMeshes)
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            for(int j=0;j<3;j++)
            {
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }

    rst::rasterizer r(700, 700);

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;

    if (argc >= 2)
    {
        command_line = argc == 4;
        filename = obj_name + "_" + std::string(argv[1]);
        if (argc >= 3) {
            filename = obj_name + "_" + std::string(argv[2]) + "_" + std::string(argv[1]);
        }
        
        if (argc >= 3 && std::string(argv[2]) == "texture")
        {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            if (texture_path) {
                r.set_texture(Texture(obj_path + texture_path));
            }
        }
        else if (argc >= 3 && std::string(argv[2]) == "normal")
        {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        }
        else if (argc >= 3 && std::string(argv[2]) == "phong")
        {
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        }
        else if (argc >= 3 && std::string(argv[2]) == "bump")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
            auto texture_path = "hmap.jpg";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (argc >= 3 && std::string(argv[2]) == "displacement")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = displacement_fragment_shader;
            auto texture_path = "hmap.jpg";
            r.set_texture(Texture(obj_path + texture_path));
        }
    }

    Eigen::Vector3f eye_pos = {0,0,10};

    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    int key = -1;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(1)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        printf("frame count %d\n", ++frameC);
        
        while(key == -1) {
            key = cv::waitKey(10000);
        }
        if (key == 'a' )
        {
            angle -= 20;
        }
        else if (key == 'd')
        {
            angle += 20;
        } else if (key == 27) {
            return 0;
        }
        key = -1;
    }
    return 0;
}
