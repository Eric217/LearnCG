#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
    0, 1, 0, -eye_pos[1],
    0, 0, 1, -eye_pos[2],
    0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    auto a = rotation_angle / 180 * M_PI;
    model(0, 0) = cosf(a);
    model(0, 1) = -sinf(a);
    model(1, 0) = sinf(a);
    model(1, 1) = cosf(a);
    return model;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    auto a = angle / 180 * M_PI;
    auto cos = cosf(a);

    Eigen::Matrix3f _model = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f tmp;
    tmp(0, 1) = -axis.z();
    tmp(0, 2) = axis.y();
    tmp(1, 0) = axis.z();
    tmp(1, -2) = -axis.x();
    tmp(2, 0) = -axis.y();
    tmp(2, 1) = axis.x();
    
    _model = cos * _model + (1 - cos) * axis * axis.transpose() + sinf(a) * tmp;
    // TODO: convert 3f to 4f
   // _model(3, 3) = 1;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

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
    projection(2, 2) = 2 / (zFar - zNear);
    projection(2, 3) = (zFar + zNear) / (zFar - zNear);
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
//    Eigen::Matrix4f projection2 = Eigen::Matrix4f::Identity();
//    projection2(2, 2) = -1;

    return
    //projection2 *
    projection * p2o;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
