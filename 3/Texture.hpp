//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto u_img_int = int(u_img);
        auto v_img_int = int(v_img);
        
        auto A = image_data.at<cv::Vec3b>(v_img_int, u_img_int);
        auto B = image_data.at<cv::Vec3b>(v_img_int, u_img_int + 1);
        auto C = image_data.at<cv::Vec3b>(v_img_int + 1, u_img_int);
        auto D = image_data.at<cv::Vec3b>(v_img_int + 1, u_img_int + 1);
        auto x1 = u_img - u_img_int;
        auto x2 = 1 - x1;
        auto y1 = v_img - v_img_int;
        auto y2 = 1 - y1;
        auto color = (x1 * B + x2 * A) * y2 + y1 * (x1 * D + x2 * C);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
