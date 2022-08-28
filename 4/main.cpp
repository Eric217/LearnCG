#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
    }
}

static cv::Point2f pointOf(const cv::Point2f& p0, const cv::Point2f& p1, float t) {
    return (1 - t) * p0 + t * p1;
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // de Casteljau's algorithm
    if (control_points.size() < 2) {
        assert(false);
        return {0};
    }
    if (control_points.size() == 2) {
        return pointOf(control_points[0], control_points[1], t);
    }
    std::vector<cv::Point2f> result(control_points.size() - 1);
    for (auto i = 0; i < result.size(); i++) {
        result[i] = (pointOf(control_points[i], control_points[i+1], t));
    }
    return recursive_bezier(result, t);
}

constexpr static float maxWorkingDistance = 1;

static float transformToColorCoeff(float distance) {
    return 1 - std::pow(distance, 1.5);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    if (control_points.size() < 2) {
        assert(false);
        return;
    }
    
    for (double t = 0.0; t <= 1.0; t += 0.001) {
        const auto& p = recursive_bezier(control_points, t);
        auto x0 = int(p.x - 1);
        auto x1 = std::ceil(p.x + 1);
        auto y0 = int(p.y - 1);
        auto y1 = std::ceil(p.y + 1);
        for (auto i = x0; i <= x1; i++) {
            if (i < 0 || i >= window.cols) {
                continue;
            }
            for (auto j = y0; j <= y1; j++) {
                if (j < 0 || j >= window.rows) {
                    continue;
                }
                
                float x = 0.5 + i;
                float y = 0.5 + j;
                
                x = (p.x - x);
                y = p.y - y;
               
                auto distance = pow(x * x + y * y, 0.5);
                if (distance >= maxWorkingDistance) {
                    continue;
                }
                auto color = transformToColorCoeff(distance) * 255;
                auto& pixel = window.at<cv::Vec3b>(0.5 + j, 0.5 + i)[1];
                if (pixel < color) {
                    pixel = color;
                }
            }
        }
        //window.at<cv::Vec3b>(p.y, p.x)[1] = 255;
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);
    
    cv::Point2f p1(220, 8);
    cv::Point2f p2(117, 167);
    cv::Point2f p3(284, 144);
    cv::Point2f p4(330, 109);
    
    control_points = {p1, p2, p3, p4};
    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            naive_bezier(control_points, window);
           // bezier(control_points, window);
            
            cv::imshow("Bezier Curve", window);
            char arr[100] = {0};
            sprintf(arr, "bezier_curve_%zd.png", std::time(0));
            cv::imwrite(arr, window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
