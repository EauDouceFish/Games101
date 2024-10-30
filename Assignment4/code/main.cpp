#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 500) 
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
        auto point = std::pow(1-t, 3)*p_0 + 3*t*std::pow(1-t, 2)*p_1 +
                 3*std::pow(t, 2)*(1-t)*p_2 + std::pow(t, 3)*p_3;


        //draw the curve in row major type, and it is BGR in traditional libs.
        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}


std::vector<cv::Point2f> rec(std::vector<cv::Point2f> &rec_vector, int seg_nums, float t)
{
    std::vector<cv::Point2f> result_vec;
    for(int i = 0; i < seg_nums; i++)
    {
        auto newx = t*(rec_vector[i+1].x - rec_vector[i].x) + rec_vector[i].x;
        auto newy = t*(rec_vector[i+1].y - rec_vector[i].y) + rec_vector[i].y;
        cv::Point2f new_ctrl_point(newx, newy);
        result_vec.push_back(new_ctrl_point);
    }
    return result_vec;
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    int point_nums = control_points.size();
    int seg_nums = point_nums-1;
    std::vector<cv::Point2f> rec_vector = control_points;

    while(seg_nums>0)
    {
        rec_vector = rec(rec_vector, seg_nums, t);
        seg_nums--;
    }

    cv::Point2f result_point = rec_vector[0];
    // TODO: Implement de Casteljau's algorithm
    return result_point;
}

//input a vector including a sets of points.
void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{


    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
    }
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
}

int main() 
{
    int n=0;
    std::cin >> n;
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == n) 
        {
            // naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
