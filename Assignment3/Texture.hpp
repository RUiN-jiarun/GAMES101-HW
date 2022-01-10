//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
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
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;

        float u_min = floor(u_img);
        float u_max = MIN(width, ceil(u_img));
        float v_min = floor(v_img);
        float v_max = MIN(height, ceil(v_img));

        // T3------O2-----T4 
        // |       |      |
        // |       |      |
        // |       |      |
        // T-------O------T'
        // |       |      |
        // |       |      |
        // T1------O1-----T2

        auto T1 = image_data.at<cv::Vec3b>(v_max, u_min);
        auto T2 = image_data.at<cv::Vec3b>(v_max, u_max);
        auto T3 = image_data.at<cv::Vec3b>(v_min, u_min);
        auto T4 = image_data.at<cv::Vec3b>(v_min, u_max);

        float alpha = (u_img - u_min) / (u_max - u_min);
        auto O1 = (1 - alpha) * T1 + alpha * T2;
        auto O2 = (1 - alpha) * T3 + alpha * T4;

        float beta = (v_max - v_img) / (v_max - v_min);
        auto O = (1 - beta) * O1 + beta * O2;

        return Eigen::Vector3f(O[0], O[1], O[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
