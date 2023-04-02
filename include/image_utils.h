#include <string>
#include<iostream>
#include <algorithm>
#include "opencv2/core/core.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include "plan_utils.h"

#pragma once
#define FMT_HEADER_ONLY
#include "format.h"
namespace std
{
    using fmt::format;
    using fmt::format_error;
    using fmt::formatter;
}

void display_image(cv::Mat img)
{
    cv::imshow("Image", img);
    cv::waitKey(-1);
}

std::string a_library_function();

template <class T>
T add(T a, T b){
    return a + b;
}

cv::Mat load_image(std::string path){
    return cv::imread(path);
}

std::vector<std::vector<float>> img_to_vector(const cv::Mat& img){
    std::vector<std::vector<float>> vecOut;
    vecOut.resize(img.rows, std::vector<float>(img.cols));
    for(int i = 0 ;i < img.rows; ++i){
        for(int j = 0; j < img.cols; ++j){
            vecOut[i][j] = float(img.at<cv::Vec3b>(i, j)[0]);
        }
    }
    return vecOut;
}

cv::Mat vector_to_img(floatGrid vec){
    cv::Mat dst;
    for(int i = 0; i < vec.size(); ++i){
        for(int j = 0; j < vec[0].size(); ++j){
            dst.at<float>(i, j) = vec[i][j];
        }
    }
    return dst;
}



void draw_path_on_img(std::vector<PixelWithCost> path, std::string img_path){
    cv::Mat color_img = cv::imread(img_path, CV_LOAD_IMAGE_COLOR);
    for(auto p : path){
        color_img.at<cv::Vec3b>(p.row, p.col) = cv::Vec3b({255, 0, 0});
    }
    display_image(color_img);
}
