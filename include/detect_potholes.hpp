#ifndef DETECT_POTHOLES_HPP
#define DETECT_POTHOLES_HPP

#include <cv_bridge/cv_bridge.h>

std::vector<cv::Point> highlight_potholes(const cv_bridge::CvImagePtr& input);

#endif