//
// Created by jpetlund on 29.04.2021.
//

#ifndef TEK5030_PROJECT_SEGMENTATION_H
#define TEK5030_PROJECT_SEGMENTATION_H

#endif //TEK5030_PROJECT_SEGMENTATION_H

#include "opencv2/imgproc.hpp"


cv::Mat segmentation(cv::Mat frame, cv::Mat& currentSamples,  bool init);
cv::Mat detectGroundPlane(cv::Mat& depth);
void drawSamplingRectangle(cv::Mat& image);

