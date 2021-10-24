//
// Created by adrianbe on 18.05.2021.
//
// source https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html

#include "opencv2/imgproc.hpp"
#include "opencv2/aruco.hpp"
#include <iostream>

class ArucoController{
private: cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

public:
    float p_gain_;
    std::vector<int> ids_;
    std::vector<std::vector<cv::Point2f>> corners_;
    int xcenter_;
    int ycenter_;

    void Detect(cv:: Mat rgbImage)
    {
        cv::aruco::detectMarkers(rgbImage,dictionary,corners_, ids_);
    }

    void VisDetect(const cv::Mat& rgbImageCopy)
    {
        if(ids_.size()>0) {
            cv::aruco::drawDetectedMarkers(rgbImageCopy, corners_, ids_);
        }
    }

    void CalcCenter()
    {
        cv::Point2f A, B, C, D;
        if(ids_.size()>0) {
            A = corners_.at(0).at(0);
            B = corners_.at(0).at(1);
            C = corners_.at(0).at(2);
            D = corners_.at(0).at(3);
            ycenter_ = (C.y - A.y) / 2 + A.y;
            xcenter_ = (C.x - A.x) / 2 + A.x;
        }
    }


    void DepthControl(float sp_off, float sp_on, cv::Mat depthImg, bool& forward )
    {
        double arucoDepth;

        if(ids_.size()>0) {
            arucoDepth = depthImg.at<double>(cv::Point(xcenter_, ycenter_));
        }

       if(!(ids_.size()>0)) {
           forward = false;
       }

       else if (arucoDepth < sp_off) {
           forward = false;
       }
       else if (!forward && arucoDepth > sp_on){
            forward = true;
        }

    }

    float HorizControl(cv::Mat rgbImg)
    {

        float error = rgbImg.cols/2 - xcenter_;

        return p_gain_*error;
    }


};
