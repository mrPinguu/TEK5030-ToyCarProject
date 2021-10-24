#include "Eigen/Eigen"
#include "opencv2/opencv.hpp"
#include "opencv2/core/eigen.hpp"
#include <iostream>
#include "tek5030/realsense_rgbd.h"
#include "segmentation.h"
#include "object_follow.cpp"

using namespace tek5030;
using namespace tek5030::RealSense;


bool ChkCollision(double min_lim, double max_lim, int N_min, cv::Mat depthImg);

int main() {

//    code for generating aruco marker, Need only use this once
//    cv::Mat markerImage;
//    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
//    cv::aruco::drawMarker(dictionary, 23, 200, markerImage, 1);
//    cv::imwrite("marker23.png", markerImage);

//  Create window
    const std::string win_name_camera{"Camera"};
    cv::namedWindow(win_name_camera, cv::WINDOW_NORMAL);

    const std::string win_name_depth{"Depth"};
    cv::namedWindow(win_name_depth, cv::WINDOW_NORMAL);

    const std::string win_name_aruco{"Aruco"};
    cv::namedWindow(win_name_aruco, cv::WINDOW_NORMAL);

    const std::string win_name_control{"Controls"};
    cv::namedWindow(win_name_control, cv::WINDOW_NORMAL);

    const std::string win_name_ground{"Ground plane segmentation"};
    cv::namedWindow(win_name_ground, cv::WINDOW_NORMAL);

    const std::string win_name_4{"Camera, Depth, Color segmentation, Ground and Color segmentation"};
    cv::namedWindow(win_name_4, cv::WINDOW_NORMAL);



// Fetch RGBD camera from Realsesene
    RGBDCamera rgbdcam;
    ArucoController arucoController;
    arucoController.p_gain_ = 1;



    cv::Mat currentSamples;
    rgbdcam.setLaserMode(RGBDCamera::LaserMode::ON);

    bool init = true;
    bool driveForward = false;
    bool turnLeft = false;
    float u = 0.0;
    bool collisionDetected = false;

    while (true) {

        RGBDImage rgbdframe;
        rgbdcam >> rgbdframe;

        cv::Mat depth;
        cv::Mat depthSegmented;
        depth = rgbdframe.depth.clone();
        depthSegmented = rgbdframe.depth.clone();

        cv::Mat rgbImg;
        cv::Mat rgbImgNorm;
        rgbImg = rgbdframe.color.clone();
        cv::normalize(rgbImg, rgbImgNorm);

        cv::Mat visDepth;
        // Find the ground plane mask from the depth image
        cv::Mat GroundPlane = detectGroundPlane(depth);

        //std::cout << depth.at<double>(cv::Point(depth.rows / 2, depth.cols / 2)) << "\n";

        rgbdframe.depth.convertTo(visDepth,CV_8UC1, 255);

        cv::Mat depth_color;
        cv::applyColorMap(visDepth,depth_color, cv::COLORMAP_JET);

        cv::Mat seg = segmentation(rgbImg, currentSamples, init);
        visDepth.setTo(255, seg);
        depthSegmented.setTo(10.0, seg);


        cv::Mat SegmentedGroundPlane;
        GroundPlane.convertTo(GroundPlane,CV_8UC1,255);
        SegmentedGroundPlane = depthSegmented.clone();
        SegmentedGroundPlane.setTo(1, GroundPlane);

        //object detection
        arucoController.Detect(rgbImg); //detect aruco markers
        arucoController.CalcCenter();
        arucoController.DepthControl(0.5,0.7,depth, driveForward);

        collisionDetected = ChkCollision(0.1, 0.4, 2500, depthSegmented);


        if (collisionDetected)
        {
            driveForward= false;
        }

        u = arucoController.HorizControl(rgbImg);
        turnLeft = u>=0;


        cv::Mat visDetection;
        visDetection = rgbImg.clone();
        arucoController.VisDetect(visDetection);

        cv::Mat controlsImg;

        if (collisionDetected){
            controlsImg = cv::imread("../controlsImg/collision.PNG");
        } else if (driveForward) {
            if (turnLeft) {
                controlsImg = cv::imread("../controlsImg/fl.PNG");
            }else{
                controlsImg = cv::imread("../controlsImg/fr.PNG");}
        } else {
            if (turnLeft){
                controlsImg = cv::imread("../controlsImg/l.PNG");
            }else{
                controlsImg = cv::imread("../controlsImg/r.PNG");
            }
        }
        controlsImg.cv::Mat::convertTo(controlsImg,CV_32F, 1.f/255.f);

        // Apply color map to grayscale images for better visualization
        cv::Mat segdepth_color;
        cv::applyColorMap(visDepth,segdepth_color, cv::COLORMAP_JET);

        SegmentedGroundPlane.convertTo(SegmentedGroundPlane,CV_8UC1,255);
        cv::Mat GroundPlane_Color;
        cv::applyColorMap(SegmentedGroundPlane,GroundPlane_Color,cv::COLORMAP_JET);

        // Add sampling frame to image
        drawSamplingRectangle(rgbdframe.color);

        // Visualize.
        cv::Mat vis;
        cv::hconcat(rgbdframe.color, segdepth_color, vis);

        cv::Mat segmentedANDground;
        cv::hconcat( segdepth_color, GroundPlane_Color, segmentedANDground);

        cv::Mat show4;
        std::vector<cv::Mat> Mat4view = {rgbdframe.color,depth_color,segdepth_color,GroundPlane_Color };
        cv::hconcat(Mat4view,show4);

        cv::imshow(win_name_camera, vis);
        cv::imshow(win_name_depth, depth_color);
        cv::imshow(win_name_aruco, visDetection);
        cv::imshow(win_name_ground, segmentedANDground);
        cv::imshow(win_name_control, controlsImg);
        cv::imshow(win_name_4, show4);

        // Keypress kills loop
        char key = static_cast<char>(cv::waitKey(10));
        if (key >= 0) {
            break;
        }
        init = false;

    }
    return EXIT_SUCCESS;
}

bool ChkCollision(double min_lim, double max_lim, int N_min, cv::Mat depthImg){

    int cnt = 0;
    for (int i=0; i < depthImg.rows ; i++){
        for(int j=0; j < depthImg.cols; j++){
            if (depthImg.at<double>(cv::Point(j, i)) < max_lim && depthImg.at<double>(cv::Point(j, i)) > min_lim ){
                cnt += 1;
            }
        }
    }
    return cnt>=N_min;
}
