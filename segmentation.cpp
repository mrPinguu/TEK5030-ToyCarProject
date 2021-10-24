//
// Created by adrianbe on 29.04.2021.
//
#include "segmentation.h"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include "Eigen/Eigen"

//Class for creating a multivariate model of the ground based on colour
class Model {
public:
    cv::Mat mean_;
    cv::Mat covar_;
    cv::Mat invcov_;

    void TrainModel(const cv::Mat& samples)
    {
        cv::calcCovarMatrix(samples,covar_,mean_, cv::COVAR_NORMAL | cv::COVAR_COLS);
        cv::invert(covar_, invcov_, cv::DECOMP_SVD);

    }
    cv::Mat GetMahalanobisImg(const cv::Mat& image)
    {
        constexpr double distToUint8Scale = 1000.0;

        cv::Mat mahalanobisImg(image.size(), CV_8UC1);

        using Pixel = cv::Vec<uint8_t,3>;
        image.forEach<Pixel>(
                [&](const Pixel& pixel, const int* pos)
                {
                    const cv::Vec3d double_pixel(pixel(0), pixel(1), pixel(2));
                    const double mahalanobis = cv::Mahalanobis(double_pixel,mean_,invcov_);
                    mahalanobisImg.at<uint8_t>(pos[0], pos[1]) = static_cast<uint8_t>(distToUint8Scale * mahalanobis);
                }
        );


        return mahalanobisImg;
    }

};

//----Decleration of functions

/// \brief Replace a ratio of old_samples with new_samples
/// \param[in] old_samples Samples used in the current model.
/// \param[in] new_samples New samples.
/// \param[in] update_ratio The ratio of samples to replace on average.
void updateSamples(cv::Mat& old, const cv::Mat& newSamples, float updateRatio);

cv::Rect samplingRectangle(const cv::Size& imgSize);

cv::Mat getTrainingSamples(const cv::Mat& srcImg, const cv::Rect& samplRec);

void drawSamplingRectangle(cv::Mat& image, const cv::Rect& sampling_rectangle);




cv::Mat segmentation(cv::Mat frame, cv::Mat& currentSamples,  bool init)
{

    cv::Rect samplRec = samplingRectangle(frame.size());
    Model model;
    cv::Mat segmentedImage;

    int threshVal = 30;

    //looped part start
    //update samples
    cv::Mat featureImage = frame.clone();
    cv::Mat newSamples = getTrainingSamples(featureImage, samplRec);
    if(init)
    {
       currentSamples = newSamples;
    }
    else
    {
        updateSamples(currentSamples,newSamples,0.4);
    }

    model.TrainModel(currentSamples);


    cv::Mat mahalonobisImg = model.GetMahalanobisImg(featureImage);

    //use otsu
    threshVal = static_cast<int>(cv::threshold(mahalonobisImg,segmentedImage,threshVal,255,cv::THRESH_BINARY_INV | cv::THRESH_OTSU));

    cv::morphologyEx(segmentedImage, segmentedImage, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, {5,5}));
    cv::morphologyEx(segmentedImage, segmentedImage, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, {5,5}));


    //Set segmented image to red
    featureImage.setTo(cv::Scalar{255,0,0}, segmentedImage);

//    drawSamplingRectangle(segmentedImage, samplRec);

    return segmentedImage;
}

 void updateSamples(cv::Mat& old, const cv::Mat& newSamples, float updateRatio)
{
    //draw random numbers
    cv::Mat randNum = cv::Mat::zeros(1, newSamples.cols, CV_32FC1);
    cv::randu(randNum,0.,1);

    //update samples
    for (int i = 0; i<randNum.cols; i++)
    {
        if(randNum.at<float>(0,i) < updateRatio)
        {
            newSamples.col(i).copyTo(old.col(i));
        }
    }
}

cv::Rect samplingRectangle(const cv::Size& imgSize)
{
    int cx = imgSize.width/2;
    int cy = 5*imgSize.height/6;
    int w =800;
    int h = 200;
    int tlx = cx - w/2;
    int tly = cy - h/2;

    const cv::Rect samplRec(tlx,tly,w,h);
    const cv::Rect entireImage(0,0,imgSize.width,imgSize.height);

    return(samplRec & entireImage);
}

cv::Mat getTrainingSamples(const cv::Mat& srcImg, const cv::Rect& samplRec)
{
    cv::Mat patch = srcImg(samplRec).clone();
    cv::Mat samples = patch.reshape(1, patch.total()).t();
    return samples;
}

void drawSamplingRectangle(cv::Mat& image)
{
    cv::Rect sampling_rectangle = samplingRectangle(image.size());
    const cv::Scalar color{0, 255, 255};
    constexpr int thickness = 10;
    cv::rectangle(image, sampling_rectangle, color, thickness);
}

cv::Mat detectGroundPlane(cv::Mat& depth)
{
    int downsamplingWidth = 128;
    double fitThreshold = 0.8;
    double fit = 0.0;
    double tolerance = 4.9;
    int maxtest = 30;

    cv::Mat downsampleImg;
    downsampleImg = depth;

    cv::Rect samplRec = samplingRectangle(depth.size());
    cv::Rect downsampleRec = samplRec;

    while (downsampleImg.cols >= downsamplingWidth )
    {
        cv::pyrDown(downsampleImg,downsampleImg);
//        cv::resize(downsampleImg,downsampleImg,downsampleImg.size()/2,0,0,cv::INTER_NEAREST);
        downsampleRec = downsampleRec - downsampleRec.size()/2;
        downsampleRec.x = downsampleRec.x - downsampleRec.x/2;
        downsampleRec.y = downsampleRec.y - downsampleRec.y/2;
    }

    cv::Mat GroundPlaneMask = cv::Mat::zeros(downsampleImg.size(),downsampleImg.type());

    int x1,y1,x2,y2,x3,y3;
    int cnt = 0;
    cv::RNG rng;
    // Check if the plane has a good enough fit within the sampling rectangle
    while (fit < fitThreshold)
    {
        cnt++;
        x1 = rng.uniform(downsampleRec.x,downsampleRec.x+ downsampleRec.width);
        x2 = rng.uniform(downsampleRec.x,downsampleRec.x+ downsampleRec.width);
        x3 = rng.uniform(downsampleRec.x,downsampleRec.x+ downsampleRec.width);
        y1 = rng.uniform(downsampleRec.y,downsampleRec.y+ downsampleRec.height);
        y2 = rng.uniform(downsampleRec.y,downsampleRec.y+ downsampleRec.height);
        y3 = rng.uniform(downsampleRec.y,downsampleRec.y+ downsampleRec.height);

//        Use 3 points to create a plane (x,y,depth)
        Eigen::Vector3d A(x1,y1,downsampleImg.at<double>(cv::Point(x1,y1)));
        Eigen::Vector3d B(x2,y2,downsampleImg.at<double>(cv::Point(x2,y2)));
        Eigen::Vector3d C(x3,y3,downsampleImg.at<double>(cv::Point(x3,y3)));

        Eigen::Vector3d u = B - A;
        Eigen::Vector3d v = C - A;
        Eigen::Vector3d normal = (v.cross(u));
        int cntFit = 0;
//      Check if the other points are within this plane, given an error tolerance
        for (int i = 0; i < downsampleRec.width; ++i) {
            for (int j = 0; j < downsampleRec.height; ++j) {
                int x = downsampleRec.x + i;
                int y = downsampleRec.y + j;
                Eigen::Vector3d P(x,y,downsampleImg.at<double>(cv::Point(x,y)));
                Eigen::Vector3d PA = P-A;
                if (-tolerance < normal.dot(PA) && normal.dot(PA) < tolerance)
                {
                    cntFit++;
                }
            }
        }
        if (cnt >= maxtest){break;}

        fit = ((double) cntFit)/downsampleRec.area();
        if (fitThreshold < fit)
        {
            for (int i = 0; i < downsampleImg.cols; ++i) {
                for (int j = 0; j < downsampleImg.rows; ++j) {
                    Eigen::Vector3d P(i, j, downsampleImg.at<double>(cv::Point(i, j)));
                    Eigen::Vector3d PA = P-A;
                    if (-tolerance < normal.dot(PA) && normal.dot(PA) < tolerance)
                    {
                        GroundPlaneMask.at<double>(cv::Point(i, j)) = 1.0;
                    }
                }
            }
        }
    }

    cv::resize(GroundPlaneMask,GroundPlaneMask,depth.size(),0,0,cv::INTER_NEAREST);
    return GroundPlaneMask;
}
