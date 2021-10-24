# TEK5030 project - Object following vehicle
This project was one in colaberation with two fellow students.
This repo contains the source code for a project in the course TEK5030, 
with procedures for collision detection and object following.
As part of the collision detection,  procedures for ground segmentation are implemented.


## About the files
- *CMakeLists.txt*

  The cmake project file.

- *main.cpp*

  Contains the main function. Visualization is done within the main function loop. 
  It also contains the collision detection function, which takes the
  arguments; minimum and maximum distance a collision can be detected, 
  the number of pixels needed for a collision to be detected, and the depth image that is to be checked.
  ```cpp
  bool ChkCollision(double min_lim, double max_lim, int N_min, cv::Mat depthImg)
  ```

- *segmentation.cpp/h*
  
  Contains segmentation procedures. The following functions are available:
  - Color segmentation 
  - Ground plane estimation
  -  Drawing the sampling rectangle.
  ```cpp
  cv::Mat segmentation(cv::Mat frame, cv::Mat& currentSamples,  bool init);
  cv::Mat detectGroundPlane(cv::Mat& depth);
  void drawSamplingRectangle(cv::Mat& image);
  ```

- *object_follow.cpp* 

  Contains the ArUco controller class. The following functions are available to the class: 
  - Detection of ArUco markers
  - Visualization of detected aruco markers
  - Calculating the markers center position
  - Depth controller
  - Horizontal controller
  ```cpp
  class ArucoController
      void Detect(cv:: Mat rgbImage);
      void VisDetect(const cv::Mat& rgbImageCopy);
      void CalcCenter();
      void DepthControl(float sp_off, float sp_on, cv::Mat depthImg, bool& forward );
      float HorizControl(cv::Mat rgbImg);
  ```  


- *controlsImg*

  Folder containing the controller output images.

- *marker23.png*

  ArUco marker used for the object following. 

## Dependecies
- OpenCv
- Eigen
- [tek5030 camera library](https://github.com/tek5030/camera-library)
- [librealsense](https://github.com/IntelRealSense/librealsense)

## How to use the project
- Clone this repository to your computer at a desired location.
- Build and run the project. It should run out of the box.
- By default, six image windows will show up:
  1. The camera feed together with the segmented depth image.
  2. The depth image.
  3. The camera feed with a visualization of ArUco markers detected.
  4. The segmented depth image and the segmented depth image with the ground plane estimation mask applied.
  5. The controller output. 
  6. A four view window with: The camera feed, the depth image, the segmented depth image 
     and the segmented depth image with the ground plane estimation mask applied.  
- Press any key to close the program.
- Tweak and change the  settings as desired. 
