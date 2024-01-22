# capra_landolt_ros

capra_landolt_ros detect landolt C in a camera feed in ROS

### Landolt Detector
Landolt Detector is a detector using OpenCV to find gap in Landolt C in a camera feed. It search contours with circular shape and look for hole in the shapes.
The techniques used to find the landolt are in order:

#### Pre-process
- Transform the image to grayscale
- Blur the grayscale
- Apply binary thresholding 
- Find contours

#### Process
- From the contours, the detector remove any contour smaller than a min value. 
- After that, the detector try to find if the contour is circular. To do that, the detector create a hull of the contour and compute the area of the hull. The detector compute a ratio by divinding the hull area with the min circle area. With that, the detector compare the ratio with a min threshold.
- From the hull, the detector search for [defect](https://docs.opencv.org/2.4/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html?highlight=convexhull#convexitydefects) (concavity) in the form. The detector find all defects in a hull and keep the start, the end and the farthest point of the defect. The detector compare the number of defect and keep any defect with only one defect wich reprensented by the hole in the landolt C.

#### Drawing
- The detector find the min enclosing cirlce of the start and the end point of the defect to draw on the figure. Because the start and end point are the entry points of the hole, the detector need to center it. The detector use the farthest point and center of the circle to compute a normalize vector and multiply it by the radius of the circle.

#### Example

| Binary Threshold | Defect Convexity | Offset Circle | Final |
| ------ | ------ | ------ | ------ |
| <img src="./capra_landolt_ros/doc/threshold.png" width="200"> | <img src="./capra_landolt_ros/doc/convexity.png" width="200"> | <img src="./capra_landolt_ros/doc/offset.jpg" width="200"> | <img src="./capra_landolt_ros/doc/final.jpg" width="200"> |

## Limitation

- The detector try to find circular hull. By doing that, it can invalidate ellipse and other form which can happen if in angle or if a part is outside the camera. 
- If the landolt C is damage some hole could happen. In this case, the landolt will be invalidate. A solution could be to approximate the shape before doing the processing.

## MRS extensions
- subscribes 
  * realsense rgbd camera image
  * oakd rgbd camera image
  * basler camera image
- republish grayscale image for tunnig some parameters (use config file)
- using zbar library to decode QR tags

## TODO:
1. add qr code detector https://learnopencv.com/opencv-qr-code-scanner-c-and-python/ ..... for this we need custom build of the libopencv-dev which is not supported right now
2. https://wiki.ros.org/zbar_ros
3. https://answers.ros.org/question/9299/how-to-display-a-mesh-with-texture-in-rviz/
