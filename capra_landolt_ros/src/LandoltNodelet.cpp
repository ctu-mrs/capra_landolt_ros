#include "ros/ros.h"

#include "image_transport/image_transport.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/objdetect.hpp>
#include "cv_bridge/cv_bridge.h"
#include <zbar.h>

#include "capra_landolt_msgs/Landolts.h"
#include "capra_landolt_msgs/BoundingCircles.h"
#include "nodelet/nodelet.h"

#include <pluginlib/class_list_macros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>
#include <vector>
#include <visualization_msgs/Marker.h>
/* for protecting variables from simultaneous by from multiple threads */
#include <mrs_lib/mutex.h>

using namespace zbar;

namespace capra
{

struct Gaps
{
  std::vector<float>       angles;
  std::vector<float>       radius;
  std::vector<cv::Point2f> centers;
};

struct DecodedObject
{
  std::string            type;
  std::string            data;
  std::vector<cv::Point> location;
};

class LandoltNodelet : public nodelet::Nodelet {
public:
  LandoltNodelet() = default;
  void onInit() override;

private:
  int         queue_size_{};
  std::string uav_name_;
  std::string camera_oakd_reading_;
  std::string camera_rgbd_reading_;
  std::string camera_basler_reading_;

  double oakd_detection_counter;
  double rgbd_detection_counter;
  double basler_detection_counter;

  int oakd_thr_min;
  int rgbd_thr_min;
  int basler_thr_min;
  int oakd_thr_max;
  int rgbd_thr_max;
  int basler_thr_max;

  cv::QRCodeDetector qrDecoder = cv::QRCodeDetector();
  cv::Mat            bbox, rectifiedImage;


  // mutexes
  bool       is_initialized_ = false;
  std::mutex is_initialized_mutex;

  // Subscriber
  boost::shared_ptr<image_transport::ImageTransport> it_oakd_;
  image_transport::CameraSubscriber                  sub_oakd_camera_;

  boost::shared_ptr<image_transport::ImageTransport> it_rgbd_;
  image_transport::CameraSubscriber                  sub_rgbd_camera_;

  boost::shared_ptr<image_transport::ImageTransport> it_basler_;
  image_transport::CameraSubscriber                  sub_basler_camera_;

  // Publisher
  image_transport::Publisher oakd_image_pub_;
  image_transport::Publisher rgbd_image_pub_;
  image_transport::Publisher basler_image_pub_;

  image_transport::Publisher oakd_image_debug_pub_;
  image_transport::Publisher rgbd_image_debug_pub_;
  image_transport::Publisher basler_image_debug_pub_;

  ros::Publisher oakd_landolt_pub_;
  ros::Publisher rgbd_landolt_pub_;
  ros::Publisher basler_landolt_pub_;

  ros::Publisher oakd_bounding_pub_;
  ros::Publisher rgbd_bounding_pub_;
  ros::Publisher basler_bounding_pub_;

  void connectOakdCb();
  void imageOakdCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);

  void connectRgbdCb();
  void imageRgbdCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);

  void connectBaslerCb();
  void imageBaslerCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);

  void findLandoltGaps(const cv::Mat& imageRaw, Gaps& gaps, int minEdge, float minRatioCircle, int minDepth, int minBinThr, int maxBinThr);

  void decodeQR(const cv::Mat& im, std::vector<DecodedObject>& decodedObjects);

  void display(const cv::Mat& im, std::vector<DecodedObject>& decodedObjects);
};


/* magnitudePoint() //{ */
float magnitudePoint(const cv::Point2f& diff) {
  return sqrtf(diff.dot(diff));
}
//}

/* normalizePoint() //{ */
cv::Point2f normalizePoint(const cv::Point2f& diff) {
  return diff / magnitudePoint(diff);
}
//}

/* angleBetween() //{ */
float angleBetween(cv::Point2f origin, cv::Point2f dest) {
  float dot = origin.x * dest.x + origin.y * dest.y;  // dot product between[x1, y1] and [x2, y2]
  float det = origin.x * dest.y - origin.y * dest.x;  // determinant
  // Get a angle between [0, 360]
  return atan2f(det, dot) * (float)(180.0 / M_PI) + 180;  // atan2(y, x) or atan2(sin, cos)
}
//}

/* onInit() //{ */
void LandoltNodelet::onInit() {
  ros::NodeHandle& nh         = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  image_transport::ImageTransport it_debug_oakd_(nh);
  image_transport::ImageTransport it_debug_rgbd_(nh);
  image_transport::ImageTransport it_debug_basler_(nh);

  // Read parameters
  private_nh.param("queue_size", queue_size_, 5);
  private_nh.param("uav_name", uav_name_, std::string("uav1"));
  private_nh.param("camera_oakd_reading", camera_oakd_reading_, std::string(uav_name_ + "/rgbd/color/image_raw"));
  ROS_INFO("Awaiting data on topic: %s", camera_oakd_reading_.c_str());
  private_nh.param("camera_rgbd_reading", camera_rgbd_reading_, std::string(uav_name_ + "/rgbd/color/image_raw"));
  ROS_INFO("Awaiting data on topic: %s", camera_rgbd_reading_.c_str());
  private_nh.param("camera_basler_reading", camera_basler_reading_, std::string(uav_name_ + "/rgbd/color/image_raw"));
  ROS_INFO("Awaiting data on topic: %s", camera_basler_reading_.c_str());

  private_nh.param("oakd/bin_threshold_min", oakd_thr_min, 144);
  private_nh.param("rgbd/bin_threshold_min", rgbd_thr_min, 144);
  private_nh.param("basler/bin_threshold_min", basler_thr_min, 255);
  private_nh.param("oakd/bin_threshold_max", oakd_thr_max, 255);
  private_nh.param("rgbd/bin_threshold_max", rgbd_thr_max, 255);
  private_nh.param("basler/bin_threshold_max", basler_thr_max, 255);

  it_oakd_.reset(new image_transport::ImageTransport(nh));
  it_rgbd_.reset(new image_transport::ImageTransport(nh));
  it_basler_.reset(new image_transport::ImageTransport(nh));

  // | ----------------------- subscribers ---------------------- |
  ros::SubscriberStatusCallback             oakd_connect_cb     = boost::bind(&LandoltNodelet::connectOakdCb, this);
  image_transport::SubscriberStatusCallback oakd_img_connect_cb = boost::bind(&LandoltNodelet::connectOakdCb, this);

  ros::SubscriberStatusCallback             rgbd_connect_cb     = boost::bind(&LandoltNodelet::connectRgbdCb, this);
  image_transport::SubscriberStatusCallback rgbd_img_connect_cb = boost::bind(&LandoltNodelet::connectRgbdCb, this);

  ros::SubscriberStatusCallback             basler_connect_cb     = boost::bind(&LandoltNodelet::connectBaslerCb, this);
  image_transport::SubscriberStatusCallback basler_img_connect_cb = boost::bind(&LandoltNodelet::connectBaslerCb, this);

  // | ----------------------- publishers ----------------------- |
  oakd_bounding_pub_    = nh.advertise<capra_landolt_msgs::BoundingCircles>("oakd/boundings_out", 1, oakd_connect_cb, oakd_connect_cb);
  oakd_landolt_pub_     = nh.advertise<capra_landolt_msgs::Landolts>("oakd/landolts_out", 1, oakd_connect_cb, oakd_connect_cb);
  oakd_image_pub_       = it_oakd_->advertise("oakd/processed_image_out", 1, oakd_img_connect_cb, oakd_img_connect_cb);
  oakd_image_debug_pub_ = it_debug_oakd_.advertise("oakd/debug/processed_image_out", 1);

  rgbd_bounding_pub_    = nh.advertise<capra_landolt_msgs::BoundingCircles>("rgbd/boundings_out", 1, rgbd_connect_cb, rgbd_connect_cb);
  rgbd_landolt_pub_     = nh.advertise<capra_landolt_msgs::Landolts>("rgbd/landolts_out", 1, rgbd_connect_cb, rgbd_connect_cb);
  rgbd_image_pub_       = it_rgbd_->advertise("rgbd/processed_image_out", 1, rgbd_img_connect_cb, rgbd_img_connect_cb);
  rgbd_image_debug_pub_ = it_debug_rgbd_.advertise("rgbd/debug/processed_image_out", 1);

  basler_bounding_pub_    = nh.advertise<capra_landolt_msgs::BoundingCircles>("basler/boundings_out", 1, basler_connect_cb, basler_connect_cb);
  basler_landolt_pub_     = nh.advertise<capra_landolt_msgs::Landolts>("basler/landolts_out", 1, basler_connect_cb, basler_connect_cb);
  basler_image_pub_       = it_basler_->advertise("basler/processed_image_out", 1, basler_img_connect_cb, basler_img_connect_cb);
  basler_image_debug_pub_ = it_debug_basler_.advertise("basler/debug/processed_image_out", 1);

  // | --------------------- finish the init -------------------- |
  // nodelet initialized = set flag is_initialized to true
  mrs_lib::set_mutexed(is_initialized_mutex, true, is_initialized_);
  ROS_INFO_ONCE("[LandoltNodelet]: initialized");
}
//}

/* connectOakdCb() //{ */
void LandoltNodelet::connectOakdCb() {

  // do not continue if the nodelet is not initialized
  if (!mrs_lib::get_mutexed(is_initialized_mutex, is_initialized_)) {
    return;
  }

  /* boost::lock_guard<boost::mutex> lock(connect_mutex_); */
  if (oakd_landolt_pub_.getNumSubscribers() == 0 && oakd_image_pub_.getNumSubscribers() == 0 && oakd_bounding_pub_.getNumSubscribers() == 0) {
    ROS_INFO("Disconnect OAKD camera to Landolt Detector...");
    sub_oakd_camera_.shutdown();
  } else if (!sub_oakd_camera_) {
    ROS_INFO("Connect OAKD camera to Landolt Detector...");
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_oakd_camera_ = it_oakd_->subscribeCamera(camera_oakd_reading_, static_cast<uint32_t>(queue_size_), &LandoltNodelet::imageOakdCb, this, hints);
  }
}
//}

/* connectRgbdCb() //{ */
void LandoltNodelet::connectRgbdCb() {

  // do not continue if the nodelet is not initialized
  if (!mrs_lib::get_mutexed(is_initialized_mutex, is_initialized_)) {
    return;
  }

  /* boost::lock_guard<boost::mutex> lock(connect_mutex_); */
  if (rgbd_landolt_pub_.getNumSubscribers() == 0 && rgbd_image_pub_.getNumSubscribers() == 0 && rgbd_bounding_pub_.getNumSubscribers() == 0) {
    ROS_INFO("Disconnect REALSENSE to Landolt Detector...");
    sub_rgbd_camera_.shutdown();
  } else if (!sub_rgbd_camera_) {
    ROS_INFO("Connect REALSENSE camera to Landolt Detector...");
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_rgbd_camera_ = it_rgbd_->subscribeCamera(camera_rgbd_reading_, static_cast<uint32_t>(queue_size_), &LandoltNodelet::imageRgbdCb, this, hints);
  }
}
//}

/* connectBaslerCb() //{ */
void LandoltNodelet::connectBaslerCb() {

  // do not continue if the nodelet is not initialized
  if (!mrs_lib::get_mutexed(is_initialized_mutex, is_initialized_)) {
    return;
  }

  /* boost::lock_guard<boost::mutex> lock(connect_mutex_); */
  if (basler_landolt_pub_.getNumSubscribers() == 0 && basler_image_pub_.getNumSubscribers() == 0 && basler_bounding_pub_.getNumSubscribers() == 0) {
    NODELET_INFO("Disconnect BASLER camera to Landolt Detector...");
    sub_basler_camera_.shutdown();
  } else if (!sub_basler_camera_) {
    NODELET_INFO("Connect BASLER camera to Landolt Detector...");
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_basler_camera_ = it_basler_->subscribeCamera(camera_basler_reading_, static_cast<uint32_t>(queue_size_), &LandoltNodelet::imageBaslerCb, this, hints);
  }
}
//}

/* imageOakdCb() //{ */
void LandoltNodelet::imageOakdCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {

  // do not continue if the nodelet is not initialized
  if (!mrs_lib::get_mutexed(is_initialized_mutex, is_initialized_)) {
    return;
  }

  cv_bridge::CvImageConstPtr img_ptr;
  try {
    img_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Find and decode QR codes
  std::vector<DecodedObject> decodedObjects;
  decodeQR(img_ptr->image, decodedObjects);

  // Find gaps
  Gaps gaps;
  findLandoltGaps(img_ptr->image, gaps, 12, 0.8f, 10, oakd_thr_min, oakd_thr_max);

  std_msgs::Header header;
  header.stamp    = ros::Time::now();
  header.frame_id = "oakd_camera";

  if (oakd_landolt_pub_.getNumSubscribers() > 0) {
    capra_landolt_msgs::Landolts landolts;
    landolts.angles = gaps.angles;
    landolts.header = header;

    oakd_landolt_pub_.publish(landolts);
  }

  if (oakd_bounding_pub_.getNumSubscribers() > 0) {
    capra_landolt_msgs::BoundingCircles boundings;
    boundings.header = header;
    boundings.radius = gaps.radius;

    std::vector<capra_landolt_msgs::Point2f> centers;
    for (auto& i : gaps.centers) {
      capra_landolt_msgs::Point2f center;
      center.x = i.x;
      center.y = i.y;
      centers.push_back(center);
    }

    boundings.centers = centers;
    oakd_bounding_pub_.publish(boundings);
  }

  if (oakd_image_pub_.getNumSubscribers() > 0) {
    cv_bridge::CvImage img_msg(header, sensor_msgs::image_encodings::BGR8, img_ptr->image);

    // Draw circle
    for (int i = 0; i < gaps.angles.size(); i++) {
      circle(img_msg.image, gaps.centers[i], static_cast<int>(gaps.radius[i]), cv::Scalar(0, 0, 1), 3);
    }

    /* // Draw box around detected QR code //{ */
    /* for (int i = 0; i < decodedObjects.size(); i++) { */
    /*   std::vector<cv::Point> points = decodedObjects[i].location; */
    /*   std::vector<cv::Point> hull; */

    /*   // If the points do not form a quad, find convex hull */
    /*   if (points.size() > 4) */
    /*     convexHull(points, hull); */
    /*   else */
    /*     hull = points; */

    /*   // Number of points in the convex hull */
    /*   int n = hull.size(); */

    /*   for (int j = 0; j < n; j++) { */
    /*     line(img_msg.image, hull[j], hull[(j + 1) % n], cv::Scalar(255, 0, 0), 3); */
    /*   } */
    /* } */
    /* //} */

    oakd_image_pub_.publish(img_msg.toImageMsg());

    ROS_INFO_THROTTLE(2, "[LandoltNodelet]: publishing oakd camera processed");

    cv::Mat thresholdMat;
    cvtColor(img_ptr->image, thresholdMat, cv::COLOR_BGR2GRAY);                            // convert to grayscale
    blur(thresholdMat, thresholdMat, cv::Size(3, 3));                                      // apply blur to grayscaled image
    threshold(thresholdMat, thresholdMat, oakd_thr_min, oakd_thr_max, cv::THRESH_BINARY);  // apply binary thresholding

    cv_bridge::CvImage img_msg_debug(header, sensor_msgs::image_encodings::MONO8, thresholdMat);
    oakd_image_debug_pub_.publish(img_msg_debug.toImageMsg());
  }
}
//}

/* imageRgbdCb() //{ */
void LandoltNodelet::imageRgbdCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {

  // do not continue if the nodelet is not initialized
  if (!mrs_lib::get_mutexed(is_initialized_mutex, is_initialized_)) {
    return;
  }

  cv_bridge::CvImageConstPtr img_ptr;
  try {
    img_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Find and decode QR codes
  std::vector<DecodedObject> decodedObjects;
  decodeQR(img_ptr->image, decodedObjects);

  // Find gaps
  Gaps gaps;
  findLandoltGaps(img_ptr->image, gaps, 12, 0.8f, 10, rgbd_thr_min, rgbd_thr_max);

  std_msgs::Header header;
  header.stamp    = ros::Time::now();
  header.frame_id = "rgbd_camera";

  if (rgbd_landolt_pub_.getNumSubscribers() > 0) {
    capra_landolt_msgs::Landolts landolts;
    landolts.angles = gaps.angles;
    landolts.header = header;

    rgbd_landolt_pub_.publish(landolts);
  }

  if (rgbd_bounding_pub_.getNumSubscribers() > 0) {
    capra_landolt_msgs::BoundingCircles boundings;
    boundings.header = header;
    boundings.radius = gaps.radius;

    std::vector<capra_landolt_msgs::Point2f> centers;
    for (auto& i : gaps.centers) {
      capra_landolt_msgs::Point2f center;
      center.x = i.x;
      center.y = i.y;
      centers.push_back(center);
    }

    boundings.centers = centers;
    rgbd_bounding_pub_.publish(boundings);
  }

  if (rgbd_image_pub_.getNumSubscribers() > 0) {
    cv_bridge::CvImage img_msg(header, sensor_msgs::image_encodings::BGR8, img_ptr->image);
    for (int i = 0; i < gaps.angles.size(); i++) {
      circle(img_msg.image, gaps.centers[i], static_cast<int>(gaps.radius[i]), cv::Scalar(0, 0, 1), 3);
    }

    /* // Draw box around detected QR codes //{ */
    /* for (int i = 0; i < decodedObjects.size(); i++) { */
    /*   std::vector<cv::Point> points = decodedObjects[i].location; */
    /*   std::vector<cv::Point> hull; */

    /*   // If the points do not form a quad, find convex hull */
    /*   if (points.size() > 4) */
    /*     convexHull(points, hull); */
    /*   else */
    /*     hull = points; */

    /*   // Number of points in the convex hull */
    /*   int n = hull.size(); */

    /*   for (int j = 0; j < n; j++) { */
    /*     line(img_msg.image, hull[j], hull[(j + 1) % n], cv::Scalar(255, 0, 0), 3); */
    /*   } */
    /* } */
    /* //} */

    rgbd_image_pub_.publish(img_msg.toImageMsg());

    ROS_INFO_THROTTLE(2, "[LandoltNodelet]: publishing rgbd camera processed");

    cv::Mat thresholdMat;
    cvtColor(img_ptr->image, thresholdMat, cv::COLOR_BGR2GRAY);                            // convert to grayscale
    blur(thresholdMat, thresholdMat, cv::Size(3, 3));                                      // apply blur to grayscaled image
    threshold(thresholdMat, thresholdMat, rgbd_thr_min, rgbd_thr_max, cv::THRESH_BINARY);  // apply binary thresholding

    cv_bridge::CvImage img_msg_debug(header, sensor_msgs::image_encodings::MONO8, thresholdMat);
    rgbd_image_debug_pub_.publish(img_msg_debug.toImageMsg());
  }
}
//}

/* imageBaslerCb() //{ */
void LandoltNodelet::imageBaslerCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {

  // do not continue if the nodelet is not initialized
  if (!mrs_lib::get_mutexed(is_initialized_mutex, is_initialized_)) {
    return;
  }

  cv_bridge::CvImageConstPtr img_ptr;
  try {
    img_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Find and decode QR codes
  std::vector<DecodedObject> decodedObjects;
  decodeQR(img_ptr->image, decodedObjects);

  // Find gaps
  Gaps gaps;
  findLandoltGaps(img_ptr->image, gaps, 12, 0.8f, 10, basler_thr_min, basler_thr_max);

  std_msgs::Header header;
  header.stamp    = ros::Time::now();
  header.frame_id = "basler_camera";

  if (basler_landolt_pub_.getNumSubscribers() > 0) {
    capra_landolt_msgs::Landolts landolts;
    landolts.angles = gaps.angles;
    landolts.header = header;

    basler_landolt_pub_.publish(landolts);
  }

  if (basler_bounding_pub_.getNumSubscribers() > 0) {
    capra_landolt_msgs::BoundingCircles boundings;
    boundings.header = header;
    boundings.radius = gaps.radius;

    std::vector<capra_landolt_msgs::Point2f> centers;
    for (auto& i : gaps.centers) {
      capra_landolt_msgs::Point2f center;
      center.x = i.x;
      center.y = i.y;
      centers.push_back(center);
    }

    boundings.centers = centers;
    basler_bounding_pub_.publish(boundings);
  }

  if (basler_image_pub_.getNumSubscribers() > 0) {
    cv_bridge::CvImage img_msg(header, sensor_msgs::image_encodings::BGR8, img_ptr->image);
    for (int i = 0; i < gaps.angles.size(); i++) {
      circle(img_msg.image, gaps.centers[i], static_cast<int>(gaps.radius[i]), cv::Scalar(0, 0, 1), 3);
    }

    /* // Draw box around detected QR codes //{ */
    /* for (int i = 0; i < decodedObjects.size(); i++) { */
    /*   std::vector<cv::Point> points = decodedObjects[i].location; */
    /*   std::vector<cv::Point> hull; */

    /*   // If the points do not form a quad, find convex hull */
    /*   if (points.size() > 4) */
    /*     convexHull(points, hull); */
    /*   else */
    /*     hull = points; */

    /*   // Number of points in the convex hull */
    /*   int n = hull.size(); */

    /*   for (int j = 0; j < n; j++) { */
    /*     line(img_msg.image, hull[j], hull[(j + 1) % n], cv::Scalar(255, 0, 0), 3); */
    /*   } */
    /* } */
    /* //} */

    basler_image_pub_.publish(img_msg.toImageMsg());

    ROS_INFO_THROTTLE(2, "[LandoltNodelet]: publishing basler camera processed");

    cv::Mat thresholdMat;
    cvtColor(img_ptr->image, thresholdMat, cv::COLOR_BGR2GRAY);                                // convert to grayscale
    blur(thresholdMat, thresholdMat, cv::Size(3, 3));                                          // apply blur to grayscaled image
    threshold(thresholdMat, thresholdMat, basler_thr_min, basler_thr_max, cv::THRESH_BINARY);  // apply binary thresholding

    cv_bridge::CvImage img_msg_debug(header, sensor_msgs::image_encodings::MONO8, thresholdMat);
    basler_image_debug_pub_.publish(img_msg_debug.toImageMsg());
  }
}
//}

/* findLandoltGaps() //{ */
void LandoltNodelet::findLandoltGaps(const cv::Mat& imageRaw, Gaps& gaps, int minEdge, float minRatioCircle, int minDepth, int minBinThr, int maxBinThr) {
  cv::Mat thresholdMat;
  cvtColor(imageRaw, thresholdMat, cv::COLOR_BGR2GRAY);                            // convert to grayscale
  blur(thresholdMat, thresholdMat, cv::Size(3, 3));                                // apply blur to grayscaled image
  threshold(thresholdMat, thresholdMat, minBinThr, maxBinThr, cv::THRESH_BINARY);  // apply binary thresholding

  std::vector<std::vector<cv::Point>> contours;  // list of contour points
  findContours(thresholdMat, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  for (auto& contour : contours) {

    if (contour.size() > minEdge) {
      //@todo: Look if approxPolyDP would give better results.
      // vector<Point> approx;
      // approxPolyDP(contours[i], approx, arcLength(contours[i], true) * 0.001, true);

      std::vector<cv::Point> hull;
      convexHull(contour, hull, true, true);
      double hullArea = cv::contourArea(hull);

      float       contourRadius;
      cv::Point2f contourCenter;
      minEnclosingCircle(contour, contourCenter, contourRadius);
      double minArea = contourRadius * contourRadius * M_PI;

      if (hullArea / minArea > minRatioCircle) {
        std::vector<cv::Vec4i> defects;
        std::vector<int>       hullsI;
        convexHull(contour, hullsI, true, false);
        convexityDefects(contour, hullsI, defects);

        std::vector<cv::Vec4i> deepDefects;
        for (const auto& v : defects) {
          float depth = (float)v[3] / 256;
          if (depth > minDepth) {
            deepDefects.push_back(v);
          }
        }

        if (deepDefects.size() == 1) {
          const cv::Vec4i& v = deepDefects[0];

          int startidx = v[0];
          int endidx   = v[1];
          int faridx   = v[2];

          std::vector<cv::Point> points;
          points.emplace_back(contour[startidx]);
          points.emplace_back(contour[endidx]);

          float       defectRadius;
          cv::Point2f defectCenter;
          minEnclosingCircle(points, defectCenter, defectRadius);
          // Recenter the cercle at the defectCenter of the gaps
          cv::Point2f dir = normalizePoint(cv::Point2f(contour[faridx]) - defectCenter);
          defectCenter += dir * defectRadius;

          float defectAngle = angleBetween(dir, cv::Point2f(1, 0));

          gaps.angles.push_back(defectAngle);
          gaps.radius.push_back(defectRadius);
          gaps.centers.push_back(defectCenter);
        }
      }
    }
  }
}
//}

/* decodeQR() //{ */
void LandoltNodelet::decodeQR(const cv::Mat& im, std::vector<DecodedObject>& decodedObjects) {

  // Create zbar scanner
  ImageScanner scanner;

  // Configure scanner
  scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);

  // Convert image to grayscale
  cv::Mat imGray;
  cvtColor(im, imGray, CV_BGR2GRAY);

  // Wrap image data in a zbar image
  Image image(im.cols, im.rows, "Y800", (uchar*)imGray.data, im.cols * im.rows);

  // Scan the image for barcodes and QRCodes
  int n = scanner.scan(image);

  // Print results
  for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
    DecodedObject obj;
    obj.type = symbol->get_type_name();
    obj.data = symbol->get_data();
    // Print type and data
    /* std::cout << "Type : " << obj.type << std::endl; */
    std::cout << "Data : " << obj.data << std::endl << std::endl;
    // Obtain location
    for (int i = 0; i < symbol->get_location_size(); i++) {
      obj.location.push_back(cv::Point(symbol->get_location_x(i), symbol->get_location_y(i)));
    }
    decodedObjects.push_back(obj);
  }
}
//}

// Display barcode and QR code location //{
void LandoltNodelet::display(const cv::Mat& im, std::vector<DecodedObject>& decodedObjects) {
  // Loop over all decoded objects
  for (int i = 0; i < decodedObjects.size(); i++) {
    std::vector<cv::Point> points = decodedObjects[i].location;
    std::vector<cv::Point> hull;

    // If the points do not form a quad, find convex hull
    if (points.size() > 4)
      convexHull(points, hull);
    else
      hull = points;

    // Number of points in the convex hull
    int n = hull.size();

    for (int j = 0; j < n; j++) {
      line(im, hull[j], hull[(j + 1) % n], cv::Scalar(255, 0, 0), 3);
    }
  }

  std_msgs::Header header;
  header.stamp    = ros::Time::now();
  header.frame_id = "oakd_camera";

  cv_bridge::CvImage img_msg(header, sensor_msgs::image_encodings::BGR8, im);
  // Display results
  oakd_image_pub_.publish(img_msg.toImageMsg());
}
//}

PLUGINLIB_EXPORT_CLASS(capra::LandoltNodelet, nodelet::Nodelet)
}  // namespace capra
