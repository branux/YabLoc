#include "common/util.hpp"
#include "imgproc/graph_segment.hpp"

#include <opencv4/opencv2/imgproc.hpp>

namespace imgproc
{
GraphSegment::GraphSegment() : Node("graph_segment")
{
  using std::placeholders::_1;

  // Subscriber
  sub_image_ = create_subscription<Image>(
    "/sensing/camera/traffic_light/image_raw/compressed", 10,
    std::bind(&GraphSegment::callbackImage, this, _1));

  pub_cloud_ = create_publisher<PointCloud2>("/graph_segmented", 10);
  pub_image_ = create_publisher<Image>("/segmented_image", 10);

  segmentation_ = cv::ximgproc::segmentation::createGraphSegmentation();
}

void GraphSegment::callbackImage(const Image & msg)
{
  cv::Mat image = util::decompress2CvMat(msg);
  cv::Mat resized;
  cv::resize(image, resized, cv::Size(), 0.5, 0.5);

  cv::Mat segmented;
  segmentation_->processImage(resized, segmented);

  // TODO: THIS IS EFFICIENT BUT STUPID
  int target_class = segmented.at<int>(cv::Point2i(resized.cols / 2, resized.rows * 0.8));

  cv::Mat output_image = cv::Mat::zeros(resized.size(), CV_8UC1);
  for (int w = 0; w < resized.cols; w++) {
    for (int h = 0; h < resized.rows; h++) {
      cv::Point2i px(w, h);
      if (segmented.at<int>(px) == target_class) output_image.at<uchar>(px) = 255;
    }
  }
  cv::resize(output_image, output_image, image.size(), 0, 0, cv::INTER_NEAREST);

  // Convert segmented area to polygon
  std::vector<std::vector<cv::Point2i>> contours;
  cv::findContours(output_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if (contours.size() == 0) {
    RCLCPP_WARN_STREAM(this->get_logger(), "there are no contours");
    return;
  }
  auto & hull = contours.front();
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (const cv::Point2i p : hull) {
    pcl::PointXYZ xyz(p.x, p.y, 0);
    cloud.push_back(xyz);
  }
  util::publishCloud(*pub_cloud_, cloud, msg.header.stamp);

  publishImage(image, segmented, msg.header.stamp);
}

void GraphSegment::publishImage(
  const cv::Mat & raw_image, const cv::Mat & segmentation, const rclcpp::Time & stamp)
{
  const cv::Size size = segmentation.size();
  int target_class = segmentation.at<int>(cv::Point2i(size.width / 2, size.height * 0.8));

  auto randomHsv = [](int index) -> cv::Scalar {
    double base = (double)(index)*0.618033988749895 + 0.24443434;
    return cv::Scalar(fmod(base, 1.2) * 255, 0.70 * 255, 0.5 * 255);
  };

  cv::Mat segmented_image = cv::Mat::zeros(size, CV_8UC3);
  for (int i = 0; i < segmented_image.rows; i++) {
    const int * p = segmentation.ptr<int>(i);
    uchar * p2 = segmented_image.ptr<uchar>(i);
    for (int j = 0; j < segmented_image.cols; j++) {
      cv::Scalar color(30, 255, 255);
      if (p[j] != target_class) color = randomHsv(p[j]);
      p2[j * 3] = (uchar)color[0];
      p2[j * 3 + 1] = (uchar)color[1];
      p2[j * 3 + 2] = (uchar)color[2];
    }
  }
  cv::cvtColor(segmented_image, segmented_image, cv::COLOR_HSV2BGR);
  cv::resize(segmented_image, segmented_image, raw_image.size(), 0, 0, cv::INTER_NEAREST);

  cv::Mat show_image;
  cv::addWeighted(raw_image, 0.5, segmented_image, 0.8, 1.0, show_image);
  util::publishImage(*pub_image_, show_image, stamp);
}

}  // namespace imgproc

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imgproc::GraphSegment>());
  rclcpp::shutdown();
  return 0;
}