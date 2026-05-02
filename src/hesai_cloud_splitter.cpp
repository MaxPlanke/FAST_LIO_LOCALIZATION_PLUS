#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <limits>
#include <string>
#include <vector>

class HesaiCloudSplitter
{
public:
  HesaiCloudSplitter()
    : nh_(), private_nh_("~")
  {
    private_nh_.param<std::string>("input_topic", input_topic_, "/hesai/pandar");
    private_nh_.param<std::string>("output_topic", output_topic_, "/hesai/pandar_split");
    private_nh_.param<std::string>("timestamp_field", timestamp_field_, "timestamp");
    private_nh_.param<double>("split_interval", split_interval_, 0.01);
    private_nh_.param<int>("queue_size", queue_size_, 100);
    private_nh_.param<bool>("use_msg_stamp_as_start", use_msg_stamp_as_start_, true);
    private_nh_.param<bool>("stamp_output_with_bucket_start", stamp_output_with_bucket_start_, true);
    private_nh_.param<bool>("publish_in_scan_time", publish_in_scan_time_, true);

    if (split_interval_ <= 0.0)
    {
      ROS_WARN("split_interval must be positive, reset to 0.01 sec");
      split_interval_ = 0.01;
    }

    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, queue_size_);
    sub_ = nh_.subscribe(input_topic_, queue_size_, &HesaiCloudSplitter::cloudCallback, this);

    ROS_INFO_STREAM("hesai_cloud_splitter input: " << input_topic_
                    << ", output: " << output_topic_
                    << ", split_interval: " << split_interval_
                    << " sec, publish_in_scan_time: " << publish_in_scan_time_);
  }

private:
  bool findTimestampField(const sensor_msgs::PointCloud2& msg, sensor_msgs::PointField& field) const
  {
    for (const auto& item : msg.fields)
    {
      if (item.name == timestamp_field_)
      {
        field = item;
        return true;
      }
    }
    return false;
  }

  bool readTimestamp(const sensor_msgs::PointCloud2& msg,
                     const sensor_msgs::PointField& field,
                     uint32_t point_index,
                     double& timestamp) const
  {
    const uint32_t row = point_index / msg.width;
    const uint32_t col = point_index % msg.width;
    const size_t offset = static_cast<size_t>(row) * msg.row_step +
                          static_cast<size_t>(col) * msg.point_step + field.offset;

    if (offset + fieldSize(field.datatype) > msg.data.size())
    {
      return false;
    }

    if (field.datatype == sensor_msgs::PointField::FLOAT64)
    {
      std::memcpy(&timestamp, &msg.data[offset], sizeof(double));
      return true;
    }

    if (field.datatype == sensor_msgs::PointField::FLOAT32)
    {
      float value = 0.0f;
      std::memcpy(&value, &msg.data[offset], sizeof(float));
      timestamp = static_cast<double>(value);
      return true;
    }

    return false;
  }

  size_t fieldSize(uint8_t datatype) const
  {
    switch (datatype)
    {
      case sensor_msgs::PointField::INT8:
      case sensor_msgs::PointField::UINT8:
        return 1;
      case sensor_msgs::PointField::INT16:
      case sensor_msgs::PointField::UINT16:
        return 2;
      case sensor_msgs::PointField::INT32:
      case sensor_msgs::PointField::UINT32:
      case sensor_msgs::PointField::FLOAT32:
        return 4;
      case sensor_msgs::PointField::FLOAT64:
        return 8;
      default:
        return 0;
    }
  }

  void appendPoint(const sensor_msgs::PointCloud2& msg,
                   uint32_t point_index,
                   std::vector<uint8_t>& data) const
  {
    const uint32_t row = point_index / msg.width;
    const uint32_t col = point_index % msg.width;
    const size_t offset = static_cast<size_t>(row) * msg.row_step +
                          static_cast<size_t>(col) * msg.point_step;
    data.insert(data.end(), msg.data.begin() + offset, msg.data.begin() + offset + msg.point_step);
  }

  void publishBucket(const sensor_msgs::PointCloud2& msg,
                     const sensor_msgs::PointField& timestamp_field,
                     const std::vector<uint32_t>& point_indices) const
  {
    if (point_indices.empty())
    {
      return;
    }

    double bucket_start_time = std::numeric_limits<double>::max();
    for (uint32_t point_index : point_indices)
    {
      double timestamp = 0.0;
      if (readTimestamp(msg, timestamp_field, point_index, timestamp) && std::isfinite(timestamp))
      {
        bucket_start_time = std::min(bucket_start_time, timestamp);
      }
    }

    if (bucket_start_time == std::numeric_limits<double>::max())
    {
      bucket_start_time = msg.header.stamp.toSec();
    }

    sensor_msgs::PointCloud2 output;
    output.header = msg.header;
    if (stamp_output_with_bucket_start_)
    {
      output.header.stamp = ros::Time(bucket_start_time);
    }
    output.height = 1;
    output.width = point_indices.size();
    output.fields = msg.fields;
    output.is_bigendian = msg.is_bigendian;
    output.point_step = msg.point_step;
    output.row_step = output.point_step * output.width;
    output.is_dense = msg.is_dense;
    output.data.reserve(static_cast<size_t>(output.row_step));

    for (uint32_t point_index : point_indices)
    {
      appendPoint(msg, point_index, output.data);
    }

    pub_.publish(output);
  }

  void waitForBucketStep(int previous_bucket_index, int bucket_index) const
  {
    if (!publish_in_scan_time_ || previous_bucket_index < 0)
    {
      return;
    }

    const int bucket_step = bucket_index - previous_bucket_index;
    if (bucket_step <= 0)
    {
      return;
    }

    const ros::Duration delay(bucket_step * split_interval_);
    if (delay.toSec() > 0.0)
    {
      delay.sleep();
    }
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    if (msg->height == 0 || msg->width == 0 || msg->point_step == 0)
    {
      return;
    }

    sensor_msgs::PointField timestamp_field;
    if (!findTimestampField(*msg, timestamp_field))
    {
      ROS_WARN_THROTTLE(1.0, "PointCloud2 has no timestamp field named '%s'", timestamp_field_.c_str());
      return;
    }

    if (timestamp_field.datatype != sensor_msgs::PointField::FLOAT64 &&
        timestamp_field.datatype != sensor_msgs::PointField::FLOAT32)
    {
      ROS_WARN_THROTTLE(1.0, "Timestamp field '%s' must be FLOAT64 or FLOAT32", timestamp_field_.c_str());
      return;
    }

    const uint32_t point_num = msg->height * msg->width;
    double first_timestamp = 0.0;
    if (!readTimestamp(*msg, timestamp_field, 0, first_timestamp) || !std::isfinite(first_timestamp))
    {
      ROS_WARN_THROTTLE(1.0, "Failed to read first point timestamp");
      return;
    }

    const double start_time = use_msg_stamp_as_start_ ? msg->header.stamp.toSec() : first_timestamp;
    std::map<int, std::vector<uint32_t>> buckets;

    for (uint32_t i = 0; i < point_num; ++i)
    {
      double timestamp = 0.0;
      if (!readTimestamp(*msg, timestamp_field, i, timestamp) || !std::isfinite(timestamp))
      {
        continue;
      }

      int bucket_index = static_cast<int>(std::floor((timestamp - start_time) / split_interval_));
      bucket_index = std::max(0, bucket_index);
      buckets[bucket_index].push_back(i);
    }

    int previous_bucket_index = -1;
    for (const auto& bucket : buckets)
    {
      waitForBucketStep(previous_bucket_index, bucket.first);
      publishBucket(*msg, timestamp_field, bucket.second);
      previous_bucket_index = bucket.first;
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  std::string input_topic_;
  std::string output_topic_;
  std::string timestamp_field_;
  double split_interval_;
  int queue_size_;
  bool use_msg_stamp_as_start_;
  bool stamp_output_with_bucket_start_;
  bool publish_in_scan_time_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hesai_cloud_splitter");
  HesaiCloudSplitter splitter;
  ros::spin();
  return 0;
}
