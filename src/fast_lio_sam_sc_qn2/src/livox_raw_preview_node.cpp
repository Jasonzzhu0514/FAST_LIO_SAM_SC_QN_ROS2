#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{
sensor_msgs::msg::PointField makeField(const std::string &name, const uint32_t offset)
{
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    return field;
}

void writeFloat(std::vector<uint8_t> &data, const size_t offset, const float value)
{
    std::memcpy(data.data() + offset, &value, sizeof(float));
}
}  // namespace

class LivoxRawPreviewNode : public rclcpp::Node
{
public:
    LivoxRawPreviewNode()
        : Node("livox_raw_preview_node")
    {
        input_topic_ = declare_parameter<std::string>("input_topic", "/livox/lidar");
        output_topic_ = declare_parameter<std::string>("output_topic", "/web_mapping/raw_cloud");
        status_topic_ = declare_parameter<std::string>("status_topic", "/web_mapping/lidar_status");
        frame_id_override_ = declare_parameter<std::string>("frame_id", "");
        max_points_ = declare_parameter<int>("max_points", 1000);
        min_interval_sec_ = declare_parameter<double>("min_interval_sec", 0.0);

        publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::SensorDataQoS());
        status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic_, rclcpp::QoS(1).reliable());
        auto input_qos = rclcpp::QoS(rclcpp::KeepLast(16)).reliable();
        subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
            input_topic_,
            input_qos,
            std::bind(&LivoxRawPreviewNode::callback, this, std::placeholders::_1));
        preview_timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&LivoxRawPreviewNode::publishPreview, this));
        status_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&LivoxRawPreviewNode::publishStatus, this));

        RCLCPP_INFO(
            get_logger(),
            "Livox raw preview: %s -> %s, max_points=%d, min_interval=%.3fs",
            input_topic_.c_str(),
            output_topic_.c_str(),
            max_points_,
            min_interval_sec_);
    }

private:
    void callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        const auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(mutex_);
        recordInputFrame(now, msg->point_num);
        latest_msg_ = msg;
        latest_generation_ = input_frames_;
    }

    void publishPreview()
    {
        const auto now = std::chrono::steady_clock::now();
        if (min_interval_sec_ > 0.0 && last_publish_.time_since_epoch().count() > 0)
        {
            const auto elapsed = std::chrono::duration<double>(now - last_publish_).count();
            if (elapsed < min_interval_sec_)
            {
                return;
            }
        }

        livox_ros_driver2::msg::CustomMsg::SharedPtr msg;
        uint64_t generation = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!latest_msg_ || latest_generation_ == last_published_generation_)
            {
                return;
            }
            msg = latest_msg_;
            generation = latest_generation_;
        }

        const size_t available_points = std::min<size_t>(msg->point_num, msg->points.size());
        if (available_points == 0)
        {
            return;
        }

        const size_t point_budget = max_points_ <= 0 ? available_points : static_cast<size_t>(max_points_);
        const size_t sample_every = std::max<size_t>(1, (available_points + point_budget - 1) / point_budget);
        const size_t planned_points = (available_points + sample_every - 1) / sample_every;

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header = msg->header;
        if (!frame_id_override_.empty())
        {
            cloud.header.frame_id = frame_id_override_;
        }
        cloud.height = 1;
        cloud.is_bigendian = false;
        cloud.is_dense = false;
        cloud.point_step = 16;
        cloud.fields = {
            makeField("x", 0),
            makeField("y", 4),
            makeField("z", 8),
            makeField("intensity", 12),
        };
        cloud.data.resize(planned_points * cloud.point_step);

        size_t accepted_points = 0;
        for (size_t point_index = 0; point_index < available_points; point_index += sample_every)
        {
            const auto &point = msg->points[point_index];
            const float x = point.x;
            const float y = point.y;
            const float z = point.z;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            {
                continue;
            }

            const size_t offset = accepted_points * cloud.point_step;
            writeFloat(cloud.data, offset, x);
            writeFloat(cloud.data, offset + 4, y);
            writeFloat(cloud.data, offset + 8, z);
            writeFloat(cloud.data, offset + 12, static_cast<float>(point.reflectivity));
            ++accepted_points;
        }

        if (accepted_points == 0)
        {
            return;
        }
        cloud.width = static_cast<uint32_t>(accepted_points);
        cloud.row_step = cloud.width * cloud.point_step;
        cloud.data.resize(cloud.row_step);

        publisher_->publish(cloud);
        last_publish_ = now;
        std::lock_guard<std::mutex> lock(mutex_);
        last_published_generation_ = std::max(last_published_generation_, generation);
    }

    void recordInputFrame(const std::chrono::steady_clock::time_point now, const uint32_t point_count)
    {
        ++input_frames_;
        last_input_points_ = point_count;
        last_input_ = now;
        input_times_.push_back(now);
        while (input_times_.size() > 64)
        {
            input_times_.erase(input_times_.begin());
        }
    }

    void publishStatus()
    {
        const auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(mutex_);
        double hz = 0.0;
        if (input_times_.size() >= 2)
        {
            const auto elapsed = std::chrono::duration<double>(input_times_.back() - input_times_.front()).count();
            if (elapsed > 0.0)
            {
                hz = static_cast<double>(input_times_.size() - 1) / elapsed;
            }
        }
        const double age = last_input_.time_since_epoch().count() > 0
            ? std::chrono::duration<double>(now - last_input_).count()
            : -1.0;
        const char *state = age >= 0.0 && age < 2.0 ? "online" : "stale";

        std_msgs::msg::String status;
        status.data =
            std::string("{\"type\":\"lidar_status\"")
            + ",\"topic\":\"" + input_topic_ + "\""
            + ",\"state\":\"" + state + "\""
            + ",\"hz\":" + std::to_string(hz)
            + ",\"age_sec\":" + std::to_string(age)
            + ",\"last_points\":" + std::to_string(last_input_points_)
            + ",\"frames\":" + std::to_string(input_frames_)
            + "}";
        status_publisher_->publish(status);
    }

    std::string input_topic_;
    std::string output_topic_;
    std::string status_topic_;
    std::string frame_id_override_;
    int max_points_ = 1000;
    double min_interval_sec_ = 0.0;
    std::mutex mutex_;
    std::chrono::steady_clock::time_point last_publish_{};
    std::chrono::steady_clock::time_point last_input_{};
    std::vector<std::chrono::steady_clock::time_point> input_times_;
    uint64_t input_frames_ = 0;
    uint64_t latest_generation_ = 0;
    uint64_t last_published_generation_ = 0;
    uint32_t last_input_points_ = 0;
    livox_ros_driver2::msg::CustomMsg::SharedPtr latest_msg_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr preview_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LivoxRawPreviewNode>());
    rclcpp::shutdown();
    return 0;
}
