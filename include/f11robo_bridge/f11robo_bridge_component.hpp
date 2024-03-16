#pragma once
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
// boost
#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
// ros2
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
// tf2
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "../f11robo_msg/f11robo_msg.hpp"
#include "../kinematics/move/move_base.hpp"
#include "../kinematics/move/two_wheels.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class F11RoboBridge : public rclcpp::Node
{
public:
  F11RoboBridge(const rclcpp::NodeOptions &options) : F11RoboBridge("", options) {}
  F11RoboBridge(
      const std::string &name_space = "",
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("f11robo_bridge_node", name_space, options), broadcaster_(this), tf_buffer_(this->get_clock()), listener_(tf_buffer_)
  {
    RCLCPP_INFO(this->get_logger(), "start f11robo_bridge_node");
    CMD_VEL_TOPIC = param<std::string>("f11robo_bridge.topic_name.cmd_vel", "/cmd_vel");
    ODOM_FRAME = param<std::string>("f11robo_bridge.frame_id.odom", "odom");
    IMU_FRAME = param<std::string>("f11robo_bridge.frame_id.imu_link", "imu_link");
    BASE_FRAME = param<std::string>("f11robo_bridge.frame_id.base_link", "base_link");
    std::vector<double> l_pid_gain = param<std::vector<double>>("f11robo_bridge.pid_gain.left_md", std::vector<double>{0.1, 0.0, 0.008});
    std::vector<double> r_pid_gain = param<std::vector<double>>("f11robo_bridge.pid_gain.right_md", std::vector<double>{0.1, 0.0, 0.008});
    MAX_VEL = param<double>("f11robo_bridge.max.vel", 0.5);
    MAX_ANGULAR = param<double>("f11robo_bridge.max.angular", 1.0);
    std::string port = param<std::string>("f11robo_bridge.serial.port", "/dev/ttyUSB0");
    int baudrate = param<int>("f11robo_bridge.serial.baudrate", 115200);
    double BROADCAST_PERIOD = param<double>("f11robo_bridge.broadcast_period", 0.001);
    reset_odom_func_ = param<bool>("f11robo_bridge.odom_reset_button", false);
    RCLCPP_INFO(this->get_logger(), "port:%s", port.c_str());
    RCLCPP_INFO(this->get_logger(), "baudrate:%d", baudrate);
    move_ = std::make_shared<kinematics::TwoWheelsd>(f11robo::param::R, f11robo::param::L);
    serial_ = std::make_shared<boost::asio::serial_port>(io);
    serial_->open(port);
    serial_->set_option(boost::asio::serial_port_base::baud_rate(baudrate));
    cmd_vel_ = stop();
    tf_output_ = param<bool>("f11robo_bridge.tf_output", true);
    debug_output_ = param<bool>("f11robo_bridge.debug_output", false);
    use_imu_ = param<bool>("f11robo_bridge.odometry.use_imu", false);
    std::vector<bool> rpy_inv = param<std::vector<bool>>("f11robo_bridge.rpy_inversion", std::vector<bool>{false, false, false});
    for (int i = 0; i < 3; i++)
      rpy_dir_.push_back(rpy_inv[i] ? -1.0 : 1.0);
    RCLCPP_INFO(this->get_logger(), "rpy_dir:%f,%f,%f", rpy_dir_[0], rpy_dir_[1], rpy_dir_[2]);
    // publisher
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::QoS(10));
    light_sensor_pub_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("sensor/lights", rclcpp::QoS(10));
    sw_pub_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("sensor/switchs", rclcpp::QoS(10));
    ems_pub_ = this->create_publisher<std_msgs::msg::Bool>("emergency_stop", rclcpp::QoS(10));
    button_pub_ = this->create_publisher<std_msgs::msg::Bool>("button", rclcpp::QoS(10));
    ems_status_pub_ = this->create_publisher<std_msgs::msg::String>("status/emergency_stop", rclcpp::QoS(10));
    buttery_status_pub_ = this->create_publisher<std_msgs::msg::Float32>("status/battery_voltage", rclcpp::QoS(10));
    battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery", rclcpp::QoS(10));
    // subscriber
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        CMD_VEL_TOPIC, rclcpp::QoS(10), [&](geometry_msgs::msg::Twist::SharedPtr msg)
        { cmd_vel_ = *msg; });
    // timer
    main_timer_ = this->create_wall_timer(1s * BROADCAST_PERIOD, [&]()
                                          {
      sensor_msgs::msg::Imu imu;
      std_msgs::msg::ByteMultiArray light_sensor;
      std_msgs::msg::ByteMultiArray sw;
      std_msgs::msg::Bool ems,button;
      sensor_msgs::msg::BatteryState battery;
      static auto latest_time = this->get_clock()->now();
      light_sensor.data.resize(5);
      sw.data.resize(2);
      imu.header = make_header(IMU_FRAME, this->get_clock()->now());
      odom_.header = make_header(ODOM_FRAME, this->get_clock()->now());
      battery.header = make_header(BASE_FRAME, this->get_clock()->now());
      f11robo::command_msg_t command_msg;
      f11robo::sensor_msg_t sensor_msg;
      // command set
      cmd_vel_.linear.x = std::max(std::min(cmd_vel_.linear.x, MAX_VEL), -MAX_VEL);
      cmd_vel_.angular.z = std::max(std::min(cmd_vel_.angular.z, MAX_ANGULAR), -MAX_ANGULAR);
      move_->move(cmd_vel_.linear.x, 0.0, cmd_vel_.angular.z);
      std::vector<double> wheels = move_->get_wheel_speeds();
      command_msg.velocity.left_wheel.data = wheels[0];
      command_msg.velocity.right_wheel.data = wheels[1];
      // データを送信
      boost::asio::write(*serial_, boost::asio::buffer({f11robo::DATA_HEADER}));
      boost::asio::write(*serial_, boost::asio::buffer(command_msg.get_data()));
      boost::asio::write(*serial_, boost::asio::buffer({f11robo::END}));
      // データを受信
      uint8_t buf[1], data[f11robo::sensor_msg_t::size];
      size_t len = boost::asio::read(*serial_, boost::asio::buffer(buf));
      if (len != 0 && buf[0] == f11robo::DATA_HEADER) {
        len = boost::asio::read(*serial_, boost::asio::buffer(data));
        if(debug_output_)
          std::cout << "data_len: " << len << std::endl;
        for (int i = 0; i < f11robo::sensor_msg_t::size; i++)
          sensor_msg.set(i, data[i]);
        len = boost::asio::read(*serial_, boost::asio::buffer(buf));
        // data set and publish
        double w_l     = static_cast<double>(math_util::constants::RPS_TO_RADPS*sensor_msg.velocity.left_wheel.data);// w[ras/s]=2*PI*n  PI=3.14... n[rps]
        double w_r     = static_cast<double>(math_util::constants::RPS_TO_RADPS*sensor_msg.velocity.right_wheel.data);
        double rx      = f11robo::param::R * ((w_r + w_l) / 2.0);
        double ry      = f11robo::param::R * ((w_r + w_l) / 2.0);
        double angular = (f11robo::param::R / (2.0 * f11robo::param::L)) * (w_r - w_l);
        double dt = (this->get_clock()->now() - latest_time).seconds();
        latest_time = this->get_clock()->now();
        double roll,pitch,yaw;
        roll=0.0;
        pitch=0.0;
        if(use_imu_)
        {
          // theta_ += angular * dt;
          // roll=rpy_dir_[0]*sensor_msg.rpy.roll.data;
          // pitch=rpy_dir_[1]*sensor_msg.rpy.pitch.data;
          yaw=rpy_dir_[2]*sensor_msg.rpy.yaw.data;
          imu.orientation = EulerToQuaternion(roll, pitch, yaw);
        }
        else
        {
          theta_ += angular * dt;
          yaw=theta_;
          imu.orientation = EulerToQuaternion(rpy_dir_[0]*sensor_msg.rpy.roll.data, rpy_dir_[1]*sensor_msg.rpy.pitch.data, rpy_dir_[2]*sensor_msg.rpy.yaw.data);
        }
        imu.angular_velocity.x = sensor_msg.gyro.x.data;
        imu.angular_velocity.y = sensor_msg.gyro.y.data;
        imu.angular_velocity.z = sensor_msg.gyro.z.data;
        imu.linear_acceleration.x = sensor_msg.acc.x.data;
        imu.linear_acceleration.y = sensor_msg.acc.y.data;
        imu.linear_acceleration.z = sensor_msg.acc.z.data * -1.f;
        imu_pub_->publish(imu);
        odom_.pose.pose.position.x += rx * std::cos(yaw) * dt;
        odom_.pose.pose.position.y += ry * std::sin(yaw) * dt;
        odom_.pose.pose.position.z = 0.0;
        odom_.pose.pose.orientation = EulerToQuaternion(roll, pitch, yaw);
        odom_.twist.twist.linear.x = rx;
        odom_.twist.twist.linear.y = ry;
        odom_.twist.twist.angular.z = angular;
        odom_pub_->publish(odom_);
        for (int i = 0; i < 5; i++)
          light_sensor.data[i] = sensor_msg.sensor_data.light[i];
        light_sensor_pub_->publish(light_sensor);
        for (int i = 0; i < 2; i++)
          sw.data[i] = sensor_msg.sensor_data.sw[i];
        sw_pub_->publish(sw);
        ems.data = sensor_msg.ems;
        ems_pub_->publish(ems);
        battery.voltage = sensor_msg.battery_voltage.data;
        battery_pub_->publish(battery);
        button.data = sensor_msg.button;
        if(reset_odom_func_&&button.data)
        {
          odom_.pose.pose.position.x = 0.0;
          odom_.pose.pose.position.y = 0.0;
          theta_ = 0.0;
          RCLCPP_INFO(this->get_logger(), "Reset odometry");
        }
        button_pub_->publish(button);
        // status
        std_msgs::msg::String ems_status;
        ems_status.data = "Hardware ems OFF";
        if(ems.data)
          ems_status.data = "Hardware ems ON";
        ems_status_pub_->publish(ems_status);
        std_msgs::msg::Float32 buttery_status;
        buttery_status.data = battery.voltage;
        buttery_status_pub_->publish(buttery_status);
        // debug
        if(debug_output_)
        {
          std::cout << "left_wheel:" << wheels[0] << " right_wheel:" << wheels[1] << std::endl;
          std::cout << "rx: " << rx << " ry:" << ry << std::endl;
          std::cout << "angular:" << angular << std::endl;
          std::cout << "sensor_msg.velocity.right_wheel: " << sensor_msg.velocity.right_wheel.data << std::endl;
          std::cout << "sensor_msg.velocity.left_wheel: " << sensor_msg.velocity.left_wheel.data << std::endl;
          std::cout << "sensor_msg.rpy.roll: " << sensor_msg.rpy.roll.data << std::endl;
          std::cout << "sensor_msg.rpy.pitch: " << sensor_msg.rpy.pitch.data << std::endl;
          std::cout << "sensor_msg.rpy.yaw: " << sensor_msg.rpy.yaw.data << std::endl;
          std::cout << "sensor_msg.acc.x: " << sensor_msg.acc.x.data << std::endl;
          std::cout << "sensor_msg.acc.y: " << sensor_msg.acc.y.data << std::endl;
          std::cout << "sensor_msg.acc.z: " << sensor_msg.acc.z.data << std::endl;
          std::cout << "sensor_msg.gyro.x: " << sensor_msg.gyro.x.data << std::endl;
          std::cout << "sensor_msg.gyro.y: " << sensor_msg.gyro.y.data << std::endl;
          std::cout << "sensor_msg.gyro.z: " << sensor_msg.gyro.z.data << std::endl;
          std::cout << "sensor_msg.sensor_data.light:";
          for (int i = 0; i < 5; i++)
            std::cout << " " << (int)sensor_msg.sensor_data.light[i];
          std::cout << std::endl;
          for (int i = 0; i < 2; i++)
            std::cout << "sensor_msg.sensor_data.sw[" << i << "]: " << sensor_msg.sensor_data.sw[i] << std::endl;
          std::cout << "sensor_msg.ems: " << sensor_msg.ems << std::endl;
          std::cout << "sensor_msg.battery_voltage: " << sensor_msg.battery_voltage.data << std::endl;
          std::cout << "sensor_msg.button: " << sensor_msg.button << std::endl;
          std::cout << "end: " << (buf[0] == f11robo::END) << std::endl << std::endl;
        }
      }
      if(tf_output_)
      {
        // tf
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header         = make_header(ODOM_FRAME, this->get_clock()->now());
        transform_stamped.child_frame_id = BASE_FRAME;
        transform_stamped.transform.translation.x = odom_.pose.pose.position.x;
        transform_stamped.transform.translation.y = odom_.pose.pose.position.y;
        transform_stamped.transform.translation.z = odom_.pose.pose.position.z;
        transform_stamped.transform.rotation.x    = odom_.pose.pose.orientation.x;
        transform_stamped.transform.rotation.y    = odom_.pose.pose.orientation.y;
        transform_stamped.transform.rotation.z    = odom_.pose.pose.orientation.z;
        transform_stamped.transform.rotation.w    = odom_.pose.pose.orientation.w;
        broadcaster_.sendTransform(transform_stamped);
      } });
    // param送信
    f11robo::param_msg_t param_msg;
    RCLCPP_INFO(this->get_logger(), "Parameter Sending!");
    while (1)
    {
      param_msg.left_md_pid_gain.kp.data = l_pid_gain[0];
      param_msg.left_md_pid_gain.ki.data = l_pid_gain[1];
      param_msg.left_md_pid_gain.kd.data = l_pid_gain[2];
      param_msg.right_md_pid_gain.kp.data = r_pid_gain[0];
      param_msg.right_md_pid_gain.ki.data = r_pid_gain[1];
      param_msg.right_md_pid_gain.kd.data = r_pid_gain[2];
      boost::asio::write(*serial_, boost::asio::buffer({f11robo::PARAM_HEADER}));
      boost::asio::write(*serial_, boost::asio::buffer(param_msg.get_data()));
      boost::asio::write(*serial_, boost::asio::buffer({f11robo::END}));
      uint8_t buf[1];
      size_t len = boost::asio::read(*serial_, boost::asio::buffer(buf));
      if (len != 0 && buf[0] == f11robo::PARAM_HEADER)
      {
        len = boost::asio::read(*serial_, boost::asio::buffer(buf));
        if (buf[0] == f11robo::END)
          break;
      }
      rclcpp::sleep_for(50ms);
    }
    RCLCPP_INFO(this->get_logger(), "Initialization complete!");
  }

private:
  bool tf_output_;
  bool debug_output_;
  bool use_imu_;
  bool reset_odom_func_;
  double MAX_VEL;
  double MAX_ANGULAR;
  std::string CMD_VEL_TOPIC;
  std::string ODOM_FRAME;
  std::string BASE_FRAME;
  std::string IMU_FRAME;
  std::vector<float> rpy_dir_;
  // serial
  boost::asio::io_service io;
  std::shared_ptr<boost::asio::serial_port> serial_;
  // tf
  tf2_ros::TransformBroadcaster broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;
  // timer
  rclcpp::TimerBase::SharedPtr main_timer_;
  // publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr light_sensor_pub_;
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr sw_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ems_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ems_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr buttery_status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  // subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  // odometry
  double theta_;
  nav_msgs::msg::Odometry odom_;
  // twist
  geometry_msgs::msg::Twist cmd_vel_;

  std::shared_ptr<kinematics::MoveBased> move_;

  template <class T>
  T param(const std::string &name, const T &def)
  {
    T value;
    declare_parameter(name, def);
    get_parameter(name, value);
    return value;
  }

  inline std_msgs::msg::Header make_header(const std::string &frame_name, const rclcpp::Time &t)
  {
    std_msgs::msg::Header header;
    header.frame_id = frame_name;
    header.stamp = t;
    return header;
  }

  // オイラー角とクオータニオン変換
  inline geometry_msgs::msg::Quaternion EulerToQuaternion(double roll, double pitch, double yaw)
  {
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(roll, pitch, yaw);
    return tf2::toMsg(tf_quat);
  }

  geometry_msgs::msg::Twist stop()
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    return twist;
  }
};