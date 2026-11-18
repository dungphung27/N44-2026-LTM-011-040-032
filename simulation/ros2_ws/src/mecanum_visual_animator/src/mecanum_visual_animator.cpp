\
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <functional>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo {
class MecanumVisualAnimator : public ModelPlugin {
 private:
  struct Wheel {
    std::string link, visual, joint;
    ignition::math::Pose3d original;
    ignition::math::Vector3d pivot, axis;
    double sign{1}, angle{0};
  };
  physics::ModelPtr model_;
  transport::NodePtr gz_node_;
  transport::PublisherPtr visual_pub_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  event::ConnectionPtr update_conn_;
  std::vector<Wheel> wheels_;
  common::Time previous_;
  std::chrono::steady_clock::time_point last_cmd_;
  std::mutex lock_;
  double vx_{0}, vy_{0}, wz_{0};
  double radius_{0.048}, L_{0.328}, max_speed_{3.0};

  static ignition::math::Vector3d ReadVec(const std::string &s) {
    ignition::math::Vector3d out;
    std::istringstream in(s);
    double a,b,c;
    if (!(in >> a >> b >> c)) throw std::runtime_error("bad vec3: " + s);
    out.Set(a,b,c);
    return out;
  }
  static ignition::math::Pose3d ReadPose(const std::string &s) {
    std::istringstream in(s);
    double x,y,z,r,p,w;
    if (!(in >> x >> y >> z >> r >> p >> w))
      throw std::runtime_error("bad pose: " + s);
    return ignition::math::Pose3d(x,y,z,r,p,w);
  }
 public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    radius_ = sdf->Get<double>("wheel_radius");
    L_ = sdf->Get<double>("half_length") + sdf->Get<double>("half_width");
    max_speed_ = sdf->Get<double>("max_wheel_rad_s");
    if (!sdf->HasElement("wheel")) {
      gzerr << "No visual wheels configured\n";
      return;
    }
    auto element = sdf->GetElement("wheel");
    while (element) {
      Wheel v;
      v.joint = element->Get<std::string>("joint");
      v.link = element->Get<std::string>("link");
      v.visual = element->Get<std::string>("visual");
      v.original = ReadPose(element->Get<std::string>("pose"));
      v.pivot = ReadVec(element->Get<std::string>("pivot"));
      v.axis = ReadVec(element->Get<std::string>("axis")).Normalize();
      v.sign = element->Get<double>("sign");
      wheels_.push_back(v);
      element = element->GetNextElement("wheel");
    }
    if (wheels_.size() != 4) {
      gzerr << "Need exactly 4 visual wheels, got " << wheels_.size() << "\n";
      wheels_.clear();
      return;
    }
    gz_node_.reset(new transport::Node());
    gz_node_->Init(model_->GetWorld()->Name());
    visual_pub_ = gz_node_->Advertise<msgs::Visual>("~/visual", 50);
    ros_node_ = gazebo_ros::Node::Get(sdf);
    joint_state_pub_ = ros_node_->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(10));
    std::string topic = sdf->Get<std::string>("topic");
    cmd_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      topic, rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) ||
            !std::isfinite(msg->angular.z)) return;
        std::lock_guard<std::mutex> guard(lock_);
        vx_ = std::clamp(msg->linear.x, -0.08, 0.08);
        vy_ = std::clamp(msg->linear.y, -0.06, 0.06);
        wz_ = std::clamp(msg->angular.z, -0.2, 0.2);
        last_cmd_ = std::chrono::steady_clock::now();
      });
    last_cmd_ = std::chrono::steady_clock::now();
    previous_ = model_->GetWorld()->SimTime();
    update_conn_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&MecanumVisualAnimator::OnUpdate, this, std::placeholders::_1));
    gzmsg << "Mecanum wheel mesh-only animation loaded (no dynamic wheel joints).\n";
  }
  void OnUpdate(const common::UpdateInfo &info) {
    if (!visual_pub_ || wheels_.size() != 4) return;
    const double dt = (info.simTime - previous_).Double();
    previous_ = info.simTime;
    if (dt <= 0 || dt > 0.2) return;
    double vx,vy,wz;
    {
      std::lock_guard<std::mutex> guard(lock_);
      bool fresh = std::chrono::duration<double>(
          std::chrono::steady_clock::now()-last_cmd_).count() < 0.35;
      vx = fresh ? vx_ : 0.;
      vy = fresh ? vy_ : 0.;
      wz = fresh ? wz_ : 0.;
    }
    // Order Joint002 (FL), Joint003 (RL), Joint006 (FR), Joint005 (RR).
    const double values[4] = {(vx-vy-L_*wz)/radius_,
                              (vx+vy-L_*wz)/radius_,
                              (vx+vy+L_*wz)/radius_,
                              (vx-vy+L_*wz)/radius_};
    static unsigned frame = 0;
    const bool publish = (++frame % 25 == 0); // 40 Hz with 1000 Hz physics.
    sensor_msgs::msg::JointState wheel_states;
    if (publish) {
      // Stamp using Gazebo simulation time, not host wall time.
      wheel_states.header.stamp.sec = static_cast<int32_t>(info.simTime.sec);
      wheel_states.header.stamp.nanosec = static_cast<uint32_t>(info.simTime.nsec);
    }
    for (size_t i=0; i<4; ++i) {
      auto &w = wheels_[i];
      const double rate = std::clamp(w.sign*values[i], -max_speed_, max_speed_);
      w.angle = std::remainder(w.angle + rate*dt, 2*M_PI);
      if (!publish) continue;
      const ignition::math::Quaterniond spin(w.axis, w.angle);
      const ignition::math::Pose3d p(
          w.pivot + spin.RotateVector(w.original.Pos() - w.pivot),
          spin*w.original.Rot());
      msgs::Visual msg;
      msg.set_name(model_->GetName() + "::" + w.link + "::" + w.visual);
      msg.set_parent_name(model_->GetName() + "::" + w.link);
      msg.set_type(msgs::Visual::VISUAL);
      msgs::Set(msg.mutable_pose(), p);
      visual_pub_->Publish(msg);
      // Exactly the angle/rate currently used for this Gazebo mesh.
      wheel_states.name.push_back(w.joint);
      wheel_states.position.push_back(w.angle);
      wheel_states.velocity.push_back(rate);
    }
    if (publish && joint_state_pub_ && wheel_states.name.size() == wheels_.size()) {
      joint_state_pub_->publish(wheel_states);
    }
  }
};
GZ_REGISTER_MODEL_PLUGIN(MecanumVisualAnimator)
}  // namespace gazebo
