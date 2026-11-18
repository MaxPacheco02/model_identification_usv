// Open-loop identification experiments for a differential-thrust USV.
//
// Runs a fixed program of test segments (surge, yaw, sway, turning circles, zig-zags, multisine),
// publishes the thrust of each side and, for the sway tests, an external body-frame force (in
// Gazebo through the BodyWrench system of this package; on the water, a rotated thruster). The
// segments are driven by the odometry time stamps, so the program runs the same at any real-time
// factor. Every odometry message is written to <output_dir>/<run_name>.csv with the inputs active
// at that moment; one line per segment goes to <run_name>_segments.csv.
//
// Velocities are taken from the odometry twist, expected in the body frame (x forward, y left,
// z up), as published by gz-sim's OdometryPublisher.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <geometry_msgs/msg/wrench.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "model_identification_usv/model3dof.hpp"

namespace
{

double wrap(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

enum class Mode
{
  Constant,  // inputs fixed
  Zigzag,    // common thrust, differential flipped at +-angle from the initial heading
  Multisine  // common and differential thrust as two random-phase multisines
};

struct Segment
{
  std::string block;
  int step{0};
  std::string phase;      // settle, accel, hold, release, run
  Mode mode{Mode::Constant};
  double t_left{0}, t_right{0};  // thrust per side [N]
  double fx{0}, fy{0};    // external body-frame force [N] (sway tests)
  bool heading_hold{false};  // add a differential thrust that holds the initial heading
  bool until_steady{false};  // end when (u, v, r) are steady, after t_min
  double t_min{0}, t_max{0};
  double zz_diff{0}, zz_angle{0};  // zig-zag
};

}  // namespace

class ExperimentNode : public rclcpp::Node
{
public:
  ExperimentNode()
  : Node("id_experiment")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/gz_sim/odometry");
    const auto left_topic = declare_parameter<std::string>(
      "left_thrust_topic", "/model/usv/joint/left_engine_propeller_joint/cmd_thrust");
    const auto right_topic = declare_parameter<std::string>(
      "right_thrust_topic", "/model/usv/joint/right_engine_propeller_joint/cmd_thrust");
    const auto wrench_topic = declare_parameter<std::string>("wrench_topic", "/id/body_wrench");
    output_dir_ = declare_parameter<std::string>("output_dir", "/tmp/model_identification_usv");
    run_name_ = declare_parameter<std::string>("run_name", "run");
    blocks_ = declare_parameter<std::vector<std::string>>(
      "blocks", {"surge", "yaw", "sway", "turn", "zigzag", "multisine"});

    arm_ = declare_parameter<double>("thruster_arm", 0.28);
    t_lo_ = declare_parameter<double>("thrust_min", -60.0);
    t_hi_ = declare_parameter<double>("thrust_max", 73.0);
    px_ = declare_parameter<double>("force_point_x", 0.0);
    py_ = declare_parameter<double>("force_point_y", 0.0);

    // steady-state detection: every channel within its band over the window
    steady_window_ = declare_parameter<double>("steady_window", 5.0);
    steady_tol_ = declare_parameter<std::vector<double>>("steady_tol", {0.004, 0.002, 0.004});
    heading_kp_ = declare_parameter<double>("heading_kp", 20.0);
    heading_kd_ = declare_parameter<double>("heading_kd", 20.0);
    heading_max_ = declare_parameter<double>("heading_max_diff", 50.0);

    settle_time_ = declare_parameter<double>("settle_time", 10.0);
    hold_min_ = declare_parameter<double>("hold_min", 15.0);
    hold_max_ = declare_parameter<double>("hold_max", 90.0);
    release_time_ = declare_parameter<double>("release_time", 40.0);

    surge_levels_ = declare_parameter<std::vector<double>>(
      "surge_levels", {0.5, 1.0, 2.0, 4.0, 7.0, 10.0, 15.0, 20.0, 25.0, 30.0, -0.5, -1.0, -2.0,
        -4.0, -8.0});
    yaw_levels_ = declare_parameter<std::vector<double>>(
      "yaw_levels", {0.25, 0.5, 1.0, 2.0, 4.0, 8.0, -0.25, -0.5, -1.0, -2.0, -4.0, -8.0});
    sway_forces_ = declare_parameter<std::vector<double>>(
      "sway_forces", {0.5, 1.0, 2.0, 4.0, 8.0, 15.0, -0.5, -1.0, -2.0, -4.0, -8.0, -15.0});
    turn_thrust_ = declare_parameter<std::vector<double>>("turn_thrust", {5.0, 10.0, 20.0});
    turn_diff_ = declare_parameter<std::vector<double>>("turn_diff", {1.0, 2.0, 4.0});
    zigzag_thrust_ = declare_parameter<double>("zigzag_thrust", 10.0);
    zigzag_diff_ = declare_parameter<double>("zigzag_diff", 4.0);
    zigzag_angles_ = declare_parameter<std::vector<double>>("zigzag_angles_deg", {10.0, 20.0});
    zigzag_time_ = declare_parameter<double>("zigzag_time", 120.0);
    ms_common_mean_ = declare_parameter<double>("multisine_common_mean", 8.0);
    ms_common_amp_ = declare_parameter<double>("multisine_common_amp", 6.0);
    ms_diff_amp_ = declare_parameter<double>("multisine_diff_amp", 3.0);
    ms_fmin_ = declare_parameter<double>("multisine_fmin", 0.01);
    ms_fmax_ = declare_parameter<double>("multisine_fmax", 0.5);
    ms_n_ = declare_parameter<int>("multisine_n", 12);
    ms_time_ = declare_parameter<double>("multisine_time", 300.0);
    ms_seed_ = declare_parameter<int>("multisine_seed", 1);

    pub_left_ = create_publisher<std_msgs::msg::Float64>(left_topic, 10);
    pub_right_ = create_publisher<std_msgs::msg::Float64>(right_topic, 10);
    if (!wrench_topic.empty()) {
      pub_wrench_ = create_publisher<geometry_msgs::msg::Wrench>(wrench_topic, 10);
    }

    build_program();
    open_logs();
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {on_odom(*msg);});
    RCLCPP_INFO(
      get_logger(), "%zu segments, waiting for odometry on %s; log: %s", program_.size(),
      odom_topic_.c_str(), log_path_.c_str());
  }

  ~ExperimentNode() override
  {
    publish(0.0, 0.0, 0.0, 0.0);
  }

private:
  void build_program()
  {
    auto add = [this](Segment s) {program_.push_back(s);};
    Segment settle;
    settle.block = "start";
    settle.phase = "settle";
    settle.t_min = settle.t_max = settle_time_;
    add(settle);

    auto release = [&](const std::string & b, int k, bool hold) {
        Segment s;
        s.block = b;
        s.step = k;
        s.phase = "release";
        s.heading_hold = hold;
        s.t_min = s.t_max = release_time_;
        add(s);
      };
    auto hold = [&](const std::string & b, int k) {
        Segment s;
        s.block = b;
        s.step = k;
        s.phase = "hold";
        s.until_steady = true;
        s.t_min = hold_min_;
        s.t_max = hold_max_;
        return s;
      };

    for (const auto & b : blocks_) {
      if (b == "surge") {
        // equal thrust on both sides: pure surge force
        for (size_t k = 0; k < surge_levels_.size(); ++k) {
          Segment s = hold(b, k);
          s.t_left = s.t_right = surge_levels_[k];
          add(s);
          release(b, k, false);
        }
      } else if (b == "yaw") {
        // opposite thrust: pure yaw moment arm*(t_right - t_left) = 2*arm*level
        for (size_t k = 0; k < yaw_levels_.size(); ++k) {
          Segment s = hold(b, k);
          s.t_left = -yaw_levels_[k];
          s.t_right = yaw_levels_[k];
          add(s);
          release(b, k, false);
        }
      } else if (b == "sway") {
        // external lateral force (rotated thruster), heading held with differential thrust
        if (!pub_wrench_) {
          RCLCPP_WARN(get_logger(), "no wrench_topic: sway block skipped");
          continue;
        }
        for (size_t k = 0; k < sway_forces_.size(); ++k) {
          Segment s = hold(b, k);
          s.fy = sway_forces_[k];
          s.heading_hold = true;
          add(s);
          release(b, k, true);
        }
      } else if (b == "turn") {
        // turning circles: straight run to steady speed, constant differential until steady,
        // release
        int k = 0;
        for (double t0 : turn_thrust_) {
          for (double dt : turn_diff_) {
            Segment a = hold(b, k);
            a.phase = "accel";
            a.t_left = a.t_right = t0;
            a.heading_hold = true;
            add(a);
            Segment s = hold(b, k);
            s.t_left = t0 - dt;
            s.t_right = t0 + dt;
            add(s);
            release(b, k, false);
            ++k;
          }
        }
      } else if (b == "zigzag") {
        int k = 0;
        for (double ang : zigzag_angles_) {
          Segment a = hold(b, k);
          a.phase = "accel";
          a.t_left = a.t_right = zigzag_thrust_;
          a.heading_hold = true;
          add(a);
          Segment s;
          s.block = b;
          s.step = k;
          s.phase = "run";
          s.mode = Mode::Zigzag;
          s.t_left = s.t_right = zigzag_thrust_;
          s.zz_diff = zigzag_diff_;
          s.zz_angle = ang * M_PI / 180.0;
          s.t_min = s.t_max = zigzag_time_;
          add(s);
          release(b, k, false);
          ++k;
        }
      } else if (b == "multisine") {
        Segment s;
        s.block = b;
        s.phase = "run";
        s.mode = Mode::Multisine;
        s.t_min = s.t_max = ms_time_;
        add(s);
        release(b, 0, false);
      } else {
        RCLCPP_WARN(get_logger(), "unknown block '%s' skipped", b.c_str());
      }
    }

    // multisine: log-spaced frequencies, interleaved between the common and the differential
    // channel so that the two inputs are uncorrelated; random phases (Schroeder would also do)
    std::mt19937 gen(ms_seed_);
    std::uniform_real_distribution<double> ph(0.0, 2 * M_PI);
    const int n = std::max(2, 2 * static_cast<int>(ms_n_));
    for (int i = 0; i < n; ++i) {
      const double f = ms_fmin_ * std::pow(ms_fmax_ / ms_fmin_, static_cast<double>(i) / (n - 1));
      (i % 2 == 0 ? ms_common_ : ms_diff_).push_back({f, ph(gen)});
    }
  }

  void open_logs()
  {
    std::filesystem::create_directories(output_dir_);
    log_path_ = output_dir_ + "/" + run_name_ + ".csv";
    log_.open(log_path_);
    log_ << "t,block,step,phase,x,y,psi,u,v,r,t_left,t_right,fx,fy,tau_x,tau_y,tau_n\n";
    seg_log_.open(output_dir_ + "/" + run_name_ + "_segments.csv");
    seg_log_ << "block,step,phase,t_start,t_end,steady,t_left,t_right,fy\n";
  }

  void publish(double tl, double tr, double fx, double fy)
  {
    std_msgs::msg::Float64 m;
    m.data = tl;
    pub_left_->publish(m);
    m.data = tr;
    pub_right_->publish(m);
    if (pub_wrench_) {
      geometry_msgs::msg::Wrench w;
      w.force.x = fx;
      w.force.y = fy;
      w.torque.z = px_ * fy - py_ * fx;  // force applied at (px, py), wrench about the origin
      pub_wrench_->publish(w);
    }
  }

  // steady: the means of the two halves of the window differ by less than the tolerance in every
  // channel (robust to the noise of the mesh-based hydrodynamics, unlike a min-max band)
  bool steady() const
  {
    if (window_.empty() || window_.back()[0] - window_.front()[0] < steady_window_ - 0.05) {
      return false;
    }
    const double tm = 0.5 * (window_.front()[0] + window_.back()[0]);
    for (int c = 0; c < 3; ++c) {
      double s1 = 0, s2 = 0;
      int n1 = 0, n2 = 0;
      for (const auto & w : window_) {
        if (w[0] < tm) {
          s1 += w[c + 1];
          ++n1;
        } else {
          s2 += w[c + 1];
          ++n2;
        }
      }
      if (n1 == 0 || n2 == 0 || std::abs(s2 / n2 - s1 / n1) > steady_tol_[c]) {
        return false;
      }
    }
    return true;
  }

  void on_odom(const nav_msgs::msg::Odometry & msg)
  {
    if (idx_ >= program_.size()) {
      return;
    }
    const double t = rclcpp::Time(msg.header.stamp).seconds();
    const auto & q = msg.pose.pose.orientation;
    const double psi = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    const double u = msg.twist.twist.linear.x, v = msg.twist.twist.linear.y;
    const double r = msg.twist.twist.angular.z;

    Segment & s = program_[idx_];
    if (!started_) {
      started_ = true;
      t0_ = t;
      psi_ref_ = psi;
      zz_dir_ = 1;
      window_.clear();
      RCLCPP_INFO(
        get_logger(), "[%zu/%zu] %s %d %s", idx_ + 1, program_.size(), s.block.c_str(), s.step,
        s.phase.c_str());
    }
    const double ts = t - t0_;

    // inputs of this segment at this instant
    double tl = s.t_left, tr = s.t_right;
    if (s.mode == Mode::Zigzag) {
      const double e = wrap(psi - psi_ref_);
      if (zz_dir_ > 0 && e >= s.zz_angle) {zz_dir_ = -1;}
      if (zz_dir_ < 0 && e <= -s.zz_angle) {zz_dir_ = 1;}
      tl -= zz_dir_ * s.zz_diff;
      tr += zz_dir_ * s.zz_diff;
    } else if (s.mode == Mode::Multisine) {
      double c = ms_common_mean_, d = 0;
      for (const auto & [f, p] : ms_common_) {
        c += ms_common_amp_ / std::sqrt(ms_common_.size()) * std::sin(2 * M_PI * f * ts + p);
      }
      for (const auto & [f, p] : ms_diff_) {
        d += ms_diff_amp_ / std::sqrt(ms_diff_.size()) * std::sin(2 * M_PI * f * ts + p);
      }
      tl = c - d;
      tr = c + d;
    }
    if (s.heading_hold) {
      const double d = std::clamp(
        heading_kp_ * wrap(psi_ref_ - psi) - heading_kd_ * r, -heading_max_, heading_max_);
      tl -= d;
      tr += d;
    }
    tl = std::clamp(tl, t_lo_, t_hi_);
    tr = std::clamp(tr, t_lo_, t_hi_);
    publish(tl, tr, s.fx, s.fy);

    const auto tau = model_identification_usv::generalized_force(
      tl, tr, arm_, s.fx, s.fy, 0.0, px_, py_);
    log_ << std::fixed;
    log_.precision(4);
    log_ << t << ',' << s.block << ',' << s.step << ',' << s.phase << ',';
    log_.precision(6);
    log_ << msg.pose.pose.position.x << ',' << msg.pose.pose.position.y << ',' << psi << ',' << u <<
      ',' << v << ',' << r << ',' << tl << ',' << tr << ',' << s.fx << ',' << s.fy << ',' <<
      tau[0] << ',' << tau[1] << ',' << tau[2] << '\n';

    // steady-state window without the yaw-rate spikes of gz's OdometryPublisher at psi = +-pi
    // (the log keeps the raw values)
    if (std::abs(r) < 5.0 || window_.empty()) {
      window_.push_back({t, u, v, std::abs(r) < 5.0 ? r : 0.0});
    }
    while (!window_.empty() && t - window_.front()[0] > steady_window_) {
      window_.pop_front();
    }

    const bool is_steady = s.until_steady && ts >= s.t_min && steady();
    if (is_steady || ts >= s.t_max) {
      seg_log_ << s.block << ',' << s.step << ',' << s.phase << ',' << t0_ << ',' << t << ',' <<
        (is_steady ? 1 : 0) << ',' << s.t_left << ',' << s.t_right << ',' << s.fy << '\n';
      seg_log_.flush();
      if (s.until_steady && !is_steady) {
        RCLCPP_WARN(get_logger(), "  not steady after %.0f s", ts);
      }
      ++idx_;
      started_ = false;
      if (idx_ >= program_.size()) {
        publish(0.0, 0.0, 0.0, 0.0);
        log_.flush();
        RCLCPP_INFO(get_logger(), "program finished: %s", log_path_.c_str());
        rclcpp::shutdown();
      }
    }
  }

  // parameters
  std::string odom_topic_, output_dir_, run_name_, log_path_;
  std::vector<std::string> blocks_;
  double arm_, t_lo_, t_hi_, px_, py_;
  double steady_window_;
  std::vector<double> steady_tol_;
  double heading_kp_, heading_kd_, heading_max_;
  double settle_time_, hold_min_, hold_max_, release_time_;
  std::vector<double> surge_levels_, yaw_levels_, sway_forces_, turn_thrust_, turn_diff_;
  double zigzag_thrust_, zigzag_diff_, zigzag_time_;
  std::vector<double> zigzag_angles_;
  double ms_common_mean_, ms_common_amp_, ms_diff_amp_, ms_fmin_, ms_fmax_, ms_time_;
  int64_t ms_n_, ms_seed_;
  std::vector<std::pair<double, double>> ms_common_, ms_diff_;

  // state
  std::vector<Segment> program_;
  size_t idx_{0};
  bool started_{false};
  double t0_{0}, psi_ref_{0};
  int zz_dir_{1};
  std::deque<std::array<double, 4>> window_;
  std::ofstream log_, seg_log_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_, pub_right_;
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_wrench_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExperimentNode>();
  rclcpp::spin(node);
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
