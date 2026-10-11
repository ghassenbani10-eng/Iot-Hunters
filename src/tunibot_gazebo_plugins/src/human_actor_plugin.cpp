// HumanActorPlugin — a walking person for the TuniBot restaurant simulation.
//
// Gazebo Classic actors are animated skins without collision geometry, so a ray
// (lidar) sensor cannot see them. This plugin
//   1. walks the actor along a list of waypoints at a given speed, with the
//      walking animation synchronised to the distance travelled, and
//   2. moves a separate collision "body" model (a cylinder the size of a person)
//      to the actor's position in the same physics update, so the robot's lidar
//      and bumper perceive the person exactly where it is drawn.
//
// Behaviour of the simulated person (deliberately simple and reproducible):
//   - walks waypoint to waypoint, turning smoothly, pausing `dwell` s at each one;
//   - if the robot is right in front of it (closer than `yield_distance`, inside a
//     +-50 deg cone), it stops and waits like a polite pedestrian;
//   - if it has been waiting more than `max_yield_time` s, it walks around the
//     robot: a 1 m sidestep away from it, then on to its destination, ignoring
//     the robot for 5 s meanwhile (two polite agents waiting for each other would
//     deadlock; turning back does not work at the end of a path);
//   - whatever it does, it never steps closer than `min_robot_distance` to the
//     robot centre (it stops instead): people do not walk into a robot.
//
// SDF parameters (all optional except <waypoint>):
//   <waypoint>x y [dwell]</waypoint> (repeat, world frame, metres; optional wait there, s)
//   <speed>0.9</speed>              walking speed, m/s
//   <dwell>1.0</dwell>              pause at each waypoint, s
//   <start_delay>0</start_delay>    s of simulation time before starting to walk
//   <body>human_1</body>            name of the collision proxy model
//   <robot>waiter_robot</robot>     model to yield to
//   <yield_distance>0.9</yield_distance>
//   <max_yield_time>6.0</max_yield_time>
//   <min_robot_distance>0.62</min_robot_distance>
//   <animation_factor>5.1</animation_factor>
//   <ping_pong>true</ping_pong>     true: A-B-C-B-A..., false: A-B-C-A-B...
//   <ros><namespace>/human_x</namespace></ros>
//                                   optional: subscribe to <namespace>/cmd
//                                   (std_msgs/Float64MultiArray) to give the person a new
//                                   walk at runtime (tests, demos):
//                                   [speed, dwell, ping_pong(0/1), start_delay, x0, y0, x1, y1, ...]
//                                   the person is moved to (x0, y0) and walks from there.

#include <algorithm>
#include <cmath>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace tunibot
{

class HumanActorPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    actor_ = boost::dynamic_pointer_cast<gazebo::physics::Actor>(model);
    if (!actor_) {
      gzerr << "HumanActorPlugin must be attached to an <actor>\n";
      return;
    }
    world_ = actor_->GetWorld();

    if (sdf->HasElement("waypoint")) {
      // "x y" or "x y dwell": an optional per-waypoint waiting time (e.g. a queue, a toilet)
      for (auto e = sdf->GetElement("waypoint"); e; e = e->GetNextElement("waypoint")) {
        std::istringstream in(e->Get<std::string>());
        double x = 0.0, y = 0.0, d = -1.0;
        in >> x >> y;
        if (!(in >> d)) {
          d = -1.0;
        }
        waypoints_.emplace_back(x, y);
        dwells_.push_back(d);
      }
    }
    if (waypoints_.size() < 2) {
      gzerr << "HumanActorPlugin[" << actor_->GetName() << "]: need >= 2 waypoints\n";
      return;
    }
    speed_ = sdf->Get<double>("speed", 0.9).first;
    dwell_ = sdf->Get<double>("dwell", 1.0).first;
    start_delay_ = sdf->Get<double>("start_delay", 0.0).first;
    body_name_ = sdf->Get<std::string>("body", "").first;
    robot_name_ = sdf->Get<std::string>("robot", "waiter_robot").first;
    yield_distance_ = sdf->Get<double>("yield_distance", 0.9).first;
    max_yield_time_ = sdf->Get<double>("max_yield_time", 6.0).first;
    min_robot_distance_ = sdf->Get<double>("min_robot_distance", 0.62).first;
    animation_factor_ = sdf->Get<double>("animation_factor", 5.1).first;
    ping_pong_ = sdf->Get<bool>("ping_pong", true).first;

    pos_ = waypoints_[0];
    const auto d = waypoints_[1] - waypoints_[0];
    yaw_ = std::atan2(d.Y(), d.X());
    target_ = 1;

    if (sdf->HasElement("ros")) {
      ros_node_ = gazebo_ros::Node::Get(sdf);
      cmd_sub_ = ros_node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "cmd", 10, [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(cmd_mutex_);
          pending_cmd_ = msg->data;
          has_pending_cmd_ = true;
        });
    }

    // Play the skin's "walking" animation under our own (custom) trajectory.
    auto traj = std::make_shared<gazebo::physics::TrajectoryInfo>();
    traj->type = "walking";
    traj->duration = 1.0;
    actor_->SetCustomTrajectory(traj);

    update_conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&HumanActorPlugin::OnUpdate, this, std::placeholders::_1));
    Apply();
  }

private:
  void ApplyCommand(const std::vector<double> & c, double now)
  {
    if (c.size() < 8) {
      gzerr << "HumanActorPlugin[" << actor_->GetName() << "]: cmd needs >= 2 waypoints\n";
      return;
    }
    speed_ = c[0];
    dwell_ = c[1];
    ping_pong_ = c[2] > 0.5;
    waypoints_.clear();
    dwells_.clear();
    for (size_t i = 4; i + 1 < c.size(); i += 2) {
      waypoints_.emplace_back(c[i], c[i + 1]);
      dwells_.push_back(-1.0);
    }
    pos_ = waypoints_[0];
    const auto d = waypoints_[1] - waypoints_[0];
    yaw_ = std::atan2(d.Y(), d.X());
    target_ = 1;
    previous_ = 0;
    direction_ = 1;
    wait_until_ = now + c[3];
    detour_ = false;
    yield_start_ = -1.0;
    ignore_robot_until_ = 0.0;
  }

  void OnUpdate(const gazebo::common::UpdateInfo & info)
  {
    const double now = info.simTime.Double();
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      if (has_pending_cmd_) {
        ApplyCommand(pending_cmd_, now);
        has_pending_cmd_ = false;
      }
    }
    double dt = now - last_time_;
    last_time_ = now;
    if (dt <= 0.0 || dt > 0.5) {
      return;
    }
    if (now < start_delay_ || now < wait_until_) {
      Apply();
      return;
    }

    // Yield to the robot when it is right in front of us (stop walking, keep turning).
    // Hysteresis: the patience timer only resets once the robot has been out of the
    // way for 1 s (a robot hovering at the edge of the cone would otherwise reset it
    // forever and the two would stand in front of each other).
    bool yielding = false;
    const bool ahead = now >= ignore_robot_until_ && RobotAhead();
    if (ahead) {
      last_ahead_ = now;
    }
    if (ahead || (yield_start_ >= 0.0 && now - last_ahead_ < 1.0)) {
      if (yield_start_ < 0.0) {
        yield_start_ = now;
      }
      if (now - yield_start_ < max_yield_time_) {
        yielding = true;
      } else {
        // Waited long enough: walk around the robot - a 1 m sidestep away from it,
        // then on to the destination - without re-checking the robot meanwhile.
        StartDetour();
        ignore_robot_until_ = now + 6.0;
        yield_start_ = -1.0;
      }
    } else {
      yield_start_ = -1.0;
    }

    const auto goal = detour_ ? detour_point_ : waypoints_[target_];
    auto to_goal = goal - pos_;
    const double dist = to_goal.Length();
    if (dist < 0.05) {
      if (detour_) {
        detour_ = false;          // sidestep done, carry on to the destination
      } else {
        const double d = (target_ < static_cast<int>(dwells_.size()) && dwells_[target_] >= 0.0) ?
          dwells_[target_] : dwell_;
        AdvanceTarget();
        wait_until_ = now + d;
      }
      Apply();
      return;
    }

    // Turn towards the goal at a human-like rate, walk when roughly aligned.
    const double desired = std::atan2(to_goal.Y(), to_goal.X());
    const double err = std::remainder(desired - yaw_, 2.0 * M_PI);
    const double max_turn = 2.5 * dt;
    yaw_ += std::clamp(err, -max_turn, max_turn);
    double step = 0.0;
    if (!yielding && std::fabs(err) < 0.6) {
      step = std::min(speed_ * dt, dist);
      const auto next = pos_ + to_goal * (step / dist);
      // Personal-space floor, also during a detour (when the robot is otherwise ignored):
      // never take a step that brings the body closer than min_robot_distance to the robot
      // centre - a real person does not walk into a robot standing in the way.
      if (TooCloseToRobot(next)) {
        step = 0.0;
      } else {
        pos_ = next;
      }
    }
    actor_->SetScriptTime(actor_->ScriptTime() + (step > 0.0 ? step * animation_factor_ : 0.4 * dt));
    Apply();
  }

  void StartDetour()
  {
    // sidestep perpendicular to our heading, on the side away from the robot
    double side = 1.0;
    if (robot_) {
      const auto rp = robot_->WorldPose().Pos();
      const double cross = std::cos(yaw_) * (rp.Y() - pos_.Y()) - std::sin(yaw_) * (rp.X() - pos_.X());
      side = cross > 0.0 ? -1.0 : 1.0;   // robot on our left -> step right
    }
    // ... and far enough forward to pass the robot completely before rejoining the path
    double ahead = 0.3;
    if (robot_) {
      const auto rp = robot_->WorldPose().Pos();
      ahead = std::max(0.3, std::cos(yaw_) * (rp.X() - pos_.X()) + std::sin(yaw_) * (rp.Y() - pos_.Y()) + 0.9);
    }
    detour_point_ = pos_ + ignition::math::Vector2d(-std::sin(yaw_), std::cos(yaw_)) * side +
      ignition::math::Vector2d(std::cos(yaw_), std::sin(yaw_)) * ahead;
    detour_ = true;
  }

  void AdvanceTarget()
  {
    previous_ = target_;
    const int n = static_cast<int>(waypoints_.size());
    if (ping_pong_) {
      if (target_ + direction_ < 0 || target_ + direction_ >= n) {
        direction_ = -direction_;
      }
      target_ += direction_;
    } else {
      target_ = (target_ + 1) % n;
    }
  }

  bool TooCloseToRobot(const ignition::math::Vector2d & next)
  {
    if (!robot_) {
      robot_ = world_->ModelByName(robot_name_);
      if (!robot_) {
        return false;
      }
    }
    const auto rp = robot_->WorldPose().Pos();
    const ignition::math::Vector2d r(rp.X(), rp.Y());
    const double d_next = (next - r).Length();
    // blocked only when the step goes towards the robot: stepping away is always allowed
    return d_next < min_robot_distance_ && d_next < (pos_ - r).Length();
  }

  bool RobotAhead()
  {
    if (!robot_) {
      robot_ = world_->ModelByName(robot_name_);
      if (!robot_) {
        return false;
      }
    }
    const auto rp = robot_->WorldPose().Pos();
    const ignition::math::Vector2d rel(rp.X() - pos_.X(), rp.Y() - pos_.Y());
    const double dist = rel.Length();
    if (dist > yield_distance_ + 0.3) {  // + robot half length
      return false;
    }
    const double bearing = std::remainder(std::atan2(rel.Y(), rel.X()) - yaw_, 2.0 * M_PI);
    return std::fabs(bearing) < 0.87;
  }

  void Apply()
  {
    // The walk.dae skin is modelled lying along +Y with its root at the hips.
    ignition::math::Pose3d pose(pos_.X(), pos_.Y(), 1.0, M_PI / 2.0, 0.0, yaw_ + M_PI / 2.0);
    actor_->SetWorldPose(pose, false, false);
    if (!body_name_.empty()) {
      if (!body_) {
        body_ = world_->ModelByName(body_name_);
      }
      if (body_) {
        body_->SetWorldPose(ignition::math::Pose3d(pos_.X(), pos_.Y(), 0.0, 0.0, 0.0, yaw_));
      }
    }
  }

  gazebo::physics::ActorPtr actor_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::ModelPtr robot_;
  gazebo::physics::ModelPtr body_;
  gazebo::event::ConnectionPtr update_conn_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
  std::mutex cmd_mutex_;
  std::vector<double> pending_cmd_;
  bool has_pending_cmd_{false};

  std::vector<ignition::math::Vector2d> waypoints_;
  std::vector<double> dwells_;          // per-waypoint wait (< 0: use dwell_)
  ignition::math::Vector2d pos_;
  double yaw_{0.0};
  int target_{1};
  int previous_{0};
  int direction_{1};
  double speed_{0.9};
  double dwell_{1.0};
  double start_delay_{0.0};
  double yield_distance_{0.9};
  double max_yield_time_{6.0};
  double min_robot_distance_{0.62};   // body (0.22) + robot half-diagonal (0.37) + margin
  double animation_factor_{5.1};
  bool ping_pong_{true};
  double last_time_{0.0};
  double wait_until_{0.0};
  double yield_start_{-1.0};
  double ignore_robot_until_{0.0};
  double last_ahead_{-1.0};
  bool detour_{false};
  ignition::math::Vector2d detour_point_;
  std::string body_name_;
  std::string robot_name_;
};

GZ_REGISTER_MODEL_PLUGIN(HumanActorPlugin)

}  // namespace tunibot
