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
//   - if it has been waiting more than `max_yield_time` s, it turns around and
//     walks back to the previous waypoint, ignoring the robot for 4 s while it
//     does so (two polite agents waiting for each other would deadlock).
//
// SDF parameters (all optional except <waypoint>):
//   <waypoint>x y</waypoint>        (repeat, world frame, metres)
//   <speed>0.9</speed>              walking speed, m/s
//   <dwell>1.0</dwell>              pause at each waypoint, s
//   <start_delay>0</start_delay>    s of simulation time before starting to walk
//   <body>human_1</body>            name of the collision proxy model
//   <robot>waiter_robot</robot>     model to yield to
//   <yield_distance>0.9</yield_distance>
//   <max_yield_time>6.0</max_yield_time>
//   <animation_factor>5.1</animation_factor>
//   <ping_pong>true</ping_pong>     true: A-B-C-B-A..., false: A-B-C-A-B...

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

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
      for (auto e = sdf->GetElement("waypoint"); e; e = e->GetNextElement("waypoint")) {
        const auto v = e->Get<ignition::math::Vector2d>();
        waypoints_.emplace_back(v.X(), v.Y());
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
    animation_factor_ = sdf->Get<double>("animation_factor", 5.1).first;
    ping_pong_ = sdf->Get<bool>("ping_pong", true).first;

    pos_ = waypoints_[0];
    const auto d = waypoints_[1] - waypoints_[0];
    yaw_ = std::atan2(d.Y(), d.X());
    target_ = 1;

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
  void OnUpdate(const gazebo::common::UpdateInfo & info)
  {
    const double now = info.simTime.Double();
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
    bool yielding = false;
    if (now >= ignore_robot_until_ && RobotAhead()) {
      if (yield_start_ < 0.0) {
        yield_start_ = now;
      }
      if (now - yield_start_ < max_yield_time_) {
        yielding = true;
      } else {
        // Waited long enough: give way by walking back to the previous waypoint,
        // without re-checking the robot while turning around and leaving.
        std::swap(target_, previous_);
        direction_ = -direction_;
        ignore_robot_until_ = now + 4.0;
        yield_start_ = -1.0;
      }
    } else if (now >= ignore_robot_until_) {
      yield_start_ = -1.0;
    }

    const auto goal = waypoints_[target_];
    auto to_goal = goal - pos_;
    const double dist = to_goal.Length();
    if (dist < 0.05) {
      AdvanceTarget();
      wait_until_ = now + dwell_;
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
      pos_ += to_goal * (step / dist);
    }
    actor_->SetScriptTime(actor_->ScriptTime() + (step > 0.0 ? step * animation_factor_ : 0.4 * dt));
    Apply();
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

  std::vector<ignition::math::Vector2d> waypoints_;
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
  double animation_factor_{5.1};
  bool ping_pong_{true};
  double last_time_{0.0};
  double wait_until_{0.0};
  double yield_start_{-1.0};
  double ignore_robot_until_{0.0};
  std::string body_name_;
  std::string robot_name_;
};

GZ_REGISTER_MODEL_PLUGIN(HumanActorPlugin)

}  // namespace tunibot
