// gz-sim system: applies a body-fixed wrench to a link at every physics step.
//
// The wrench (gz.msgs.Wrench on <topic>) is expressed in the link frame and applied at the link
// origin; it is rotated to the world frame with the current link orientation at every step, so a
// lateral force stays lateral while the vessel yaws (the simulation counterpart of a thruster
// rotated to push sideways). The last command is kept until a new one arrives.
//
//   <plugin filename="model_identification_usv_body_wrench"
//           name="model_identification_usv::BodyWrench">
//     <link_name>usv::base_link</link_name>   <!-- scoped name -->
//     <topic>/id/body_wrench</topic>
//   </plugin>
//
// It can be attached to the world or to a model.

#include <mutex>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/wrench.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

namespace model_identification_usv
{

class BodyWrench
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    link_name_ = sdf->Get<std::string>("link_name", "usv::base_link").first;
    const auto topic = sdf->Get<std::string>("topic", "/id/body_wrench").first;
    node_.Subscribe(topic, &BodyWrench::OnWrench, this);
    gzmsg << "BodyWrench: link [" << link_name_ << "], topic [" << topic << "]\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused) {
      return;
    }
    if (link_ == gz::sim::kNullEntity) {
      const auto ents = gz::sim::entitiesFromScopedName(link_name_, ecm);
      if (ents.empty()) {
        return;
      }
      link_ = *ents.begin();
      gzmsg << "BodyWrench: found link entity " << link_ << "\n";
    }
    gz::math::Vector3d f, t;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      f = force_;
      t = torque_;
    }
    if (f == gz::math::Vector3d::Zero && t == gz::math::Vector3d::Zero) {
      return;
    }
    const auto pose = gz::sim::worldPose(link_, ecm);
    gz::sim::Link(link_).AddWorldWrench(ecm, pose.Rot().RotateVector(f), pose.Rot().RotateVector(t));
  }

private:
  void OnWrench(const gz::msgs::Wrench & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    force_.Set(msg.force().x(), msg.force().y(), msg.force().z());
    torque_.Set(msg.torque().x(), msg.torque().y(), msg.torque().z());
  }

  std::string link_name_;
  gz::sim::Entity link_{gz::sim::kNullEntity};
  gz::transport::Node node_;
  std::mutex mutex_;
  gz::math::Vector3d force_, torque_;
};

}  // namespace model_identification_usv

GZ_ADD_PLUGIN(
  model_identification_usv::BodyWrench, gz::sim::System,
  model_identification_usv::BodyWrench::ISystemConfigure,
  model_identification_usv::BodyWrench::ISystemPreUpdate)
