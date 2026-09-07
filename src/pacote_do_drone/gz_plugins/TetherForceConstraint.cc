#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <gz/math/Vector3.hh>
#include <gz/msgs/vector3d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace drone_cabo
{
class TetherForceConstraint:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  public: void Configure(
      const gz::sim::Entity &,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &,
      gz::sim::EventManager &) override
  {
    this->droneModel = this->Read<std::string>(_sdf, "drone_model", "x500_tether_attach_0");
    this->droneLinkName = this->Read<std::string>(_sdf, "drone_link", "tether_attach_link");
    this->tetherModel = this->Read<std::string>(_sdf, "tether_model", "tether_anchor_chain");
    this->tetherLinkName = this->Read<std::string>(_sdf, "tether_link", "tether_link_5");
    this->droneOffset = this->ReadVector(_sdf, "drone_offset", gz::math::Vector3d(0, 0, 0));
    this->tetherOffset = this->ReadVector(_sdf, "tether_offset", gz::math::Vector3d(0, 0, -0.5));
    this->stiffness = this->Read<double>(_sdf, "stiffness", 20.0);
    this->damping = this->Read<double>(_sdf, "damping", 4.0);
    this->maxForce = this->Read<double>(_sdf, "max_force", 20.0);
    this->forcePub = this->node.Advertise<gz::msgs::Vector3d>("/cabo/conexao/force");
    this->errorPub = this->node.Advertise<gz::msgs::Vector3d>("/cabo/conexao/error");
    this->statsPub = this->node.Advertise<gz::msgs::Vector3d>("/cabo/conexao/stats");
    this->anchorStatsPub = this->node.Advertise<gz::msgs::Vector3d>("/cabo/anchor/stats");
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;

    if (!this->ResolveLinks(_ecm))
      return;

    auto dronePose = this->droneLink.WorldPose(_ecm);
    auto tetherPose = this->tetherLink.WorldPose(_ecm);
    auto droneVel = this->droneLink.WorldLinearVelocity(_ecm, this->droneOffset);
    auto tetherVel = this->tetherLink.WorldLinearVelocity(_ecm, this->tetherOffset);
    if (!dronePose || !tetherPose || !droneVel || !tetherVel)
      return;

    const auto pDrone = dronePose->Pos() + dronePose->Rot().RotateVector(this->droneOffset);
    const auto pTether = tetherPose->Pos() + tetherPose->Rot().RotateVector(this->tetherOffset);
    const auto error = pTether - pDrone;
    const auto errorDot = *tetherVel - *droneVel;
    auto forceOnTether = -this->stiffness * error - this->damping * errorDot;
    const double norm = forceOnTether.Length();
    bool saturated = false;
    if (norm > this->maxForce && norm > 0.0)
    {
      forceOnTether *= this->maxForce / norm;
      saturated = true;
    }
    const double forceNorm = forceOnTether.Length();

    // AddWorldForce(_ecm, F, p) aplica F no ponto p expresso no frame do link e
    // deriva sozinho o momento r x F em torno do CoM daquele link. Nao adicionar
    // torque manualmente aqui: isso contaria o braco de alavanca duas vezes.
    this->tetherLink.AddWorldForce(_ecm, forceOnTether, this->tetherOffset);
    this->droneLink.AddWorldForce(_ecm, -forceOnTether, this->droneOffset);

    gz::msgs::Vector3d forceMsg;
    forceMsg.set_x(forceOnTether.X());
    forceMsg.set_y(forceOnTether.Y());
    forceMsg.set_z(forceOnTether.Z());
    this->forcePub.Publish(forceMsg);

    gz::msgs::Vector3d errorMsg;
    errorMsg.set_x(error.X());
    errorMsg.set_y(error.Y());
    errorMsg.set_z(error.Z());
    this->errorPub.Publish(errorMsg);

    gz::msgs::Vector3d statsMsg;
    statsMsg.set_x(error.Length());
    statsMsg.set_y(forceNorm);
    statsMsg.set_z(saturated ? 1.0 : 0.0);
    this->statsPub.Publish(statsMsg);

    this->PublishAnchorUnavailable();
  }

  private: template <typename T>
  T Read(const std::shared_ptr<const sdf::Element> &_sdf,
      const std::string &_name, const T &_default) const
  {
    if (_sdf->HasElement(_name))
      return _sdf->Get<T>(_name);
    return _default;
  }

  private: gz::math::Vector3d ReadVector(
      const std::shared_ptr<const sdf::Element> &_sdf,
      const std::string &_name,
      const gz::math::Vector3d &_default) const
  {
    if (!_sdf->HasElement(_name))
      return _default;
    return _sdf->Get<gz::math::Vector3d>(_name);
  }

  private: gz::sim::Entity LinkEntity(
      const gz::sim::EntityComponentManager &_ecm,
      const std::string &_model,
      const std::string &_link) const
  {
    auto modelEntity = _ecm.EntityByComponents(
        gz::sim::components::Model(),
        gz::sim::components::Name(_model));
    if (modelEntity == gz::sim::kNullEntity)
      return gz::sim::kNullEntity;

    auto links = _ecm.ChildrenByComponents(
        modelEntity,
        gz::sim::components::Link(),
        gz::sim::components::Name(_link));
    if (links.empty())
      return gz::sim::kNullEntity;
    return links.front();
  }

  // O wrench transmitido pela ancora nao e observavel nesta configuracao:
  // habilitar Joint::EnableTransmittedWrenchCheck em anchor_world_fixed derruba o
  // backend DART (BallJoint::updateRelativeTransform). Publicamos entao x=|F| e
  // y=|M| como NaN (medida ausente, nunca 0 N) e z=0 como flag de disponibilidade.
  private: void PublishAnchorUnavailable()
  {
    const double unavailable = std::numeric_limits<double>::quiet_NaN();
    gz::msgs::Vector3d statsMsg;
    statsMsg.set_x(unavailable);
    statsMsg.set_y(unavailable);
    statsMsg.set_z(0.0);
    this->anchorStatsPub.Publish(statsMsg);
  }

  private: bool ResolveLinks(gz::sim::EntityComponentManager &_ecm)
  {
    if (!this->resolved)
    {
      auto droneEntity = this->LinkEntity(_ecm, this->droneModel, this->droneLinkName);
      auto tetherEntity = this->LinkEntity(_ecm, this->tetherModel, this->tetherLinkName);
      if (droneEntity == gz::sim::kNullEntity || tetherEntity == gz::sim::kNullEntity)
        return false;
      this->droneLink = gz::sim::Link(droneEntity);
      this->tetherLink = gz::sim::Link(tetherEntity);
      this->droneLink.EnableVelocityChecks(_ecm);
      this->tetherLink.EnableVelocityChecks(_ecm);
      this->resolved = true;
    }
    return true;
  }

  private: std::string droneModel{"x500_tether_attach_0"};
  private: std::string droneLinkName{"tether_attach_link"};
  private: std::string tetherModel{"tether_anchor_chain"};
  private: std::string tetherLinkName{"tether_link_5"};
  private: gz::math::Vector3d droneOffset{0, 0, 0};
  private: gz::math::Vector3d tetherOffset{0, 0, -0.5};
  private: double stiffness{20.0};
  private: double damping{4.0};
  private: double maxForce{20.0};
  private: bool resolved{false};
  private: gz::sim::Link droneLink;
  private: gz::sim::Link tetherLink;
  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher forcePub;
  private: gz::transport::Node::Publisher errorPub;
  private: gz::transport::Node::Publisher statsPub;
  private: gz::transport::Node::Publisher anchorStatsPub;
};
}

GZ_ADD_PLUGIN(
    drone_cabo::TetherForceConstraint,
    gz::sim::System,
    drone_cabo::TetherForceConstraint::ISystemConfigure,
    drone_cabo::TetherForceConstraint::ISystemPreUpdate)
