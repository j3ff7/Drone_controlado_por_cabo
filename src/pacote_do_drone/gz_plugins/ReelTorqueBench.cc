// Bancada isolada de observabilidade do torque no reel_joint (A4).
//
// Aplica uma forca CONSTANTE NO FRAME DO CORPO num braco de alavanca conhecido do
// reel_link, o que produz um torque constante em torno do eixo da junta:
//
//     r_body = (0, 0, R)      F_body = (Fx, 0, 0)
//     tau_body = r_body x F_body = (0, R*Fx, 0)      // paralelo ao eixo (0,1,0)
//
// Aplicar a forca no frame do MUNDO daria um torque que varia com a rotacao, como
// gravidade num pendulo; por isso a forca e rotacionada para o mundo a cada passo.
//
// Publica /bancada/reel/state como Vector3d: x = theta [rad], y = omega [rad/s],
// z = tau_ref [N.m]. Sem estado de junta disponivel, x e y saem NaN — nunca zero.
//
// O experimento com Joint::EnableTransmittedWrenchCheck fica atras de uma flag SDF,
// desligada por padrao, e existe SOMENTE nesta bancada.
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <gz/math/Vector3.hh>
#include <gz/msgs/vector3d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace drone_cabo
{
class ReelTorqueBench:
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
    this->modelName = this->Read<std::string>(_sdf, "reel_model", "reel_bench");
    this->linkName = this->Read<std::string>(_sdf, "reel_link", "reel_link");
    this->jointName = this->Read<std::string>(_sdf, "reel_joint", "reel_joint");
    this->leverArm = this->Read<double>(_sdf, "lever_arm", 0.07);
    this->bodyForceX = this->Read<double>(_sdf, "body_force_x", 0.0);
    this->probeWrench = this->Read<bool>(_sdf, "probe_transmitted_wrench", false);
    this->statePub = this->node.Advertise<gz::msgs::Vector3d>("/bancada/reel/state");
    if (this->probeWrench)
    {
      this->wrenchPub =
          this->node.Advertise<gz::msgs::Vector3d>("/bancada/reel/wrench_torque");
    }
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;

    if (!this->Resolve(_ecm))
      return;

    const auto pose = this->reelLink.WorldPose(_ecm);
    if (pose)
    {
      const gz::math::Vector3d forceBody(this->bodyForceX, 0.0, 0.0);
      const gz::math::Vector3d offsetBody(0.0, 0.0, this->leverArm);
      const auto forceWorld = pose->Rot().RotateVector(forceBody);
      this->reelLink.AddWorldForce(_ecm, forceWorld, offsetBody);
    }

    auto position = this->reelJoint.Position(_ecm);
    auto velocity = this->reelJoint.Velocity(_ecm);

    gz::msgs::Vector3d msg;
    // Carimba o tempo simulado no header. Alem de dar a base de tempo exata para
    // derivar alpha, garante que a mensagem nunca fique vazia: com theta, omega e
    // tau_ref todos em zero (caso T0) o protobuf omitiria os tres campos.
    const auto simNs = std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime);
    auto *stamp = msg.mutable_header()->mutable_stamp();
    stamp->set_sec(static_cast<int64_t>(simNs.count() / 1000000000));
    stamp->set_nsec(static_cast<int32_t>(simNs.count() % 1000000000));
    if (position && !position->empty() && velocity && !velocity->empty())
    {
      msg.set_x(position->front());
      msg.set_y(velocity->front());
    }
    else
    {
      const double unavailable = std::numeric_limits<double>::quiet_NaN();
      msg.set_x(unavailable);
      msg.set_y(unavailable);
    }
    msg.set_z(this->leverArm * this->bodyForceX);
    this->statePub.Publish(msg);

    this->PublishWrenchProbe(_ecm);
  }

  // Experimento isolado de A4: le o wrench transmitido e projeta no eixo da junta.
  // Habilitar E consultar, porque foi na consulta que o modelo completo abortou.
  private: void PublishWrenchProbe(gz::sim::EntityComponentManager &_ecm)
  {
    if (!this->probeWrench)
      return;

    auto wrench = this->reelJoint.TransmittedWrench(_ecm);
    gz::msgs::Vector3d msg;
    if (wrench && !wrench->empty() && wrench->front().has_torque())
    {
      const auto &torque = wrench->front().torque();
      const gz::math::Vector3d vec(torque.x(), torque.y(), torque.z());
      msg.set_x(vec.Dot(gz::math::Vector3d(0, 1, 0)));
      msg.set_y(vec.Length());
      msg.set_z(1.0);
    }
    else
    {
      const double unavailable = std::numeric_limits<double>::quiet_NaN();
      msg.set_x(unavailable);
      msg.set_y(unavailable);
      msg.set_z(0.0);
    }
    this->wrenchPub.Publish(msg);
  }

  private: template <typename T>
  T Read(const std::shared_ptr<const sdf::Element> &_sdf,
      const std::string &_name, const T &_default) const
  {
    if (_sdf->HasElement(_name))
      return _sdf->Get<T>(_name);
    return _default;
  }

  private: bool Resolve(gz::sim::EntityComponentManager &_ecm)
  {
    if (this->resolved)
      return true;

    auto model = _ecm.EntityByComponents(
        gz::sim::components::Model(),
        gz::sim::components::Name(this->modelName));
    if (model == gz::sim::kNullEntity)
      return false;

    auto links = _ecm.ChildrenByComponents(
        model, gz::sim::components::Link(), gz::sim::components::Name(this->linkName));
    auto joints = _ecm.ChildrenByComponents(
        model, gz::sim::components::Joint(), gz::sim::components::Name(this->jointName));
    if (links.empty() || joints.empty())
      return false;

    this->reelLink = gz::sim::Link(links.front());
    this->reelJoint = gz::sim::Joint(joints.front());
    this->reelLink.EnableVelocityChecks(_ecm);
    this->reelJoint.EnablePositionCheck(_ecm);
    this->reelJoint.EnableVelocityCheck(_ecm);
    if (this->probeWrench)
    {
      // Experimento isolado de A4. Derrubou o DART no modelo completo (A0.2/A1);
      // aqui nao ha nenhuma ball joint, entao o teste separa as duas hipoteses.
      this->reelJoint.EnableTransmittedWrenchCheck(_ecm);
    }
    this->resolved = true;
    return true;
  }

  private: std::string modelName{"reel_bench"};
  private: std::string linkName{"reel_link"};
  private: std::string jointName{"reel_joint"};
  private: double leverArm{0.07};
  private: double bodyForceX{0.0};
  private: bool probeWrench{false};
  private: bool resolved{false};
  private: gz::sim::Link reelLink;
  private: gz::sim::Joint reelJoint;
  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher statePub;
  private: gz::transport::Node::Publisher wrenchPub;
};
}

GZ_ADD_PLUGIN(
    drone_cabo::ReelTorqueBench,
    gz::sim::System,
    drone_cabo::ReelTorqueBench::ISystemConfigure,
    drone_cabo::ReelTorqueBench::ISystemPreUpdate)
