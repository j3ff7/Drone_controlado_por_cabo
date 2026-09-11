// Atuacao em malha aberta do reel (A6).
//
// Representa de forma minima o conjunto futuro `motor DC + reducao -> reel`. O comando
// e um TORQUE no eixo do reel, ou seja **ja depois da reducao**: a modelagem eletrica e
// a razao de reducao ficam para etapa posterior, e por isso nenhum parametro de motor
// aparece aqui. Torque foi preferido a velocidade porque e a grandeza que o motor real
// entrega (corrente ~ torque) e porque e contra ele que `tau_est = I*alpha + b*omega`,
// validado em A4, pode ser conferido de forma independente.
//
// Tres limites, todos configuraveis:
//   tau_max    - saturacao do comando
//   ramp_rate  - variacao maxima de torque por segundo, para nao existir degrau
//   omega_max  - guarda de velocidade: acima dela, torque que aceleraria ainda mais e
//                zerado. Nao freia, apenas para de empurrar.
//
// O reel continua DESACOPLADO do comprimento do tether: girar o tambor nao libera nem
// recolhe cabo. Esse acoplamento e o objeto de A7.
#include <algorithm>
#include <cmath>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <gz/msgs/double.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace drone_cabo
{
class ReelActuator:
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
    this->modelName = this->Read<std::string>(_sdf, "reel_model", "tether_anchor_chain");
    this->jointName = this->Read<std::string>(_sdf, "reel_joint", "reel_joint");
    this->tauMax = std::fabs(this->Read<double>(_sdf, "tau_max", 0.05));
    this->omegaMax = std::fabs(this->Read<double>(_sdf, "omega_max", 12.0));
    this->rampRate = std::fabs(this->Read<double>(_sdf, "ramp_rate", 0.05));
    const auto topic = this->Read<std::string>(_sdf, "command_topic", "/cabo/tms/reel_cmd");

    this->node.Subscribe(topic, &ReelActuator::OnCommand, this);
    this->statePub =
        this->node.Advertise<gz::msgs::Vector3d>("/cabo/tms/reel_actuator");
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;
    if (!this->Resolve(_ecm))
      return;

    double requested;
    {
      std::lock_guard<std::mutex> lock(this->commandMutex);
      requested = this->command;
    }

    // 1. saturacao pelo torque maximo
    double target = std::clamp(requested, -this->tauMax, this->tauMax);
    const bool clampedByTau = (target != requested);

    // 2. rampa: o torque aplicado persegue o alvo com taxa limitada
    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (dt > 0.0 && this->rampRate > 0.0)
    {
      const double step = this->rampRate * dt;
      const double delta = std::clamp(target - this->applied, -step, step);
      this->applied += delta;
    }
    else
    {
      this->applied = target;
    }

    // 3. guarda de velocidade: nao empurra acima de omega_max, mas nao freia
    bool limitedByOmega = false;
    auto velocity = this->reelJoint.Velocity(_ecm);
    if (velocity && !velocity->empty())
    {
      const double omega = velocity->front();
      if (std::fabs(omega) >= this->omegaMax && this->applied * omega > 0.0)
      {
        this->applied = 0.0;
        limitedByOmega = true;
      }
    }

    this->reelJoint.SetForce(_ecm, {this->applied});

    gz::msgs::Vector3d msg;
    // Carimba o tempo simulado. Com comando zero e torque zero, um Vector3d nao
    // imprime campo algum e a mensagem some do stream de texto — foi o que escondeu
    // o caso M0 na primeira execucao. O header garante conteudo sempre.
    const auto simNs = std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime);
    auto *stamp = msg.mutable_header()->mutable_stamp();
    stamp->set_sec(static_cast<int64_t>(simNs.count() / 1000000000));
    stamp->set_nsec(static_cast<int32_t>(simNs.count() % 1000000000));
    msg.set_x(requested);
    msg.set_y(this->applied);
    msg.set_z((clampedByTau ? 1.0 : 0.0) + (limitedByOmega ? 2.0 : 0.0));
    this->statePub.Publish(msg);
  }

  private: void OnCommand(const gz::msgs::Double &_msg)
  {
    std::lock_guard<std::mutex> lock(this->commandMutex);
    this->command = std::isfinite(_msg.data()) ? _msg.data() : 0.0;
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

    auto joints = _ecm.ChildrenByComponents(
        model, gz::sim::components::Joint(), gz::sim::components::Name(this->jointName));
    if (joints.empty())
      return false;

    this->reelJoint = gz::sim::Joint(joints.front());
    this->reelJoint.EnableVelocityCheck(_ecm);
    this->resolved = true;
    return true;
  }

  private: std::string modelName{"tether_anchor_chain"};
  private: std::string jointName{"reel_joint"};
  private: double tauMax{0.05};
  private: double omegaMax{12.0};
  private: double rampRate{0.05};
  private: double command{0.0};
  private: double applied{0.0};
  private: bool resolved{false};
  private: std::mutex commandMutex;
  private: gz::sim::Joint reelJoint;
  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher statePub;
};
}

GZ_ADD_PLUGIN(
    drone_cabo::ReelActuator,
    gz::sim::System,
    drone_cabo::ReelActuator::ISystemConfigure,
    drone_cabo::ReelActuator::ISystemPreUpdate)
