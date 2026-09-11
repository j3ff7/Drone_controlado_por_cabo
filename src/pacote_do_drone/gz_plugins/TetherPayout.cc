// Acoplamento reel -> comprimento efetivo do tether (A7).
//
// Estrategia: uma junta PRISMATICA na saida da guia, cuja extensao `s` e o cabo
// liberado. O comprimento efetivo e L = L_nominal + s. Escolhida entre as
// alternativas por preservar continuidade de posicao e de velocidade, nao alterar a
// topologia em runtime, nao criar nem destruir corpos e nao mexer nas ball joints,
// que ja se mostraram fragis no DART (A0.2/A1).
//
// A prismatica e comandada em VELOCIDADE a partir do reel:
//
//     L_dot = R_eff * omega_reel        (s_dot = L_dot, pois L = L_nominal + s)
//
// Convencao de sinal: omega > 0 libera cabo (payout, L cresce);
//                     omega < 0 recolhe (retraction, L diminui).
//
// Limites: `s` fica em [s_min, s_max] (a propria junta tem <limit>, e o plugin ainda
// zera o comando ao encostar, para nao empurrar contra o batente); `rate_max` e uma
// guarda de velocidade de payout.
//
// A8 fechara a malha de tensao sobre este mecanismo. Aqui o reel segue em malha aberta.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

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
class TetherPayout:
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
    this->modelName = this->Read<std::string>(_sdf, "model_name", "tether_anchor_chain");
    this->reelJointName = this->Read<std::string>(_sdf, "reel_joint", "reel_joint");
    this->payoutJointName = this->Read<std::string>(_sdf, "payout_joint", "tether_payout");
    this->rEff = this->Read<double>(_sdf, "r_eff", 0.07);
    this->lengthNominal = this->Read<double>(_sdf, "length_nominal", 2.5);
    this->sMin = this->Read<double>(_sdf, "s_min", 0.0);
    this->sMax = this->Read<double>(_sdf, "s_max", 1.0);
    this->rateMax = std::fabs(this->Read<double>(_sdf, "rate_max", 0.5));
    this->drive = this->Read<std::string>(_sdf, "drive", "force");
    this->kp = this->Read<double>(_sdf, "kp", 200.0);
    this->kd = this->Read<double>(_sdf, "kd", 20.0);
    this->forceMax = std::fabs(this->Read<double>(_sdf, "force_max", 20.0));

    this->statePub = this->node.Advertise<gz::msgs::Vector3d>("/cabo/tms/payout");
    // Referencia publicada pelo MESMO plugin no MESMO instante que L, para que o erro
    // de rastreamento nao carregue erro de correlacao entre topicos.
    this->refPub = this->node.Advertise<gz::msgs::Vector3d>("/cabo/tms/payout_ref");
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;
    if (!this->Resolve(_ecm))
      return;

    auto reelVel = this->reelJoint.Velocity(_ecm);
    auto reelPos = this->reelJoint.Position(_ecm);
    auto payoutPos = this->payoutJoint.Position(_ecm);
    auto payoutVel = this->payoutJoint.Velocity(_ecm);
    if (!reelVel || reelVel->empty() || !reelPos || reelPos->empty()
        || !payoutPos || payoutPos->empty())
    {
      this->PublishUnavailable(_info);
      return;
    }

    const double omega = reelVel->front();
    const double s = payoutPos->front();

    // Velocidade desejada a partir do reel, saturada pela guarda de taxa.
    double target = this->rEff * omega;
    bool rateLimited = false;
    if (std::fabs(target) > this->rateMax)
    {
      target = std::copysign(this->rateMax, target);
      rateLimited = true;
    }

    // Nao empurra contra o batente: ao encostar, so admite comando que afasta.
    bool atLimit = false;
    if (s <= this->sMin && target < 0.0)
    {
      target = 0.0;
      atLimit = true;
    }
    else if (s >= this->sMax && target > 0.0)
    {
      target = 0.0;
      atLimit = true;
    }

    // Integra a referencia de comprimento a partir do reel e persegue com PD por
    // FORCA. Comando de VELOCIDADE nesta prismatica fez o comprimento divergir
    // (L chegou a 1e117 m ja com comando zero); `SetForce` e o caminho estavel, e
    // ainda deixa o payout complacente em vez de infinitamente rigido.
    const double dt = std::chrono::duration<double>(_info.dt).count();
    this->sTarget = std::clamp(this->sTarget + target * dt, this->sMin, this->sMax);

    const double sDot = payoutVel && !payoutVel->empty() ? payoutVel->front() : 0.0;
    if (this->drive == "velocity")
    {
      this->payoutJoint.SetVelocity(_ecm, {target});
    }
    else if (this->drive == "force")
    {
      double force = this->kp * (this->sTarget - s) + this->kd * (target - sDot);
      force = std::clamp(force, -this->forceMax, this->forceMax);
      this->payoutJoint.SetForce(_ecm, {force});
      this->appliedForce = force;
    }
    // drive == "none": prismatica livre, so para diagnostico.

    gz::msgs::Vector3d msg;
    this->Stamp(msg, _info);
    msg.set_x(this->lengthNominal + s);
    msg.set_y(sDot);
    msg.set_z((atLimit ? 1.0 : 0.0) + (rateLimited ? 2.0 : 0.0));
    this->statePub.Publish(msg);

    const double theta = reelPos->front();
    gz::msgs::Vector3d ref;
    this->Stamp(ref, _info);
    ref.set_x(theta);
    // A referencia e o alvo integrado e ja saturado pelos limites: e contra ele que o
    // rastreamento deve ser julgado, nao contra R_eff*theta sem saturacao.
    ref.set_y(this->lengthNominal + this->sTarget);
    ref.set_z(this->rEff);
    this->refPub.Publish(ref);
  }

  private: void PublishUnavailable(const gz::sim::UpdateInfo &_info)
  {
    const double unavailable = std::numeric_limits<double>::quiet_NaN();
    gz::msgs::Vector3d msg;
    this->Stamp(msg, _info);
    msg.set_x(unavailable);
    msg.set_y(unavailable);
    msg.set_z(-1.0);
    this->statePub.Publish(msg);
  }

  private: void Stamp(gz::msgs::Vector3d &_msg, const gz::sim::UpdateInfo &_info) const
  {
    // Sem carimbo, uma mensagem toda zerada nao imprime campo algum e some do stream.
    const auto simNs = std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime);
    auto *stamp = _msg.mutable_header()->mutable_stamp();
    stamp->set_sec(static_cast<int64_t>(simNs.count() / 1000000000));
    stamp->set_nsec(static_cast<int32_t>(simNs.count() % 1000000000));
  }

  private: template <typename T>
  T Read(const std::shared_ptr<const sdf::Element> &_sdf,
      const std::string &_name, const T &_default) const
  {
    if (_sdf->HasElement(_name))
      return _sdf->Get<T>(_name);
    return _default;
  }

  private: gz::sim::Entity JointEntity(
      gz::sim::EntityComponentManager &_ecm,
      const gz::sim::Entity &_model, const std::string &_name) const
  {
    auto joints = _ecm.ChildrenByComponents(
        _model, gz::sim::components::Joint(), gz::sim::components::Name(_name));
    return joints.empty() ? gz::sim::kNullEntity : joints.front();
  }

  private: bool Resolve(gz::sim::EntityComponentManager &_ecm)
  {
    if (this->resolved)
      return true;

    auto model = _ecm.EntityByComponents(
        gz::sim::components::Model(), gz::sim::components::Name(this->modelName));
    if (model == gz::sim::kNullEntity)
      return false;

    auto reel = this->JointEntity(_ecm, model, this->reelJointName);
    auto payout = this->JointEntity(_ecm, model, this->payoutJointName);
    if (reel == gz::sim::kNullEntity || payout == gz::sim::kNullEntity)
      return false;

    this->reelJoint = gz::sim::Joint(reel);
    this->payoutJoint = gz::sim::Joint(payout);
    this->reelJoint.EnableVelocityCheck(_ecm);
    this->reelJoint.EnablePositionCheck(_ecm);
    this->payoutJoint.EnablePositionCheck(_ecm);
    this->payoutJoint.EnableVelocityCheck(_ecm);
    this->resolved = true;
    return true;
  }

  private: std::string modelName{"tether_anchor_chain"};
  private: std::string reelJointName{"reel_joint"};
  private: std::string payoutJointName{"tether_payout"};
  private: double rEff{0.07};
  private: double lengthNominal{2.5};
  private: double sMin{0.0};
  private: double sMax{1.0};
  private: double rateMax{0.5};
  private: std::string drive{"force"};
  private: double kp{200.0};
  private: double kd{20.0};
  private: double forceMax{20.0};
  private: double sTarget{0.0};
  private: double appliedForce{0.0};
  private: bool resolved{false};
  private: gz::sim::Joint reelJoint;
  private: gz::sim::Joint payoutJoint;
  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher statePub;
  private: gz::transport::Node::Publisher refPub;
};
}

GZ_ADD_PLUGIN(
    drone_cabo::TetherPayout,
    gz::sim::System,
    drone_cabo::TetherPayout::ISystemConfigure,
    drone_cabo::TetherPayout::ISystemPreUpdate)
