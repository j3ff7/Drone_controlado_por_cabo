#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <chrono>
#include <vector>
#include <string>

#include <gz/math/Vector3.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include "TetherGeometry.hh"

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
    this->exitLinkName = this->Read<std::string>(_sdf, "exit_link", "tether_exit_point");
    this->reelJointName = this->Read<std::string>(_sdf, "reel_joint", "reel_joint");
    this->tetherLinkCount = this->Read<int>(_sdf, "tether_link_count", 5);
    this->exitSegmentName = this->Read<std::string>(_sdf, "exit_segment_link", "tether_link_1");
    this->payoutLinkName = this->Read<std::string>(_sdf, "payout_link", "");
    this->droneOffset = this->ReadVector(_sdf, "drone_offset", gz::math::Vector3d(0, 0, 0));
    this->tetherOffset = this->ReadVector(_sdf, "tether_offset", gz::math::Vector3d(0, 0, -0.5));
    this->stiffness = this->Read<double>(_sdf, "stiffness", 20.0);
    this->damping = this->Read<double>(_sdf, "damping", 4.0);
    this->maxForce = this->Read<double>(_sdf, "max_force", 20.0);
    this->forcePub = this->node.Advertise<gz::msgs::Vector3d>("/cabo/conexao/force");
    this->errorPub = this->node.Advertise<gz::msgs::Vector3d>("/cabo/conexao/error");
    this->statsPub = this->node.Advertise<gz::msgs::Vector3d>("/cabo/conexao/stats");
    this->anchorStatsPub = this->node.Advertise<gz::msgs::Vector3d>("/cabo/anchor/stats");
    this->exitPosePub = this->node.Advertise<gz::msgs::Pose>("/cabo/estacao/exit_pose");
    this->reelStatePub = this->node.Advertise<gz::msgs::Vector3d>("/cabo/estacao/reel_state");
    this->tensionPub = this->node.Advertise<gz::msgs::Vector3d>("/cabo/estacao/tensao");
    this->exitForcePub = this->node.Advertise<gz::msgs::Vector3d>("/cabo/estacao/exit_force");
    this->exitTangentPub = this->node.Advertise<gz::msgs::Vector3d>("/cabo/estacao/exit_tangent");
    this->tangentBodyPub = this->node.Advertise<gz::msgs::Vector3d>("/cabo/conexao/tangent_body");
    this->anglesPub = this->node.Advertise<gz::msgs::Vector3d>("/cabo/conexao/angles");
    this->forceBodyPub = this->node.Advertise<gz::msgs::Vector3d>("/cabo/conexao/force_body");
    this->tangentWindow = this->Read<double>(_sdf, "tangent_window", 0.15);
    this->tangentSamples = this->Read<int>(_sdf, "tangent_samples", 4);
    this->tHatWorldPub = this->node.Advertise<gz::msgs::Vector3d>("/cabo/conexao/t_hat_world");
    this->tHatBodyPub = this->node.Advertise<gz::msgs::Vector3d>("/cabo/conexao/t_hat_body");
    this->anglesBodyPub = this->node.Advertise<gz::msgs::Vector3d>("/cabo/conexao/angles_body");
    this->anglesWorldPub = this->node.Advertise<gz::msgs::Vector3d>("/cabo/conexao/angles_world");
    this->droneRpyPub = this->node.Advertise<gz::msgs::Vector3d>("/cabo/conexao/drone_rpy");
    this->lastLinkWorldPub =
        this->node.Advertise<gz::msgs::Vector3d>("/cabo/conexao/t_hat_world_last_link");
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
    // Bancada de atitude: um corpo ESTATICO no lugar do drone nao recebe WorldPose nem
    // velocidade da fisica. Compomos a pose pela arvore e usamos velocidade zero; com o
    // X500 dinamico os componentes existem e nada muda.
    if (!dronePose)
      dronePose = gz::sim::worldPose(this->droneLink.Entity(), _ecm);
    if (!droneVel)
      droneVel = gz::math::Vector3d(0, 0, 0);
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

    this->PublishConnectionGeometry(*dronePose, *tetherPose, forceOnTether);
    this->PublishAnchorUnavailable();
    this->PublishExitPose(_ecm);
    this->PublishReelState(_ecm);
    this->PublishGroundTension(_info, _ecm, forceOnTether);
    this->PublishSmoothedTangent(_ecm, *dronePose, pTether, forceOnTether);
  }

  // Tangente suavizada junto ao UAV (TetherGeometry.hh) e sua compensacao de atitude.
  //
  //   poligonal: P0 = ponta do cabo (tether_link_N + tether_offset), P1 = origem de
  //              tether_link_N, P2 = origem de tether_link_{N-1}, ..., tether_link_1
  //   t_hat_world = regressao de P(s) em s in [0, tangent_window], tangent_samples pontos
  //   t_hat_body  = R_BW * t_hat_world = q_WB^-1 * t_hat_world, com q_WB a orientacao
  //                 atual do link do drone (tether_attach_link), sem supor atitude nula
  //
  // Topicos (gz.msgs.Vector3d, frames FLU do Gazebo):
  //   /cabo/conexao/t_hat_world            t_hat_world
  //   /cabo/conexao/t_hat_body             t_hat_body
  //   /cabo/conexao/angles_body            (azimute_body, elevacao_body, angulo forca x t_hat) [graus]
  //   /cabo/conexao/angles_world           (azimute_world, elevacao_world, janela usada [m])
  //   /cabo/conexao/drone_rpy              (roll, pitch, yaw) do link do drone [graus]
  //   /cabo/conexao/t_hat_world_last_link  -R_N (1,0,0): so o ultimo elo, para comparacao
  // So leitura: nada aqui altera a dinamica. Os topicos antigos (tangent_body, angles)
  // continuam publicando a medida pelo ultimo elo.
  private: void PublishSmoothedTangent(
      gz::sim::EntityComponentManager &_ecm,
      const gz::math::Pose3d &_drone,
      const gz::math::Vector3d &_tip,
      const gz::math::Vector3d &_forceOnTether)
  {
    if (!this->tetherResolved)
      return;
    const size_t count = std::min(this->tetherLinks.size(),
                                  static_cast<size_t>(std::max(this->tetherLinkCount, 0)));
    std::vector<gz::math::Vector3d> points;
    points.reserve(count + 1);
    points.push_back(_tip);
    gz::math::Quaterniond lastLinkRot;
    for (size_t k = count; k >= 1; --k)
    {
      auto pose = this->tetherLinks[k - 1].WorldPose(_ecm);
      if (!pose)
        return;
      if (k == count)
        lastLinkRot = pose->Rot();
      points.push_back(pose->Pos());
    }

    const auto estimate =
        geometry::SmoothedTangent(points, this->tangentWindow, this->tangentSamples);
    if (!estimate.valid)
      return;

    const gz::math::Vector3d tWorld = estimate.direction;
    const gz::math::Vector3d tBody = geometry::WorldToBody(_drone.Rot(), tWorld);
    const gz::math::Vector3d angWorld = geometry::AzimuthElevationDeg(tWorld);
    const gz::math::Vector3d angBody = geometry::AzimuthElevationDeg(tBody);
    const gz::math::Vector3d rpy = geometry::RollPitchYawDeg(_drone.Rot());
    const gz::math::Vector3d lastLink =
        (-lastLinkRot.RotateVector(gz::math::Vector3d(1, 0, 0))).Normalized();

    double misalignment = std::numeric_limits<double>::quiet_NaN();
    const gz::math::Vector3d forceOnDrone = -_forceOnTether;
    if (forceOnDrone.Length() > 1e-9)
    {
      const double c = tWorld.Dot(forceOnDrone.Normalized());
      misalignment = std::acos(std::clamp(c, -1.0, 1.0)) * 180.0 / M_PI;
    }

    auto publish = [](gz::transport::Node::Publisher &_pub, double _x, double _y, double _z)
    {
      gz::msgs::Vector3d msg;
      msg.set_x(_x);
      msg.set_y(_y);
      msg.set_z(_z);
      _pub.Publish(msg);
    };
    publish(this->tHatWorldPub, tWorld.X(), tWorld.Y(), tWorld.Z());
    publish(this->tHatBodyPub, tBody.X(), tBody.Y(), tBody.Z());
    publish(this->anglesBodyPub, angBody.X(), angBody.Y(), misalignment);
    publish(this->anglesWorldPub, angWorld.X(), angWorld.Y(), estimate.window);
    publish(this->droneRpyPub, rpy.X(), rpy.Y(), rpy.Z());
    publish(this->lastLinkWorldPub, lastLink.X(), lastLink.Y(), lastLink.Z());
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

  // A1: expor a pose do ponto de saida da ground station. Leitura simples de
  // WorldPose, sem sensor novo e sem tocar no solver. Se o link nao existir, o
  // topico simplesmente nao publica; a constraint nao depende disso.
  private: void PublishExitPose(gz::sim::EntityComponentManager &_ecm)
  {
    if (!this->exitResolved)
    {
      auto entity = this->LinkEntity(_ecm, this->tetherModel, this->exitLinkName);
      if (entity == gz::sim::kNullEntity)
        return;
      this->exitEntity = entity;
      this->exitResolved = true;
    }

    // Link::WorldPose depende de components::WorldPose, que a fisica so cria para
    // corpos que ela move. O ponto de saida esta soldado a uma base fixa ao world,
    // entao usamos gz::sim::worldPose, que compoe a pose subindo a arvore.
    const auto pose = gz::sim::worldPose(this->exitEntity, _ecm);

    gz::msgs::Pose msg;
    msg.mutable_position()->set_x(pose.Pos().X());
    msg.mutable_position()->set_y(pose.Pos().Y());
    msg.mutable_position()->set_z(pose.Pos().Z());
    msg.mutable_orientation()->set_x(pose.Rot().X());
    msg.mutable_orientation()->set_y(pose.Rot().Y());
    msg.mutable_orientation()->set_z(pose.Rot().Z());
    msg.mutable_orientation()->set_w(pose.Rot().W());
    this->exitPosePub.Publish(msg);
  }

  // A2: estado do reel passivo. Usa apenas EnablePositionCheck/EnableVelocityCheck;
  // EnableTransmittedWrenchCheck continua proibido nesta baseline. Sem medida
  // disponivel publicamos NaN, nunca zero: x=theta [rad], y=omega [rad/s],
  // z=1 quando o estado da junta esta disponivel, 0 quando nao esta.
  private: void PublishReelState(gz::sim::EntityComponentManager &_ecm)
  {
    if (this->reelJointName.empty())
      return;

    if (!this->reelResolved)
    {
      auto modelEntity = _ecm.EntityByComponents(
          gz::sim::components::Model(),
          gz::sim::components::Name(this->tetherModel));
      if (modelEntity == gz::sim::kNullEntity)
        return;
      auto joints = _ecm.ChildrenByComponents(
          modelEntity,
          gz::sim::components::Joint(),
          gz::sim::components::Name(this->reelJointName));
      if (joints.empty())
      {
        this->PublishReelUnavailable();
        return;
      }
      this->reelJoint = gz::sim::Joint(joints.front());
      this->reelJoint.EnablePositionCheck(_ecm);
      this->reelJoint.EnableVelocityCheck(_ecm);
      this->reelResolved = true;
    }

    auto position = this->reelJoint.Position(_ecm);
    auto velocity = this->reelJoint.Velocity(_ecm);
    if (!position || position->empty() || !velocity || velocity->empty())
    {
      this->PublishReelUnavailable();
      return;
    }

    gz::msgs::Vector3d msg;
    msg.set_x(position->front());
    msg.set_y(velocity->front());
    msg.set_z(1.0);
    this->reelStatePub.Publish(msg);
  }

  private: void PublishReelUnavailable()
  {
    const double unavailable = std::numeric_limits<double>::quiet_NaN();
    gz::msgs::Vector3d msg;
    msg.set_x(unavailable);
    msg.set_y(unavailable);
    msg.set_z(0.0);
    this->reelStatePub.Publish(msg);
  }

  // A5: tensao no lado terrestre por corpo livre de todo o cabo.
  //
  // Somatorio de Newton sobre os N elos, cujas unicas forcas externas sao a reacao da
  // guia, a forca da constraint na ponta do UAV e o peso:
  // Direcao local do cabo junto ao UAV, sem junta fisica. O ultimo elo se estende no
  // +x local ate a ponta ligada ao drone, entao a direcao que SAI do drone ao longo do
  // cabo e -R_N * (1,0,0). Expressa no frame do link do drone (tether_attach_link, mesma
  // orientacao do base_link do X500: x frente, y esquerda, z cima):
  //
  //     t_body   = R_drone^-1 * t_world
  //     azimute  = atan2(t_y, t_x)                  0 = frente, +90 = esquerda
  //     elevacao = atan2(t_z, sqrt(t_x^2 + t_y^2))  -90 = cabo pendurado reto abaixo
  //
  // A forca da constraint sobre o drone (-F) aponta do drone para a ponta do cabo. Com a
  // complacencia e = F/K ela nao precisa coincidir com a tangente; o angulo entre as duas
  // e publicado para medir quanto a forca, sozinha, representa a direcao do cabo.
  // So leitura: nada aqui altera a dinamica.
  private: void PublishConnectionGeometry(
      const gz::math::Pose3d &_drone,
      const gz::math::Pose3d &_tether,
      const gz::math::Vector3d &_forceOnTether)
  {
    const gz::math::Quaterniond toBody = _drone.Rot().Inverse();
    gz::math::Vector3d tangentWorld =
        -_tether.Rot().RotateVector(gz::math::Vector3d(1, 0, 0));
    tangentWorld.Normalize();
    const gz::math::Vector3d tangentBody = toBody.RotateVector(tangentWorld);
    const gz::math::Vector3d forceOnDroneBody = toBody.RotateVector(-_forceOnTether);

    const double deg = 180.0 / M_PI;
    const double azimuth = std::atan2(tangentBody.Y(), tangentBody.X()) * deg;
    const double elevation =
        std::atan2(tangentBody.Z(), std::hypot(tangentBody.X(), tangentBody.Y())) * deg;
    double misalignment = std::numeric_limits<double>::quiet_NaN();
    const double forceNorm = forceOnDroneBody.Length();
    if (forceNorm > 1e-9)
    {
      const double c = tangentBody.Dot(forceOnDroneBody / forceNorm);
      misalignment = std::acos(std::clamp(c, -1.0, 1.0)) * deg;
    }

    gz::msgs::Vector3d tangentMsg;
    tangentMsg.set_x(tangentBody.X());
    tangentMsg.set_y(tangentBody.Y());
    tangentMsg.set_z(tangentBody.Z());
    this->tangentBodyPub.Publish(tangentMsg);

    gz::msgs::Vector3d anglesMsg;
    anglesMsg.set_x(azimuth);
    anglesMsg.set_y(elevation);
    anglesMsg.set_z(misalignment);
    this->anglesPub.Publish(anglesMsg);

    gz::msgs::Vector3d forceMsg;
    forceMsg.set_x(forceOnDroneBody.X());
    forceMsg.set_y(forceOnDroneBody.Y());
    forceMsg.set_z(forceOnDroneBody.Z());
    this->forceBodyPub.Publish(forceMsg);
  }

  //
  //     sum(m_i a_i) = F_exit + F_c + sum(m_i g)
  //     F_exit       = sum(m_i a_i) - F_c - sum(m_i g)
  //
  // Nao usa TransmittedWrench (que abortou o DART com ball joints) nem tau_reel/R.
  // A tensao e a componente axial ao longo da tangente do primeiro segmento:
  //
  //     t_hat = R_link1 * (1,0,0)          // o elo se estende no +x local
  //     T_est = |F_exit . t_hat|
  private: void PublishGroundTension(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm,
      const gz::math::Vector3d &_forceOnTether)
  {
    if (!this->ResolveTether(_ecm))
    {
      this->PublishTensionUnavailable();
      return;
    }

    // WorldLinearAcceleration nao e populada para estes elos nesta versao do
    // gz-sim, entao a aceleracao vem de diferenciar WorldLinearVelocity, que a
    // constraint ja usa e sabemos disponivel. dt e o passo de simulacao.
    const double dt = std::chrono::duration<double>(_info.dt).count();
    gz::math::Vector3d inertial(0, 0, 0);
    gz::math::Vector3d weight(0, 0, 0);
    std::vector<gz::math::Vector3d> velocities(this->tetherLinks.size());
    for (size_t i = 0; i < this->tetherLinks.size(); ++i)
    {
      auto velocity = this->tetherLinks[i].WorldLinearVelocity(_ecm);
      if (!velocity)
      {
        this->PublishTensionUnavailable();
        return;
      }
      velocities[i] = *velocity;
      const double m = this->tetherMasses[i];
      if (this->havePreviousVelocities && dt > 0.0)
        inertial += m * (velocities[i] - this->previousVelocities[i]) / dt;
      weight += m * this->gravity;
    }

    const bool inertialReady = this->havePreviousVelocities && dt > 0.0;
    this->previousVelocities = velocities;
    this->havePreviousVelocities = true;
    if (!inertialReady)
    {
      this->PublishTensionUnavailable();
      return;
    }

    const auto exitPose = this->exitSegment.WorldPose(_ecm);
    if (!exitPose)
    {
      this->PublishTensionUnavailable();
      return;
    }

    const gz::math::Vector3d forceExit = inertial - _forceOnTether - weight;
    const gz::math::Vector3d tangent =
        exitPose->Rot().RotateVector(gz::math::Vector3d(1, 0, 0)).Normalized();

    // Termo quase-estatico: mesma conta ignorando a inercia dos elos.
    const gz::math::Vector3d forceExitQs = -_forceOnTether - weight;

    gz::msgs::Vector3d tension;
    tension.set_x(std::fabs(forceExit.Dot(tangent)));
    tension.set_y(std::fabs(forceExitQs.Dot(tangent)));
    tension.set_z(1.0);
    this->tensionPub.Publish(tension);

    gz::msgs::Vector3d force;
    force.set_x(forceExit.X());
    force.set_y(forceExit.Y());
    force.set_z(forceExit.Z());
    this->exitForcePub.Publish(force);

    gz::msgs::Vector3d hat;
    hat.set_x(tangent.X());
    hat.set_y(tangent.Y());
    hat.set_z(tangent.Z());
    this->exitTangentPub.Publish(hat);
  }

  private: void PublishTensionUnavailable()
  {
    const double unavailable = std::numeric_limits<double>::quiet_NaN();
    gz::msgs::Vector3d msg;
    msg.set_x(unavailable);
    msg.set_y(unavailable);
    msg.set_z(0.0);
    this->tensionPub.Publish(msg);
  }

  private: bool ResolveTether(gz::sim::EntityComponentManager &_ecm)
  {
    if (this->tetherResolved)
      return true;

    auto gravityComponent = _ecm.Component<gz::sim::components::Gravity>(
        gz::sim::worldEntity(_ecm));
    if (gravityComponent)
      this->gravity = gravityComponent->Data();

    std::vector<gz::sim::Link> links;
    std::vector<double> masses;
    for (int i = 1; i <= this->tetherLinkCount; ++i)
    {
      auto entity = this->LinkEntity(
          _ecm, this->tetherModel, "tether_link_" + std::to_string(i));
      if (entity == gz::sim::kNullEntity)
        return false;
      // Massa lida do proprio modelo, nunca redigitada aqui.
      auto inertial = _ecm.Component<gz::sim::components::Inertial>(entity);
      if (!inertial)
        return false;
      gz::sim::Link link(entity);
      link.EnableVelocityChecks(_ecm);
      links.push_back(link);
      masses.push_back(inertial->Data().MassMatrix().Mass());
    }

    // A7: o elo prismatico de payout faz parte do corpo livre do cabo.
    if (!this->payoutLinkName.empty())
    {
      auto entity = this->LinkEntity(_ecm, this->tetherModel, this->payoutLinkName);
      if (entity == gz::sim::kNullEntity)
        return false;
      auto inertial = _ecm.Component<gz::sim::components::Inertial>(entity);
      if (!inertial)
        return false;
      gz::sim::Link link(entity);
      link.EnableVelocityChecks(_ecm);
      links.push_back(link);
      masses.push_back(inertial->Data().MassMatrix().Mass());
    }

    auto exitEntity = this->LinkEntity(_ecm, this->tetherModel, this->exitSegmentName);
    if (exitEntity == gz::sim::kNullEntity)
      return false;

    this->tetherLinks = links;
    this->tetherMasses = masses;
    this->exitSegment = gz::sim::Link(exitEntity);
    this->tetherResolved = true;
    return false;   // as aceleracoes so ficam disponiveis no passo seguinte
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
  private: std::string exitLinkName{"tether_exit_point"};
  private: std::string reelJointName{"reel_joint"};
  private: std::string exitSegmentName{"tether_link_1"};
  private: std::string payoutLinkName{""};
  private: int tetherLinkCount{5};
  private: gz::math::Vector3d gravity{0, 0, -9.8};
  private: gz::math::Vector3d droneOffset{0, 0, 0};
  private: gz::math::Vector3d tetherOffset{0, 0, -0.5};
  private: double stiffness{20.0};
  private: double damping{4.0};
  private: double maxForce{20.0};
  private: bool resolved{false};
  private: bool exitResolved{false};
  private: bool reelResolved{false};
  private: bool tetherResolved{false};
  private: bool havePreviousVelocities{false};
  private: std::vector<gz::math::Vector3d> previousVelocities;
  private: gz::sim::Link droneLink;
  private: gz::sim::Link tetherLink;
  private: gz::sim::Entity exitEntity{gz::sim::kNullEntity};
  private: gz::sim::Joint reelJoint;
  private: std::vector<gz::sim::Link> tetherLinks;
  private: std::vector<double> tetherMasses;
  private: gz::sim::Link exitSegment;
  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher forcePub;
  private: gz::transport::Node::Publisher errorPub;
  private: gz::transport::Node::Publisher statsPub;
  private: gz::transport::Node::Publisher anchorStatsPub;
  private: gz::transport::Node::Publisher exitPosePub;
  private: gz::transport::Node::Publisher reelStatePub;
  private: gz::transport::Node::Publisher tensionPub;
  private: gz::transport::Node::Publisher exitForcePub;
  private: gz::transport::Node::Publisher exitTangentPub;
  private: gz::transport::Node::Publisher tangentBodyPub;
  private: gz::transport::Node::Publisher anglesPub;
  private: gz::transport::Node::Publisher forceBodyPub;
  private: double tangentWindow{0.15};
  private: int tangentSamples{4};
  private: gz::transport::Node::Publisher tHatWorldPub;
  private: gz::transport::Node::Publisher tHatBodyPub;
  private: gz::transport::Node::Publisher anglesBodyPub;
  private: gz::transport::Node::Publisher anglesWorldPub;
  private: gz::transport::Node::Publisher droneRpyPub;
  private: gz::transport::Node::Publisher lastLinkWorldPub;
};
}

GZ_ADD_PLUGIN(
    drone_cabo::TetherForceConstraint,
    gz::sim::System,
    drone_cabo::TetherForceConstraint::ISystemConfigure,
    drone_cabo::TetherForceConstraint::ISystemPreUpdate)
