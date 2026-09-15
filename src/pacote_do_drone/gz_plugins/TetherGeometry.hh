#ifndef DRONE_CABO_TETHER_GEOMETRY_HH_
#define DRONE_CABO_TETHER_GEOMETRY_HH_

// Direcao local do tether junto ao UAV e sua expressao no frame do drone.
// So gz-math: sem ECM, para poder ser testado fora do Gazebo (test/cpp).
//
// Frames (Gazebo):
//   W  mundo, ENU            (x leste, y norte, z cima)
//   B  tether_attach_link    (mesma orientacao do base_link do X500: x frente,
//                             y esquerda, z cima -- FLU)
// q_WB e a orientacao do drone no mundo (Pose3d::Rot()), que leva vetores de B para W:
//   v_W = q_WB * v_B     =>     v_B = R_BW * v_W = q_WB^-1 * v_W
//
// Tangente: o cabo e uma poligonal P0 (ponta ligada ao drone), P1, P2, ... (origens dos
// elos, da ponta para a estacao). A direcao que SAI do drone ao longo do cabo e estimada
// por minimos quadrados sobre `samples` pontos igualmente espacados em comprimento de
// arco s em [0, window] a partir de P0:
//
//   p_i = P(s_i),  s_i = i * window / (samples - 1)
//   d   = sum_i (s_i - s_mean) (p_i - p_mean)          (inclinacao dp/ds)
//   t_hat_world = d / |d|
//
// Com a janela dentro de um unico elo rigido, t_hat e exatamente a direcao desse elo;
// cruzando juntas, a regressao media as orientacoes dos elos na janela.
//
// Angulos, de um vetor unitario t = (tx, ty, tz):
//   azimute  = atan2(ty, tx)                 0 = frente (+x), +90 = esquerda (+y)
//   elevacao = atan2(tz, sqrt(tx^2 + ty^2))  0 = horizontal, -90 = reto para baixo

#include <algorithm>
#include <cmath>
#include <vector>

#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

namespace drone_cabo
{
namespace geometry
{
struct TangentEstimate
{
  gz::math::Vector3d direction{0, 0, 0};
  double window{0.0};     // janela efetivamente usada [m] (limitada ao comprimento)
  int samples{0};
  bool valid{false};
};

inline double PolylineLength(const std::vector<gz::math::Vector3d> &_points)
{
  double length = 0.0;
  for (size_t i = 1; i < _points.size(); ++i)
    length += (_points[i] - _points[i - 1]).Length();
  return length;
}

// Ponto a comprimento de arco _s de _points[0], interpolado linearmente; satura no fim.
inline gz::math::Vector3d PointAtArc(
    const std::vector<gz::math::Vector3d> &_points, double _s)
{
  if (_points.empty())
    return gz::math::Vector3d(0, 0, 0);
  double remaining = std::max(0.0, _s);
  for (size_t i = 1; i < _points.size(); ++i)
  {
    const gz::math::Vector3d segment = _points[i] - _points[i - 1];
    const double length = segment.Length();
    if (remaining <= length && length > 0.0)
      return _points[i - 1] + segment * (remaining / length);
    remaining -= length;
  }
  return _points.back();
}

inline TangentEstimate SmoothedTangent(
    const std::vector<gz::math::Vector3d> &_points, double _window, int _samples)
{
  TangentEstimate out;
  const double total = PolylineLength(_points);
  const int n = std::max(2, _samples);
  const double window = std::min(std::max(_window, 0.0), total);
  if (_points.size() < 2 || window <= 0.0)
    return out;

  std::vector<double> s(n);
  std::vector<gz::math::Vector3d> p(n);
  double sMean = 0.0;
  gz::math::Vector3d pMean(0, 0, 0);
  for (int i = 0; i < n; ++i)
  {
    s[i] = window * i / (n - 1);
    p[i] = PointAtArc(_points, s[i]);
    sMean += s[i] / n;
    pMean += p[i] / n;
  }
  gz::math::Vector3d slope(0, 0, 0);
  for (int i = 0; i < n; ++i)
    slope += (s[i] - sMean) * (p[i] - pMean);
  const double norm = slope.Length();
  if (!(norm > 1e-12) || !std::isfinite(norm))
    return out;

  out.direction = slope / norm;
  out.window = window;
  out.samples = n;
  out.valid = true;
  return out;
}

// R_BW * v: vetor do mundo expresso no frame do drone.
inline gz::math::Vector3d WorldToBody(
    const gz::math::Quaterniond &_qWB, const gz::math::Vector3d &_vWorld)
{
  return _qWB.Inverse().RotateVector(_vWorld);
}

// (azimute, elevacao, 0) em graus.
inline gz::math::Vector3d AzimuthElevationDeg(const gz::math::Vector3d &_t)
{
  const double deg = 180.0 / M_PI;
  return gz::math::Vector3d(
      std::atan2(_t.Y(), _t.X()) * deg,
      std::atan2(_t.Z(), std::hypot(_t.X(), _t.Y())) * deg,
      0.0);
}

// (roll, pitch, yaw) em graus, convencao do gz-math: R = Rz(yaw) Ry(pitch) Rx(roll).
inline gz::math::Vector3d RollPitchYawDeg(const gz::math::Quaterniond &_q)
{
  return _q.Euler() * (180.0 / M_PI);
}
}  // namespace geometry
}  // namespace drone_cabo

#endif
