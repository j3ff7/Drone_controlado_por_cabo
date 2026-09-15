// Testes de src/pacote_do_drone/gz_plugins/TetherGeometry.hh (compilado e executado por
// test/test_tether_geometry_cpp.py). Sai com codigo != 0 na primeira falha.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "../../src/pacote_do_drone/gz_plugins/TetherGeometry.hh"

using gz::math::Quaterniond;
using gz::math::Vector3d;
namespace g = drone_cabo::geometry;

static int failures = 0;

static void Check(bool _ok, const char *_what)
{
  if (!_ok)
  {
    std::printf("FAIL: %s\n", _what);
    ++failures;
  }
}

static bool Near(double _a, double _b, double _tol = 1e-9)
{
  return std::fabs(_a - _b) <= _tol;
}

static bool NearV(const Vector3d &_a, const Vector3d &_b, double _tol = 1e-9)
{
  return (_a - _b).Length() <= _tol;
}

int main()
{
  // Janela dentro de um elo reto: a tangente e a direcao do elo.
  {
    std::vector<Vector3d> pts{{0, 0, 0}, {0.5, 0, -0.5}, {1.0, 0, -1.0}};
    auto t = g::SmoothedTangent(pts, 0.15, 4);
    Check(t.valid, "tangente valida em elo reto");
    Check(NearV(t.direction, Vector3d(1, 0, -1).Normalized()), "elo reto: direcao do elo");
    Check(Near(t.window, 0.15), "janela usada");
  }

  // Janela maior que o cabo: limitada ao comprimento.
  {
    std::vector<Vector3d> pts{{0, 0, 0}, {0.1, 0, 0}};
    auto t = g::SmoothedTangent(pts, 1.0, 4);
    Check(Near(t.window, 0.1), "janela limitada ao comprimento do cabo");
    Check(NearV(t.direction, Vector3d(1, 0, 0)), "cabo curto: direcao");
  }

  // Poligonal com uma dobra simetrica: a regressao fica entre os dois elos.
  {
    std::vector<Vector3d> pts{{0, 0, 0}, {0.1, 0, -0.1}, {0.2, 0, -0.1 - 0.1}};
    pts[2] = Vector3d(0.1 + 0.1, 0, -0.1);                    // segundo elo horizontal
    auto t = g::SmoothedTangent(pts, 0.2 * std::sqrt(0.5) + 0.1, 5);
    const double elev = g::AzimuthElevationDeg(t.direction).Y();
    Check(t.valid && elev < -1.0 && elev > -44.0, "dobra: elevacao entre os elos (0 e -45)");
  }

  // Pontos degenerados.
  {
    std::vector<Vector3d> pts{{1, 1, 1}, {1, 1, 1}};
    Check(!g::SmoothedTangent(pts, 0.15, 4).valid, "poligonal de comprimento zero invalida");
    Check(!g::SmoothedTangent({}, 0.15, 4).valid, "sem pontos invalida");
  }

  // Convencao dos angulos.
  {
    Check(Near(g::AzimuthElevationDeg(Vector3d(1, 0, 0)).X(), 0.0), "frente: azimute 0");
    Check(Near(g::AzimuthElevationDeg(Vector3d(0, 1, 0)).X(), 90.0), "esquerda: azimute +90");
    Check(Near(g::AzimuthElevationDeg(Vector3d(0, 0, -1)).Y(), -90.0), "abaixo: elevacao -90");
    Check(Near(g::AzimuthElevationDeg(Vector3d(1, 0, 1)).Y(), 45.0), "elevacao +45");
  }

  // world -> body com atitude nao nula.
  {
    // yaw +90: o +x do mundo fica a direita do drone (-y do corpo).
    Quaterniond yaw90(0, 0, M_PI / 2);
    Check(NearV(g::WorldToBody(yaw90, Vector3d(1, 0, 0)), Vector3d(0, -1, 0)), "yaw 90");
    // pitch +30 no gz (rotacao +y leva +x para -z: nariz para baixo). Um prumo (cabo reto
    // abaixo no mundo) visto do corpo aponta para a FRENTE e para baixo: (0,5; 0; -0,866).
    Quaterniond pitch30(0, M_PI / 6, 0);
    Vector3d tb = g::WorldToBody(pitch30, Vector3d(0, 0, -1));
    Check(NearV(tb, Vector3d(0.5, 0, -std::sqrt(3.0) / 2)), "pitch 30: prumo no corpo");
    Check(Near(g::AzimuthElevationDeg(tb).Y(), -60.0, 1e-9), "pitch 30: elevacao -60 no corpo");
    Check(Near(g::AzimuthElevationDeg(tb).X(), 0.0, 1e-9), "pitch 30: azimute 0 (frente)");
    // roll +20 (rotacao +x leva +y para +z: lado esquerdo sobe). O prumo visto do corpo
    // aponta para a DIREITA e para baixo: (0; -0,342; -0,940), azimute -90.
    Quaterniond roll20(M_PI / 9, 0, 0);
    Vector3d tr = g::WorldToBody(roll20, Vector3d(0, 0, -1));
    Check(NearV(tr, Vector3d(0, -std::sin(M_PI / 9), -std::cos(M_PI / 9))), "roll 20: prumo");
    Check(Near(g::AzimuthElevationDeg(tr).Y(), -70.0, 1e-9), "roll 20: elevacao -70 no corpo");
    Check(Near(g::AzimuthElevationDeg(tr).X(), -90.0, 1e-9), "roll 20: azimute -90 (direita)");
    // ida e volta
    Quaterniond q(0.3, -0.2, 1.1);
    Vector3d v = Vector3d(0.3, -0.4, -0.866).Normalized();
    Check(NearV(q.RotateVector(g::WorldToBody(q, v)), v), "ida e volta world->body->world");
  }

  // roll/pitch/yaw.
  {
    Quaterniond q(0.1, -0.2, 0.3);
    Vector3d rpy = g::RollPitchYawDeg(q);
    Check(Near(rpy.X(), 0.1 * 180 / M_PI, 1e-9) && Near(rpy.Y(), -0.2 * 180 / M_PI, 1e-9) &&
          Near(rpy.Z(), 0.3 * 180 / M_PI, 1e-9), "rpy");
  }

  if (failures == 0)
    std::printf("OK\n");
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
