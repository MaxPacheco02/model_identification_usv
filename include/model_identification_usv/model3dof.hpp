// 3-DOF (surge, sway, yaw) manoeuvring model of a USV, written about the body origin:
//
//   M nu_dot + C(nu) nu + D(nu) nu = tau,   nu = [u, v, r]
//
// M = [m11 0 0; 0 m22 m23; 0 m23 m33] (rigid body plus added mass, symmetric), C(nu) from M
// (Fossen 2011, eq. 6.43 for a symmetric M), and the damping
//
//   d_X = d_u1 u + d_u2 |u| u
//   d_Y = d_v1 v + d_v2 |v| v + d_vr r
//   d_N = d_r1 r + d_r2 |r| r + d_rv1 v + d_rv2 |v| v
//
// Fossen's coefficients are the negatives: X_u = -d_u1, X_|u|u = -d_u2, Y_v = -d_v1, ...
// The equations hold in any right-handed body frame (FLU as in Gazebo, or FRD as in Fossen); the
// diagonal and cross coefficients above do not change sign between the two.
#pragma once

#include <array>
#include <cmath>

namespace model_identification_usv
{

using Vec3 = std::array<double, 3>;

struct Model3Dof
{
  // inertia (about the body origin)
  double m11{0}, m22{0}, m23{0}, m33{0};
  // damping
  double d_u1{0}, d_u2{0};
  double d_v1{0}, d_v2{0}, d_vr{0};
  double d_r1{0}, d_r2{0}, d_rv1{0}, d_rv2{0};

  static Model3Dof rigid_body(double m, double x_g, double iz_origin)
  {
    Model3Dof p;
    p.m11 = m;
    p.m22 = m;
    p.m23 = m * x_g;
    p.m33 = iz_origin;
    return p;
  }

  // C(nu) nu
  Vec3 coriolis(const Vec3 & nu) const
  {
    const double u = nu[0], v = nu[1], r = nu[2];
    const double a = m22 * v + m23 * r;
    return {-a * r, m11 * u * r, a * u - m11 * u * v};
  }

  // D(nu) nu
  Vec3 damping(const Vec3 & nu) const
  {
    const double u = nu[0], v = nu[1], r = nu[2];
    return {
      d_u1 * u + d_u2 * std::abs(u) * u,
      d_v1 * v + d_v2 * std::abs(v) * v + d_vr * r,
      d_r1 * r + d_r2 * std::abs(r) * r + d_rv1 * v + d_rv2 * std::abs(v) * v};
  }

  // nu_dot = M^-1 (tau - C nu - D nu)
  Vec3 accel(const Vec3 & nu, const Vec3 & tau) const
  {
    const Vec3 c = coriolis(nu), d = damping(nu);
    const double f0 = tau[0] - c[0] - d[0];
    const double f1 = tau[1] - c[1] - d[1];
    const double f2 = tau[2] - c[2] - d[2];
    const double det = m22 * m33 - m23 * m23;
    return {f0 / m11, (m33 * f1 - m23 * f2) / det, (-m23 * f1 + m22 * f2) / det};
  }

  // one RK4 step of the velocity dynamics with tau held constant
  Vec3 step(const Vec3 & nu, const Vec3 & tau, double dt) const
  {
    auto add = [](const Vec3 & a, const Vec3 & b, double s) {
        return Vec3{a[0] + s * b[0], a[1] + s * b[1], a[2] + s * b[2]};
      };
    const Vec3 k1 = accel(nu, tau);
    const Vec3 k2 = accel(add(nu, k1, dt / 2), tau);
    const Vec3 k3 = accel(add(nu, k2, dt / 2), tau);
    const Vec3 k4 = accel(add(nu, k3, dt), tau);
    Vec3 out;
    for (int i = 0; i < 3; ++i) {
      out[i] = nu[i] + dt / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    }
    return out;
  }
};

// generalized force about the body origin from the two side thrusters (at y = +arm and -arm,
// thrust along +x) and an external body-frame force (fx, fy) at (px, py) plus a moment mz
inline Vec3 generalized_force(
  double t_left, double t_right, double arm, double fx, double fy, double mz, double px,
  double py)
{
  return {t_left + t_right + fx, fy, arm * (t_right - t_left) + px * fy - py * fx + mz};
}

}  // namespace model_identification_usv
