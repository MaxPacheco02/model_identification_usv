// Offline fit of the 3-DOF model (model3dof.hpp) from the logs of id_experiment.
//
//   ros2 run model_identification_usv id_fit --ros-args --params-file <yaml>
//     -p logs:="['/path/run.csv']" -p output:=/path/result.yaml   (one line)
//
// 1. Steady points (end of each "hold" segment that reached steady state): damping per axis by
//    linear least squares, tau - C(nu) nu = D(nu) nu, with three forms (linear + quadratic,
//    linear only, quadratic only) so that the residuals show which form holds.
//      surge: equal thrust;  yaw: opposite thrust;  sway: external lateral force, heading held
//      (the differential thrust needed to hold it gives the yaw moment due to sway);
//      turning circles: the cross term d_vr.
// 2. Transients (step response of each "hold" and decay of each "release"): inertia per axis (rigid
//    body + added mass) jointly with the damping, and the damping alone with the rigid-body inertia,
//    which covers the low speeds the steady tests do not reach.
// 3. Validation on runs not used in the fit (multisine, zig-zag): simulation of the identified
//    model with the logged inputs, fit per state (normalised RMSE, percent).

#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "model_identification_usv/model3dof.hpp"

using model_identification_usv::Model3Dof;
using model_identification_usv::Vec3;

namespace
{

struct Row
{
  double t;
  std::string block;
  int step;
  std::string phase;
  double x, y, psi;
  Vec3 nu, tau;
};

struct Seg
{
  std::string block;
  int step;
  std::string phase;
  double t0, t1;
  bool steady;
  int file;
};

struct Fit
{
  Eigen::VectorXd c, se;  // coefficients and standard errors
  double rms{0};
  int n{0};
};

// ordinary least squares y = X c, with standard errors from the residual variance
Fit ols(const std::vector<std::vector<double>> & X, const std::vector<double> & y)
{
  Fit f;
  const int n = static_cast<int>(y.size());
  if (n == 0) {
    return f;
  }
  const int p = static_cast<int>(X[0].size());
  Eigen::MatrixXd A(n, p);
  Eigen::VectorXd b(n);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < p; ++j) {
      A(i, j) = X[i][j];
    }
    b(i) = y[i];
  }
  f.n = n;
  f.c = A.colPivHouseholderQr().solve(b);
  const Eigen::VectorXd res = b - A * f.c;
  f.rms = std::sqrt(res.squaredNorm() / n);
  f.se = Eigen::VectorXd::Zero(p);
  if (n > p) {
    const double s2 = res.squaredNorm() / (n - p);
    const Eigen::MatrixXd cov = s2 * (A.transpose() * A).inverse();
    for (int j = 0; j < p; ++j) {
      f.se(j) = std::sqrt(std::max(0.0, cov(j, j)));
    }
  }
  return f;
}

std::vector<std::string> split(const std::string & s)
{
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    out.push_back(tok);
  }
  return out;
}

double nrmse_fit(const std::vector<double> & y, const std::vector<double> & yh)
{
  double m = 0;
  for (double v : y) {m += v;}
  m /= std::max<size_t>(1, y.size());
  double e = 0, d = 0;
  for (size_t i = 0; i < y.size(); ++i) {
    e += (y[i] - yh[i]) * (y[i] - yh[i]);
    d += (y[i] - m) * (y[i] - m);
  }
  return 100.0 * (1.0 - std::sqrt(e) / std::max(1e-12, std::sqrt(d)));
}

}  // namespace

class FitNode : public rclcpp::Node
{
public:
  FitNode()
  : Node("id_fit")
  {
    logs_ = declare_parameter<std::vector<std::string>>("logs", std::vector<std::string>{});
    output_ = declare_parameter<std::string>("output", "/tmp/model_identification_usv/fit.yaml");
    const double m = declare_parameter<double>("mass", 48.4);
    const double xg = declare_parameter<double>("x_g", 0.162);
    const double iz = declare_parameter<double>("iz_origin", 12.59);
    fit_mass_ = declare_parameter<bool>("fit_mass", true);
    steady_avg_ = declare_parameter<double>("steady_average", 5.0);
    window_ = declare_parameter<double>("integral_window", 0.3);
    min_accel_ = declare_parameter<std::vector<double>>("decay_min_accel", {0.002, 0.002, 0.005});
    val_blocks_ = declare_parameter<std::vector<std::string>>(
      "validation_blocks", {"multisine", "zigzag"});
    model_ = Model3Dof::rigid_body(m, xg, iz);
    rigid_ = model_;
  }

  int run()
  {
    if (logs_.empty()) {
      RCLCPP_ERROR(get_logger(), "parameter 'logs' is empty");
      return 1;
    }
    for (size_t k = 0; k < logs_.size(); ++k) {
      if (!load(logs_[k], static_cast<int>(k))) {
        return 1;
      }
    }
    out_ << "# id_fit result. Damping in the form D(nu) nu (model3dof.hpp); Fossen's coefficients are"
      " the negatives\n# (X_u = -d_u1, X_|u|u = -d_u2, Y_v = -d_v1, ...). se = standard error.\n";
    out_ << "logs: [";
    for (size_t k = 0; k < logs_.size(); ++k) {out_ << (k ? ", " : "") << logs_[k];}
    out_ << "]\nrigid_body: {m11: " << rigid_.m11 << ", m22: " << rigid_.m22 << ", m23: " <<
      rigid_.m23 << ", m33: " << rigid_.m33 << "}\n";

    steady_fits();
    decay_fits();
    if (fit_mass_) {
      steady_fits(true);  // C(nu) nu with the identified inertia
    }
    write_model();
    validate();

    std::ofstream f(output_);
    f << out_.str();
    RCLCPP_INFO(get_logger(), "written %s", output_.c_str());
    std::printf("%s", out_.str().c_str());
    return 0;
  }

private:
  bool load(const std::string & path, int file)
  {
    std::ifstream in(path);
    if (!in) {
      RCLCPP_ERROR(get_logger(), "cannot open %s", path.c_str());
      return false;
    }
    std::string line;
    std::getline(in, line);
    std::vector<Row> rows;
    while (std::getline(in, line)) {
      const auto c = split(line);
      if (c.size() < 17) {continue;}
      Row r;
      r.t = std::stod(c[0]);
      r.block = c[1];
      r.step = std::stoi(c[2]);
      r.phase = c[3];
      r.x = std::stod(c[4]);
      r.y = std::stod(c[5]);
      r.psi = std::stod(c[6]);
      r.nu = {std::stod(c[7]), std::stod(c[8]), std::stod(c[9])};
      r.tau = {std::stod(c[14]), std::stod(c[15]), std::stod(c[16])};
      rows.push_back(r);
    }
    rows_.push_back(rows);

    std::string sp = path.substr(0, path.size() - 4) + "_segments.csv";
    std::ifstream sin(sp);
    if (!sin) {
      RCLCPP_ERROR(get_logger(), "cannot open %s", sp.c_str());
      return false;
    }
    std::getline(sin, line);
    while (std::getline(sin, line)) {
      const auto c = split(line);
      if (c.size() < 6) {continue;}
      segs_.push_back(
        {c[0], std::stoi(c[1]), c[2], std::stod(c[3]), std::stod(c[4]), c[5] == "1", file});
    }
    RCLCPP_INFO(get_logger(), "%s: %zu rows", path.c_str(), rows.size());
    return true;
  }

  // rows of a segment (by time range of its file)
  std::vector<const Row *> rows_of(const Seg & s, double from = -1e18) const
  {
    std::vector<const Row *> out;
    for (const auto & r : rows_[s.file]) {
      if (r.t >= std::max(s.t0, from) && r.t <= s.t1 && r.block == s.block && r.step == s.step &&
        r.phase == s.phase)
      {
        out.push_back(&r);
      }
    }
    return out;
  }

  struct Point
  {
    std::string block;
    int step;
    Vec3 nu, tau;
  };

  std::vector<Point> steady_points() const
  {
    std::vector<Point> pts;
    for (const auto & s : segs_) {
      if (!s.steady || s.phase != "hold") {continue;}
      const auto rs = rows_of(s, s.t1 - steady_avg_);
      if (rs.empty()) {continue;}
      Point p{s.block, s.step, {0, 0, 0}, {0, 0, 0}};
      for (const auto * r : rs) {
        for (int i = 0; i < 3; ++i) {
          p.nu[i] += r->nu[i] / rs.size();
          p.tau[i] += r->tau[i] / rs.size();
        }
      }
      pts.push_back(p);
    }
    return pts;
  }

  void report(
    const std::string & name, const std::vector<std::string> & coef, const Fit & f,
    const std::string & indent = "  ")
  {
    out_ << indent << name << ": {n: " << f.n << ", rms: " << f.rms;
    for (int j = 0; j < f.c.size(); ++j) {
      out_ << ", " << coef[j] << ": " << f.c(j) << ", " << coef[j] << "_se: " << f.se(j);
    }
    out_ << "}\n";
  }

  // damping of one axis from points, three forms; returns the linear + quadratic fit
  Fit axis_fit(
    const std::string & label, const std::vector<double> & x, const std::vector<double> & y,
    const std::string & c1, const std::string & c2)
  {
    std::vector<std::vector<double>> Xlq, Xl, Xq;
    for (double v : x) {
      Xlq.push_back({v, std::abs(v) * v});
      Xl.push_back({v});
      Xq.push_back({std::abs(v) * v});
    }
    const Fit lq = ols(Xlq, y), l = ols(Xl, y), q = ols(Xq, y);
    out_ << "  " << label << ":\n";
    report("lin_quad", {c1, c2}, lq, "    ");
    report("lin_only", {c1}, l, "    ");
    report("quad_only", {c2}, q, "    ");
    return lq;
  }

  void steady_fits(bool second_pass = false)
  {
    const auto pts = steady_points();
    out_ << (second_pass ? "steady_refit_with_identified_inertia:\n" : "steady:\n");
    if (!second_pass) {
      out_ << "  points:\n";
      for (const auto & p : pts) {
        out_ << "    - {block: " << p.block << ", step: " << p.step << ", u: " << p.nu[0] <<
          ", v: " << p.nu[1] << ", r: " << p.nu[2] << ", tau_x: " << p.tau[0] << ", tau_y: " <<
          p.tau[1] << ", tau_n: " << p.tau[2] << "}\n";
      }
    }
    auto resid = [this](const Point & p, int i) {return p.tau[i] - model_.coriolis(p.nu)[i];};

    std::vector<double> x, y;
    for (const auto & p : pts) {if (p.block == "surge") {x.push_back(p.nu[0]); y.push_back(resid(p, 0));}}
    if (!x.empty()) {
      const Fit f = axis_fit("surge", x, y, "d_u1", "d_u2");
      model_.d_u1 = f.c(0);
      model_.d_u2 = f.c(1);
    }
    x.clear();
    y.clear();
    for (const auto & p : pts) {if (p.block == "yaw") {x.push_back(p.nu[2]); y.push_back(resid(p, 2));}}
    if (!x.empty()) {
      const Fit f = axis_fit("yaw", x, y, "d_r1", "d_r2");
      model_.d_r1 = f.c(0);
      model_.d_r2 = f.c(1);
    }
    x.clear();
    y.clear();
    std::vector<double> yn;
    for (const auto & p : pts) {
      if (p.block == "sway") {
        x.push_back(p.nu[1]);
        y.push_back(resid(p, 1));
        Model3Dof m0 = model_;
        m0.d_rv1 = m0.d_rv2 = 0;
        yn.push_back(p.tau[2] - model_.coriolis(p.nu)[2] - m0.damping(p.nu)[2]);
      }
    }
    if (!x.empty()) {
      const Fit f = axis_fit("sway", x, y, "d_v1", "d_v2");
      model_.d_v1 = f.c(0);
      model_.d_v2 = f.c(1);
      const Fit g = axis_fit("yaw_moment_from_sway", x, yn, "d_rv1", "d_rv2");
      model_.d_rv1 = g.c(0);
      model_.d_rv2 = g.c(1);
    }
    // turning circles: sway row residual against r
    std::vector<std::vector<double>> X;
    y.clear();
    std::vector<double> yres;
    for (const auto & p : pts) {
      if (p.block != "turn") {continue;}
      Model3Dof m0 = model_;
      m0.d_vr = 0;
      X.push_back({p.nu[2]});
      y.push_back(p.tau[1] - model_.coriolis(p.nu)[1] - m0.damping(p.nu)[1]);
    }
    if (!X.empty()) {
      const Fit f = ols(X, y);
      out_ << "  turn:\n";
      report("sway_cross", {"d_vr"}, f, "    ");
      model_.d_vr = f.c(0);
      for (const auto & p : pts) {
        if (p.block == "turn") {
          yres.push_back(p.tau[2] - model_.coriolis(p.nu)[2] - model_.damping(p.nu)[2]);
        }
      }
      double s = 0;
      for (double e : yres) {s += e * e;}
      out_ << "    yaw_residual_rms: " << std::sqrt(s / yres.size()) << "\n";
    }
  }

  void decay_fits()
  {
    // per axis, the equation of motion integrated over short windows [t_a, t_b] (no numerical
    // derivative, whose noise would bias the inertia towards zero):
    //   joint:  int(tau_i - [C nu]_i) - m23 * delta(other) =
    //           m * delta(nu_i) + d1 * int(nu_i) + d2 * int(|nu_i| nu_i)       (m, d1, d2)
    //   damping_rigid_inertia: the same with m fixed to the rigid-body value (d1, d2)
    // other = r for sway, v for yaw; rigid-body m23. Windows where nu_i changes by less than
    // decay_min_accel * window are dropped (no information on m).
    struct Acc
    {
      std::vector<std::vector<double>> X;
      std::vector<double> y;
    };
    std::map<std::string, Acc> inertia, damp;
    for (const auto & s : segs_) {
      // a free decay only shows d/m; the step response at the start of a hold (known input)
      // separates the two, so both phases are used
      if ((s.phase != "release" && s.phase != "hold") ||
        (s.block != "surge" && s.block != "yaw" && s.block != "sway"))
      {
        continue;
      }
      const auto rs = rows_of(s);
      const int ax = s.block == "surge" ? 0 : (s.block == "sway" ? 1 : 2);
      const int other = ax == 1 ? 2 : 1;
      const double mi = ax == 0 ? rigid_.m11 : (ax == 1 ? rigid_.m22 : rigid_.m33);
      size_t i0 = 0;
      while (i0 + 1 < rs.size()) {
        // window from i0 to i1
        size_t i1 = i0 + 1;
        while (i1 + 1 < rs.size() && rs[i1]->t - rs[i0]->t < window_) {++i1;}
        const double T = rs[i1]->t - rs[i0]->t;
        if (T < 0.5 * window_) {break;}
        double iy = 0, i_nu = 0, i_nu2 = 0;
        for (size_t k = i0; k < i1; ++k) {
          const double dt = rs[k + 1]->t - rs[k]->t;
          const Row & a = *rs[k];
          const Row & b = *rs[k + 1];
          const double fa = a.tau[ax] - rigid_.coriolis(a.nu)[ax];
          const double fb = b.tau[ax] - rigid_.coriolis(b.nu)[ax];
          iy += 0.5 * dt * (fa + fb);
          i_nu += 0.5 * dt * (a.nu[ax] + b.nu[ax]);
          i_nu2 += 0.5 * dt * (std::abs(a.nu[ax]) * a.nu[ax] + std::abs(b.nu[ax]) * b.nu[ax]);
        }
        const double dnu = rs[i1]->nu[ax] - rs[i0]->nu[ax];
        double y = iy;
        if (ax != 0) {y -= rigid_.m23 * (rs[i1]->nu[other] - rs[i0]->nu[other]);}
        if (std::abs(dnu) >= min_accel_[ax] * T) {
          inertia[s.block].X.push_back({dnu, i_nu, i_nu2});
          inertia[s.block].y.push_back(y);
        }
        damp[s.block].X.push_back({i_nu, i_nu2});
        damp[s.block].y.push_back(y - mi * dnu);
        i0 = i1;
      }
    }
    out_ << "decay:\n";
    const std::map<std::string, std::vector<std::string>> names{
      {"surge", {"m11", "d_u1", "d_u2"}}, {"sway", {"m22", "d_v1", "d_v2"}},
      {"yaw", {"m33", "d_r1", "d_r2"}}};
    Model3Dof fitted = model_;
    for (const auto & [blk, nm] : names) {
      if (inertia[blk].y.empty()) {continue;}
      out_ << "  " << blk << ":\n";
      const Fit fi = ols(inertia[blk].X, inertia[blk].y);
      report("joint", {nm[0], nm[1], nm[2]}, fi, "    ");
      const Fit fd = ols(damp[blk].X, damp[blk].y);
      report("damping_rigid_inertia", {nm[1], nm[2]}, fd, "    ");
      if (blk == "surge") {fitted.m11 = fi.c(0);}
      if (blk == "sway") {fitted.m22 = fi.c(0);}
      if (blk == "yaw") {fitted.m33 = fi.c(0);}
    }
    if (fit_mass_) {
      model_.m11 = fitted.m11;
      model_.m22 = fitted.m22;
      model_.m33 = fitted.m33;
    }
  }

  void write_model()
  {
    const auto & p = model_;
    out_ << "model:\n  inertia: {m11: " << p.m11 << ", m22: " << p.m22 << ", m23: " << p.m23 <<
      ", m33: " << p.m33 << "}\n";
    out_ << "  added_mass_estimate: {X_udot: " << rigid_.m11 - p.m11 << ", Y_vdot: " <<
      rigid_.m22 - p.m22 << ", N_rdot: " << rigid_.m33 - p.m33 << "}\n";
    out_ << "  damping: {d_u1: " << p.d_u1 << ", d_u2: " << p.d_u2 << ", d_v1: " << p.d_v1 <<
      ", d_v2: " << p.d_v2 << ", d_vr: " << p.d_vr << ", d_r1: " << p.d_r1 << ", d_r2: " << p.d_r2 <<
      ", d_rv1: " << p.d_rv1 << ", d_rv2: " << p.d_rv2 << "}\n";
    out_ << "  fossen: {X_u: " << -p.d_u1 << ", X_uu: " << -p.d_u2 << ", Y_v: " << -p.d_v1 <<
      ", Y_vv: " << -p.d_v2 << ", Y_r: " << -p.d_vr << ", N_r: " << -p.d_r1 << ", N_rr: " <<
      -p.d_r2 << ", N_v: " << -p.d_rv1 << ", N_vv: " << -p.d_rv2 << "}\n";
  }

  void validate()
  {
    out_ << "validation:\n";
    for (const auto & s : segs_) {
      bool use = false;
      for (const auto & b : val_blocks_) {use = use || (s.block == b && s.phase == "run");}
      if (!use) {continue;}
      const auto rs = rows_of(s);
      if (rs.size() < 10) {continue;}
      Vec3 nu = rs[0]->nu;
      std::vector<double> y[3], yh[3];
      const std::string csv = output_.substr(0, output_.find_last_of('.')) + "_val_" + s.block +
        std::to_string(s.step) + "_" + std::to_string(s.file) + ".csv";
      std::ofstream f(csv);
      f << "t,u,v,r,u_hat,v_hat,r_hat\n";
      for (size_t i = 0; i < rs.size(); ++i) {
        if (i > 0) {nu = model_.step(nu, rs[i - 1]->tau, rs[i]->t - rs[i - 1]->t);}
        for (int k = 0; k < 3; ++k) {
          y[k].push_back(rs[i]->nu[k]);
          yh[k].push_back(nu[k]);
        }
        f << rs[i]->t - rs[0]->t << ',' << rs[i]->nu[0] << ',' << rs[i]->nu[1] << ',' <<
          rs[i]->nu[2] << ',' << nu[0] << ',' << nu[1] << ',' << nu[2] << '\n';
      }
      out_ << "  - {block: " << s.block << ", step: " << s.step << ", file: " << s.file <<
        ", fit_u: " << nrmse_fit(y[0], yh[0]) << ", fit_v: " << nrmse_fit(y[1], yh[1]) <<
        ", fit_r: " << nrmse_fit(y[2], yh[2]) << ", csv: " << csv << "}\n";
    }
  }

  std::vector<std::string> logs_, val_blocks_;
  std::string output_;
  bool fit_mass_;
  double steady_avg_;
  double window_;
  std::vector<double> min_accel_;
  Model3Dof model_, rigid_;
  std::vector<std::vector<Row>> rows_;
  std::vector<Seg> segs_;
  std::ostringstream out_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FitNode>();
  const int rc = node->run();
  rclcpp::shutdown();
  return rc;
}
