// wb_core must reproduce sim/isaaclab/scripts/wbctrl.py step by step (golden data from test/gen_golden.py).
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "gen2_control/wb_core.hpp"

using namespace gen2_control;

static std::vector<std::vector<double>> read_csv(const std::string & path, std::vector<std::string> * header)
{
  std::ifstream f(path);
  std::vector<std::vector<double>> rows;
  std::string line;
  bool first = header != nullptr;
  while (std::getline(f, line)) {
    std::stringstream ss(line);
    std::string cell;
    std::vector<double> r;
    std::vector<std::string> h;
    while (std::getline(ss, cell, ',')) {
      if (first) {h.push_back(cell);} else {r.push_back(std::stod(cell));}
    }
    if (first) {*header = h; first = false;} else {rows.push_back(r);}
  }
  return rows;
}

TEST(WBCore, MatchesPythonSpec)
{
  LqrTable lqr;
  for (const auto & r : read_csv(std::string(GOLDEN_DIR) + "/lqr.csv", nullptr)) {
    lqr.l.push_back(r[0]);
    lqr.K.push_back({r[1], r[2], r[3], r[4]});
  }
  std::vector<std::string> h;
  const auto rows = read_csv(std::string(GOLDEN_DIR) + "/golden.csv", &h);
  ASSERT_GT(rows.size(), 1000u);
  std::map<std::string, std::size_t> c;
  for (std::size_t i = 0; i < h.size(); ++i) {c[h[i]] = i;}
  WBCore core(Params{}, lqr);
  double worst = 0.0;
  std::size_t worst_k = 0;
  int lifted = 0;
  for (std::size_t k = 0; k < rows.size(); ++k) {
    const auto & r = rows[k];
    Frame f;
    f.t = r[c["t"]];
    f.g_b[0] = r[c["gx_b"]]; f.g_b[1] = r[c["gy_b"]]; f.g_b[2] = r[c["gz_b"]];
    f.w_b[0] = r[c["wx"]]; f.w_b[1] = r[c["wy"]]; f.w_b[2] = r[c["wz_b"]];
    f.h[0] = r[c["hL"]]; f.h[1] = r[c["hR"]];
    f.tau_hip[0] = r[c["tauL"]]; f.tau_hip[1] = r[c["tauR"]];
    f.w_wheel_joint[0] = r[c["wjL"]]; f.w_wheel_joint[1] = r[c["wjR"]];
    f.w_wheel_abs[0] = r[c["waL"]]; f.w_wheel_abs[1] = r[c["waR"]];
    f.th_kin = r[c["th_kin"]]; f.l_pend = r[c["l_pend"]]; f.sf = r[c["sf"]]; f.motor_scale = r[c["motor_scale"]];
    const Output o = core.step(f, r[c["vx"]], r[c["wzc"]], r[c["h_ref"]], r[c["h_mid"]]);
    const double exp_[] = {r[c["a0"]], r[c["a1"]], r[c["a2"]], r[c["a3"]], r[c["kp"]], r[c["kd"]], r[c["ff"]],
                           r[c["th"]], r[c["v"]], r[c["lifted"]]};
    const double got[] = {o.act[0], o.act[1], o.act[2], o.act[3], o.leg_kp, o.leg_kd, o.ff_force, o.th, o.v,
                          o.lifted ? 1.0 : 0.0};
    for (int j = 0; j < 10; ++j) {
      const double d = std::fabs(exp_[j] - got[j]);
      if (d > worst) {worst = d; worst_k = k;}
      ASSERT_NEAR(got[j], exp_[j], 1e-9) << "step " << k << " field " << j;
    }
    lifted += o.lifted;
  }
  EXPECT_GT(lifted, 50);   // the lift / landing path was exercised
  std::printf("max |diff| %.3g at step %zu, lifted steps %d\n", worst, worst_k, lifted);
}
