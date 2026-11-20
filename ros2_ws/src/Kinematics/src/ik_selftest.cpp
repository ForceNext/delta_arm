/**
 * @file ik_selftest.cpp
 * @brief 逆解/正解自检（不接硬件、不碰共享内存）
 *
 * 用法：ros2 run Kinematics ik_selftest
 * 退出码 0 = 全部通过；非 0 = 有失败项。
 *
 * 检查内容：
 *   1. 参考点：θ 相同三个角 → z（向下为正）是否符合几何
 *   2. IK → FK 往返残差（工作空间内网格扫描）
 *   3. 对称性：x=y=0 时三个关节角必须相等
 *   4. 不可达拒绝：基座上方 / 太浅 的点必须返回 false
 *   5. 方向规律：压 1 号臂 → 平台往 −x 走（1 号臂的正对面）
 */

#include <cmath>
#include <cstdio>

#include "Kinematics/Kinematics_ik.hpp"

namespace
{
int g_fail = 0;

void check(bool ok, const char * what, const char * detail = "")
{
  if (ok) {
    std::printf("  [PASS] %s %s\n", what, detail);
  } else {
    std::printf("  [FAIL] %s %s\n", what, detail);
    ++g_fail;
  }
}
}  // namespace

int main()
{
  const delta_arm::DeltaKinematics ik(115.0, 150.0, 292.0, 40.0);
  double th[3] = {0, 0, 0};
  double xyz[3] = {0, 0, 0};
  char buf[256];

  std::printf("Delta 几何: d1=%.1f L1=%.1f L2=%.1f d2=%.1f mm\n", ik.d1(), ik.L1(), ik.L2(), ik.d2());

  // ---- 1. 参考点：θ 相同 → z（向下为正） ----
  std::printf("\n[1] 参考点（三个关节角相同 → 末端 z，向下为正）\n");
  struct Ref { double theta, z; };
  const Ref refs[] = {
    {-25.0, 138.512}, {0.0, 186.116}, {20.625, 250.000}, {35.0, 300.770}};
  for (const auto & r : refs) {
    double t[3] = {r.theta, r.theta, r.theta};
    const bool ok = ik.forward(t, xyz) && std::fabs(xyz[2] - r.z) < 0.01 &&
      std::fabs(xyz[0]) < 1e-6 && std::fabs(xyz[1]) < 1e-6;
    std::snprintf(buf, sizeof(buf), "(θ=%.3f° → z=%.3f，期望 %.3f)", r.theta, xyz[2], r.z);
    check(ok, "FK 参考点", buf);
  }

  // ---- 2. IK → FK 往返（工作空间内网格） ----
  std::printf("\n[2] IK → FK 往返残差（z∈[150,300]，x,y∈[-40,40]）\n");
  double worst = 0.0, worst_at[3] = {0, 0, 0};
  int n_ok = 0, n_try = 0;
  for (double x = -40; x <= 40.001; x += 20) {
    for (double y = -40; y <= 40.001; y += 20) {
      for (double z = 150; z <= 300.001; z += 30) {
        ++n_try;
        double t[3];
        if (!ik.inverse(x, y, z, t)) {
          continue;                       // 几何不可达，跳过
        }
        double back[3];
        if (!ik.forward(t, back)) {
          continue;
        }
        const double e = std::sqrt(
          (back[0] - x) * (back[0] - x) + (back[1] - y) * (back[1] - y) +
          (back[2] - z) * (back[2] - z));
        if (e > worst) {
          worst = e;
          worst_at[0] = x;
          worst_at[1] = y;
          worst_at[2] = z;
        }
        ++n_ok;
      }
    }
  }
  std::snprintf(
    buf, sizeof(buf), "(%d/%d 点可达，最大残差 %.2e mm @ (%.0f,%.0f,%.0f))", n_ok, n_try, worst,
    worst_at[0], worst_at[1], worst_at[2]);
  check(n_ok > 0 && worst < 1e-6, "往返自洽", buf);

  // ---- 3. 对称性 ----
  std::printf("\n[3] 对称性（x=y=0 时三关节角相等）\n");
  {
    const bool ok = ik.inverse(0, 0, 250, th) && std::fabs(th[0] - th[1]) < 1e-9 &&
      std::fabs(th[1] - th[2]) < 1e-9;
    std::snprintf(buf, sizeof(buf), "θ=(%.6f, %.6f, %.6f)", th[0], th[1], th[2]);
    check(ok, "中心点对称", buf);
  }

  // ---- 4. 不可达必须拒绝 ----
  std::printf("\n[4] 不可达/假解必须拒绝\n");
  check(!ik.inverse(0, 0, 100, th), "太浅 (z=100mm) 拒绝");
  check(!ik.inverse(0, 0, 120, th), "太浅 (z=120mm) 拒绝");
  check(!ik.inverse(0, 0, -200, th), "基座上方 (z=-200mm) 拒绝");
  // 注意：几何最深约 435.5mm（θ≈90°），所以 400mm 在"纯几何"上是可达的；
  // 它该由节点的 z 守卫(z_max=300)或关节限幅(35°)拦下，不属于逆解类的职责。
  check(!ik.inverse(0, 0, 460, th), "超出几何极限 (z=460mm) 拒绝");
  check(ik.inverse(0, 0, 400, th), "几何可达但不该下发 (z=400mm) 逆解返回 true");
  check(ik.inverse(0, 0, 250, th), "中心 (z=250mm) 接受");

  // ---- 5. 方向规律 ----
  std::printf("\n[5] 方向规律（压一条臂 → 平台往该臂正对面走）\n");
  {
    const double cases[3][3] = {{10, 0, 0}, {0, 10, 0}, {0, 0, 10}};
    for (int i = 0; i < 3; ++i) {
      double f[3];
      ik.forward(cases[i], f);
      // 臂 i 的方位角 φ=i*120°，平台应往 φ+180° 方向移动
      const double phi = i * 120.0 * M_PI / 180.0;
      const double proj = f[0] * std::cos(phi) + f[1] * std::sin(phi);
      std::snprintf(
        buf, sizeof(buf), "压 %d 号臂 → 平台沿该臂方向的分量 %.3f mm（应为负）", i + 1, proj);
      check(proj < -1.0, "方向规律", buf);
    }
  }

  std::printf("\n%s（共 %d 项失败）\n", g_fail == 0 ? "全部通过 ✓" : "存在失败 ✗", g_fail);
  return g_fail == 0 ? 0 : 1;
}
