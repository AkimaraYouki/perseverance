"""바퀴 반지름만 바꾼 로봇 USD 사본: usd_loop -> usd_loop_R{mm} (cad.py 가 WB_WHEEL_R 로 고른다).

바꾸는 것 (같은 재질·폭의 바퀴를 키운다고 가정):
  * 충돌 원기둥 반지름 (URDF cylinder 0.06 m) 과 extent
  * 바퀴 질량 x (R/0.06)^2, 관성: 축 = 회전자 반사관성 1.625e-3 + 바퀴 몫 x (R/0.06)^4, 나머지 축 x (R/0.06)^4
  * 겉모습: 바퀴 visuals 를 R/0.06 배
다리 길이·링크는 그대로 (고관절 -> 바퀴 중심 거리는 같고, 땅에서 몸통까지만 반지름 차만큼 높아진다).

    isaaclab.sh -p _isaaclab_launch.py make_wheel_variant.py --r_mm 65 70 75
"""
import argparse
import os
import shutil

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--r_mm", type=int, nargs="+", default=[65, 70, 75])
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
args.headless = True
app = AppLauncher(args).app

from pxr import Gf, Sdf, Usd, UsdGeom  # noqa: E402

SRC = os.path.expanduser("~/wheeled_biped_isaaclab/usd_loop")
R0, I_ROTOR = 0.060, 1.625e-3


def is_wheel(path: Sdf.Path) -> bool:
    """바퀴 링크(l_wheel, r_wheel) 아래만. 루트 이름 wheeled_biped_loop 에도 'wheel' 이 들어 있어 이름 포함으로 고르면 안 된다."""
    return any(e.name in ("l_wheel", "r_wheel") for e in path.GetPrimPath().GetPrefixes())


for r_mm in args.r_mm:
    R = r_mm / 1000.0
    k = R / R0
    dst = f"{SRC}_R{r_mm}"
    if os.path.exists(dst):
        shutil.rmtree(dst)
    shutil.copytree(SRC, dst)
    stage = Usd.Stage.Open(os.path.join(dst, "robot_simple.usd"))
    log = []
    for layer in stage.GetUsedLayers():
        if not layer.realPath or not layer.realPath.startswith(dst):
            continue
        hits = []
        layer.Traverse(Sdf.Path.absoluteRootPath, lambda p: hits.append(p) if p.IsPropertyPath() and is_wheel(p) else None)
        for p in hits:
            a = layer.GetAttributeAtPath(p)
            if a is None or a.default is None:
                continue
            name, prim = p.name, layer.GetPrimAtPath(p.GetPrimPath())
            if name == "radius" and prim is not None and prim.typeName == "Cylinder":
                a.default = R
                log.append(f"{p} radius -> {R}")
                ext = layer.GetAttributeAtPath(p.GetPrimPath().AppendProperty("extent"))
                if ext is not None and ext.default is not None:
                    lo, hi = ext.default[0], ext.default[1]          # 반지름 방향 성분(|v| = 0.06)만 키운다
                    new = [Gf.Vec3f(*[v * k if abs(abs(v) - R0) < 1e-4 else v for v in lo]),
                           Gf.Vec3f(*[v * k if abs(abs(v) - R0) < 1e-4 else v for v in hi])]
                    ext.default = new
                    log.append(f"{p.GetPrimPath()} extent -> {new}")
            elif name == "physics:mass":
                a.default = float(a.default) * k * k
                log.append(f"{p} -> {a.default:.4f} kg")
            elif name == "physics:diagonalInertia":
                d = list(a.default)
                ax = max(range(3), key=lambda i: d[i])
                new = [v * k ** 4 for v in d]
                new[ax] = I_ROTOR + (d[ax] - I_ROTOR) * k ** 4
                a.default = Gf.Vec3f(*new)
                log.append(f"{p} -> {[round(v, 7) for v in new]}")
        if hits:
            layer.Save()
    # 겉모습: 바퀴 링크 아래 visuals 를 k 배 (루트 레이어에 덮어쓰기)
    stage.SetEditTarget(stage.GetRootLayer())
    for prim in stage.Traverse(Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
        if prim.IsInstanceProxy() or prim.GetName() != "visuals" or prim.GetParent().GetName() not in ("l_wheel", "r_wheel"):
            continue
        xf = UsdGeom.Xformable(prim)
        ops = [o for o in xf.GetOrderedXformOps() if o.GetOpType() == UsdGeom.XformOp.TypeScale]
        op = ops[0] if ops else xf.AddScaleOp()
        s0 = op.Get() or Gf.Vec3f(1, 1, 1)
        op.Set(Gf.Vec3f(s0[0] * k, s0[1] * k, s0[2] * k))
        log.append(f"{prim.GetPath()} visual scale x{k:.3f}")
    stage.GetRootLayer().Save()
    print(f"\n[바퀴 {2 * r_mm} mm] {dst}", flush=True)
    for line in log:
        print("   ", line, flush=True)
app.close()
