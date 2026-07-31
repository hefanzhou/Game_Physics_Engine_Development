/*
 * 卡丁车（raycast wheel）模块实现。
 */
#include <cyclone/vehicle_kart.h>
#include <float.h>
#include <math.h>
#include <stdio.h>

// 临时调试开关：所有日志都打到 stdout，用 `> log.txt 2>&1` 重定向。
#define KART_LOG 1
#if KART_LOG
  #include <stdarg.h>
static int   g_kartFrame = 0;
static const int KART_VERBOSE_FRAMES = 100000; // 全程详细日志（调试）
  static void klog(const char *fmt, ...)
  {
      static bool inited = false;
      if (!inited) { inited = true; setvbuf(stdout, NULL, _IONBF, 0); }
      va_list ap; va_start(ap, fmt); vprintf(fmt, ap); va_end(ap);
      putchar('\n');
  }
  static bool kart_verbose() { return g_kartFrame < KART_VERBOSE_FRAMES; }
#endif

using namespace cyclone;
using namespace cyclone::kart;

/* ========================================================================= */
/* RaycastWheel                                                              */
/* ========================================================================= */

RaycastWheel::RaycastWheel()
    : anchorLocal(0, 0, 0),
      restLength((real)0.4),
      maxTravel((real)0.25),
      radius((real)0.35),
      isDrive(false),
      isSteer(false),
      grounded(false),
      hitDistance(0),
      hitPoint(0, 0, 0),
      hitNormal(0, 1, 0),
      compression(0),
      steerAngle(0),
      rollAngle(0),
      wheelForward(0, 0, 1),
      wheelRight(1, 0, 0)
{
}

/* ========================================================================= */
/* KartVehicle                                                               */
/* ========================================================================= */

KartVehicle::KartVehicle()
    : chassis(NULL),
      // 默认参数（上层 demo 通常会覆盖）
      suspensionStiffness((real)35000),
      suspensionDamping((real)4500),
      maxNormalForce((real)100000),
      driveForcePerWheel((real)6000),
      reverseForcePerWheel((real)3500),
      brakeForcePerWheel((real)8000),
      maxForwardSpeed((real)28),
      maxReverseSpeed((real)14),
      rollingResistance((real)0.6),
      gripFront((real)18),
      gripRear((real)14),
      driftGripScale((real)0.35),
      maxLateralImpulse((real)15000),
      steerMaxRad((real)(0.55)),    // ~31°
      highSpeedRef((real)25),
      uprightP((real)8000),
      uprightD((real)1500),
      currentRearGripScale((real)1.0)
{
}

Vector3 KartVehicle::getPointVelocityWorld(const Vector3 &worldPoint) const
{
    if (!chassis) return Vector3(0, 0, 0);
    // v_p = v_cm + ω × (p - cm)
    Vector3 r = worldPoint - chassis->getPosition();
    Vector3 angVel = chassis->getRotation();
    Vector3 linVel = chassis->getVelocity();
    Vector3 v = linVel + (angVel % r);
    return v;
}

void KartVehicle::updateRaycasts(const CollisionPlane *ground,
                                 const CollisionBox  * const *boxes,
                                 unsigned boxCount)
{
    if (!chassis) return;

    // 车身本地 -Y 方向（在世界中），即射线方向
    Matrix4 tm = chassis->getTransform();
    // 车身本地坐标轴（世界方向）
    Vector3 bodyY(tm.data[1], tm.data[5], tm.data[9]);
    bodyY.normalise();
    Vector3 rayDir = bodyY * (real)-1;

    for (int i = 0; i < 4; ++i)
    {
        RaycastWheel &w = wheels[i];

        // 挂点世界坐标
        Vector3 anchorWorld = tm.transform(w.anchorLocal);
        real maxDist = w.restLength + w.maxTravel + w.radius;

        real bestDist = maxDist + (real)1.0;
        Vector3 bestNormal(0, 1, 0);
        bool hitAny = false;

        // 1) 半空间地面
        if (ground)
        {
            real d; Vector3 n;
            if (raycastPlane(anchorWorld, rayDir, maxDist,
                             ground->direction, ground->offset, d, n))
            {
                if (d < bestDist) { bestDist = d; bestNormal = n; hitAny = true; }
            }
        }

        // 2) 障碍 box
        for (unsigned bi = 0; bi < boxCount; ++bi)
        {
            const CollisionBox *bx = boxes[bi];
            if (!bx) continue;
            if (chassis && bx->body == chassis) continue; // 不与自身相交
            real d; Vector3 n;
            if (raycastBox(anchorWorld, rayDir, maxDist, *bx, d, n))
            {
                if (d < bestDist) { bestDist = d; bestNormal = n; hitAny = true; }
            }
        }

        if (hitAny)
        {
            w.grounded = true;
            w.hitDistance = bestDist;
            w.hitPoint = anchorWorld + rayDir * bestDist;
            w.hitNormal = bestNormal;
            real comp = w.restLength - (bestDist - w.radius);
            if (comp < 0) comp = 0;
            if (comp > w.maxTravel) comp = w.maxTravel;
            w.compression = comp;
        }
        else
        {
            w.grounded = false;
            w.hitDistance = 0;
            w.compression = 0;
        }
    }
}

void KartVehicle::applySuspension()
{
    if (!chassis) return;

    Matrix4 tm = chassis->getTransform();
    Vector3 bodyY(tm.data[1], tm.data[5], tm.data[9]);
    bodyY.normalise();

#if KART_LOG
    real dbgFn[4]    = {0,0,0,0};
    real dbgVcomp[4] = {0,0,0,0};
#endif

    for (int i = 0; i < 4; ++i)
    {
        RaycastWheel &w = wheels[i];
        if (!w.grounded) continue;
        if (w.compression <= 0) continue;

        // 挂点世界坐标 + 该点速度
        Vector3 anchorWorld = tm.transform(w.anchorLocal);
        Vector3 vAnchor = getPointVelocityWorld(anchorWorld);

        // v_compress：挂点向上速度（沿车身本地 +Y 投影），>0 表示弹簧正在被压
        // 因为悬挂被压缩等同于 anchor 相对 hit 在向 anchor 方向移动，
        // 等同于挂点世界速度沿 bodyY 投影为正 → 弹簧伸长方向被压缩。
        real vCompress = -(vAnchor * bodyY); // 取负：anchor 下沉（沿 -Y）→ vCompress 为正

        real Fn = suspensionStiffness * w.compression + suspensionDamping * vCompress;
        if (Fn < 0) Fn = 0;
        if (Fn > maxNormalForce) Fn = maxNormalForce;

        // 力沿命中法线方向
        Vector3 force = w.hitNormal * Fn;
        chassis->addForceAtBodyPoint(force, w.anchorLocal);

#if KART_LOG
        dbgFn[i] = Fn;
        dbgVcomp[i] = vCompress;
#endif
    }

#if KART_LOG
    if (kart_verbose())
    {
        klog("[susp] comp=(%5.3f %5.3f %5.3f %5.3f) vC=(%+5.2f %+5.2f %+5.2f %+5.2f) Fn=(%+8.0f %+8.0f %+8.0f %+8.0f)",
             (double)wheels[0].compression,(double)wheels[1].compression,(double)wheels[2].compression,(double)wheels[3].compression,
             (double)dbgVcomp[0],(double)dbgVcomp[1],(double)dbgVcomp[2],(double)dbgVcomp[3],
             (double)dbgFn[0],(double)dbgFn[1],(double)dbgFn[2],(double)dbgFn[3]);
    }
#endif
}

void KartVehicle::applyLongitudinal(const KartInput &input, real /*dt*/)
{
    if (!chassis) return;

    // 当前车身前向（车身本地 +X 在世界中）
    // 修正：之前错误地取了 tm.data[2/6/10]（本地 Z 轴，=车右方向）
    Matrix4 tm = chassis->getTransform();
    Vector3 bodyForward(tm.data[0], tm.data[4], tm.data[8]);
    bodyForward.normalise();
    Vector3 vCM = chassis->getVelocity();
    real fwdSpeed = vCM * bodyForward;

    for (int i = 0; i < 4; ++i)
    {
        RaycastWheel &w = wheels[i];
        if (!w.grounded) continue;

        Vector3 fwd = w.wheelForward;
        // 把 fwd 投影到地面切平面（去掉沿 hitNormal 的分量）
        fwd = fwd - w.hitNormal * (fwd * w.hitNormal);
        if (fwd.squareMagnitude() < (real)1e-8) continue;
        fwd.normalise();

        // 1) 驱动 / 倒车
        if (w.isDrive)
        {
            if (input.throttle && !input.reverse)
            {
                if (fwdSpeed < maxForwardSpeed)
                {
                    Vector3 F = fwd * driveForcePerWheel;
                    chassis->addForceAtPoint(F, w.hitPoint);
#if KART_LOG
                    if (kart_verbose())
                        klog("[long] w%d DRIVE fwd=(%+5.2f,%+5.2f,%+5.2f) F=%.0f at(%+5.2f,%+5.2f,%+5.2f)",
                             i,(double)fwd.x,(double)fwd.y,(double)fwd.z,
                             (double)driveForcePerWheel,
                             (double)w.hitPoint.x,(double)w.hitPoint.y,(double)w.hitPoint.z);
#endif
                }
            }
            else if (input.reverse && !input.throttle)
            {
                if (fwdSpeed > -maxReverseSpeed)
                {
                    Vector3 F = fwd * (-reverseForcePerWheel);
                    chassis->addForceAtPoint(F, w.hitPoint);
                }
            }
        }

        // 2) 刹车（每个触地轮）
        if (input.brake)
        {
            // 沿轮子前向速度反向施加
            Vector3 vp = getPointVelocityWorld(w.hitPoint);
            real vLong = vp * fwd;
            if (real_abs(vLong) > (real)0.05)
            {
                real sgn = (vLong > 0) ? (real)1 : (real)-1;
                Vector3 F = fwd * (-sgn * brakeForcePerWheel);
                chassis->addForceAtPoint(F, w.hitPoint);
            }
        }
        else if (!input.throttle && !input.reverse)
        {
            // 3) 滚动阻力（无油门、无倒车、无刹车）
            Vector3 vp = getPointVelocityWorld(w.hitPoint);
            real vLong = vp * fwd;
            // 单位质量假设（此处直接给一个力系数，参数化在外层调）
            Vector3 F = fwd * (-rollingResistance * vLong * (real)100); // 粗略近似 mass_per_wheel
            chassis->addForceAtPoint(F, w.hitPoint);
        }
    }
}

void KartVehicle::applyLateral(const KartInput &input, real dt)
{
    if (!chassis) return;
    if (dt <= 0) return;

    // 车身质心世界高度（用于抬高侧向力作用点，抑制转弯时的滚转力矩）
    real chassisCmY = chassis->getPosition().y;

#if KART_LOG
    real dbgFy[4] = {0,0,0,0};
    real dbgVlat[4] = {0,0,0,0};
#endif

    // 漂移：后轮抓地缩放在外层平滑（这里只读 currentRearGripScale）
    for (int i = 0; i < 4; ++i)
    {
        RaycastWheel &w = wheels[i];
        if (!w.grounded) continue;

        Vector3 right = w.wheelRight;
        // 投影到地面切平面
        right = right - w.hitNormal * (right * w.hitNormal);
        if (right.squareMagnitude() < (real)1e-8) continue;
        right.normalise();

        Vector3 vp = getPointVelocityWorld(w.hitPoint);
        real vLat = vp * right;

        // grip 在新模型里是 0..1 之间的"本帧侧滑消除比例"。
        // 1.0 = 完全消除侧向速度（理想抓地），0.0 = 不消除（自由滑行）。
        // 本来 gripFront/gripRear 设的是 ~15 的力系数量级，这里映射到 [0,1]：
        //   gripFactor = 1 - exp(-grip * dt)  （隐式离散，绝对稳定）
        real gripCoef = w.isSteer ? gripFront : (gripRear * currentRearGripScale);
        real gripFactor = (real)1 - real_exp(-gripCoef * dt);
        if (gripFactor < 0) gripFactor = 0;
        if (gripFactor > (real)1) gripFactor = (real)1;

        // 想要的速度变化：dv = -gripFactor * vLat
        // 力：F = m_per_wheel * dv / dt = -m * gripFactor * vLat / dt
        // m_per_wheel ≈ chassisMass / 4
        real massPerWheel = (real)100; // 默认值（兜底）
        if (chassis->getInverseMass() > 0)
            massPerWheel = ((real)1 / chassis->getInverseMass()) * (real)0.25;

        real Fmag = -massPerWheel * gripFactor * vLat / dt;
        // 单帧钳位（防极端情况，例如刚出生瞬间巨大 vLat）
        if (Fmag > maxLateralImpulse) Fmag = maxLateralImpulse;
        if (Fmag < -maxLateralImpulse) Fmag = -maxLateralImpulse;

        Vector3 F = right * Fmag;

        // 防翻车关键：侧向力作用点抬到质心高度。
        // 原因：如果把侧向力施在 hitPoint（地面上），力臂 ≈ 车身高度，
            //   会产生很大的绕车身前向轴的滚转力矩 → 转向立马翻车。
        // 同 btRaycastVehicle 的处理一样，昨为抬高作用点。
        Vector3 applyAt = w.hitPoint;
        applyAt.y = chassisCmY;
        chassis->addForceAtPoint(F, applyAt);

#if KART_LOG
        dbgFy[i] = Fmag;
        dbgVlat[i] = vLat;
#endif
    }

#if KART_LOG
    if (kart_verbose())
    {
        klog("[lat]  vLat=(%+6.2f %+6.2f %+6.2f %+6.2f) F=(%+8.0f %+8.0f %+8.0f %+8.0f) rearGrip=%.2f cmY=%.3f",
             (double)dbgVlat[0],(double)dbgVlat[1],(double)dbgVlat[2],(double)dbgVlat[3],
             (double)dbgFy[0],(double)dbgFy[1],(double)dbgFy[2],(double)dbgFy[3],
             (double)currentRearGripScale,(double)chassisCmY);
    }
    g_kartFrame++;
#endif

    // 抓地恢复 / 进入漂移
    real targetScale = input.drift ? driftGripScale : (real)1.0;
    real diff = targetScale - currentRearGripScale;
    real maxStep = dt / (real)0.45; // 约 0.45 秒平滑
    if (diff >  maxStep) diff =  maxStep;
    if (diff < -maxStep) diff = -maxStep;
    currentRearGripScale += diff;
}

void KartVehicle::applyStabilization()
{
    if (!chassis) return;

    // 4 轮触地状态
    int groundedCount = 0;
    for (int i = 0; i < 4; ++i) if (wheels[i].grounded) ++groundedCount;
    if (groundedCount < 3) return; // 飞行中不强制回正

    Matrix4 tm = chassis->getTransform();
    Vector3 bodyUp(tm.data[1], tm.data[5], tm.data[9]);
    bodyUp.normalise();
    Vector3 worldUp(0, 1, 0);

    // 让 bodyUp 朝 worldUp 收敛：扭矩 T = k_p * (bodyUp × worldUp) - k_d * angVel_horizontal
    Vector3 axis = bodyUp % worldUp;
    Vector3 angVel = chassis->getRotation();

    // 仅对水平分量（绕 X、Z）施加阻尼，保留 Y 方向（偏航）由玩家/侧向力控制
    Vector3 angVelHoriz = angVel;
    angVelHoriz.y = 0;

    Vector3 torque = axis * uprightP - angVelHoriz * uprightD;
    chassis->addTorque(torque);

#if KART_LOG
    if (kart_verbose())
    {
        klog("[stab] gnd=%d bodyUp=(%+5.2f,%+5.2f,%+5.2f) axis=(%+5.2f,%+5.2f,%+5.2f) torque=(%+8.0f,%+8.0f,%+8.0f)",
             groundedCount,
             (double)bodyUp.x,(double)bodyUp.y,(double)bodyUp.z,
             (double)axis.x,(double)axis.y,(double)axis.z,
             (double)torque.x,(double)torque.y,(double)torque.z);
    }
#endif
}

/* ========================================================================= */
/* raycast 工具函数                                                          */
/* ========================================================================= */

bool cyclone::kart::raycastPlane(const Vector3 &origin,
                                 const Vector3 &dir,
                                 real maxDist,
                                 const Vector3 &planeNormal,
                                 real planeOffset,
                                 real &outDist,
                                 Vector3 &outNormal)
{
    real denom = dir * planeNormal;
    real signedDist = (origin * planeNormal) - planeOffset;

    // 起点已在平面背面（已穿透）→ 立即命中（dist=0）
    if (signedDist <= 0)
    {
        outDist = 0;
        outNormal = planeNormal;
        return true;
    }

    // 与平面平行（或同向）→ 不会命中
    if (denom >= -((real)real_epsilon)) return false;

    real t = -signedDist / denom;
    if (t < 0) return false;
    if (t > maxDist) return false;

    outDist = t;
    outNormal = planeNormal;
    return true;
}

bool cyclone::kart::raycastBox(const Vector3 &origin,
                               const Vector3 &dir,
                               real maxDist,
                               const CollisionBox &box,
                               real &outDist,
                               Vector3 &outNormal)
{
    // 把射线从世界变换到 box 局部空间
    const Matrix4 &T = box.getTransform(); // box 局部 → 世界

    // T 的逆只需要 transformInverse / transformInverseDirection
    Vector3 localOrigin = T.transformInverse(origin);
    Vector3 localDir    = T.transformInverseDirection(dir);

    // 在局部坐标里 box 是 [-half, +half] 的 AABB
    real tNear = -REAL_MAX;
    real tFar  =  REAL_MAX;
    int  nearAxis = 0;
    real nearSign = 1;

    bool insideAll = true;

    for (int axis = 0; axis < 3; ++axis)
    {
        real ro = localOrigin[axis];
        real rd = localDir[axis];
        real h  = box.halfSize[axis];

        if (ro < -h || ro > h) insideAll = false;

        if (real_abs(rd) < (real)real_epsilon)
        {
            // 与该轴平行 → 必须在 slab 内，否则 miss
            if (ro < -h || ro > h) return false;
            continue;
        }
        real inv = (real)1.0 / rd;
        real t1 = (-h - ro) * inv;
        real t2 = ( h - ro) * inv;
        real signNear = -1; // 命中"-h"面 → 局部法线 -axis
        if (t1 > t2)
        {
            real tmp = t1; t1 = t2; t2 = tmp;
            signNear = 1;
        }
        if (t1 > tNear)
        {
            tNear = t1;
            nearAxis = axis;
            nearSign = signNear;
        }
        if (t2 < tFar) tFar = t2;
        if (tNear > tFar) return false;
        if (tFar < 0) return false;
    }

    // 起点在 box 内部 → 立即命中（dist=0），法线沿距离最近的面
    if (insideAll)
    {
        outDist = 0;
        // 取局部 +Y 面外法线作为兜底
        Vector3 n(0, 1, 0);
        outNormal = T.transformDirection(n);
        outNormal.normalise();
        return true;
    }

    if (tNear < 0) return false;
    if (tNear > maxDist) return false;

    outDist = tNear;

    // 局部命中面外法线
    Vector3 localN(0, 0, 0);
    localN[nearAxis] = nearSign;
    outNormal = T.transformDirection(localN);
    outNormal.normalise();
    return true;
}
