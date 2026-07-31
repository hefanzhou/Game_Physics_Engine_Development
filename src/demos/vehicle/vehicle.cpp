/*
 * 车辆 demo（卡丁车版本，方案 B：raycast wheel）。
 *
 * 与方案 A（单刚体 + 装饰轮）的关键区别：
 *   - 车身仍是 1 个 RigidBody + 1 个 CollisionBox；
 *   - 4 个车轮是 raycast 探测点（cyclone::kart::RaycastWheel），
 *     不是 RigidBody，不参与碰撞解算；
 *   - 每帧从挂点沿"车身本地 -Y"投射射线，根据命中距离：
 *       * 计算悬挂压缩量 → 沿命中法线施加弹簧 + 阻尼支撑力；
 *       * 在触地点施加纵向（驱动/制动/滚阻）与横向（侧向抓地/漂移）力；
 *   - 转向只改前轮 wheelForward / wheelRight，不直接控制偏航角速度；
 *     车辆能转弯靠"前轮侧向力 → 对质心的力矩"。
 */

#include <cyclone/cyclone.h>
#include <cyclone/vehicle_kart.h>
#include "../ogl_headers.h"
#include "../app.h"
#include "../timing.h"

#include <stdio.h>
#include <math.h>
#include <stdarg.h>

// ---- 翻车诊断日志开关 ----
#define VEHICLE_LOG 1
#if VEHICLE_LOG
static int   g_logFrame = 0;
static int   g_logVerboseFrames = 100000;  // 全程详细输出（调试用）
  static void log_open()
  {
      static bool inited = false;
      if (!inited)
      {
          inited = true;
          setvbuf(stdout, NULL, _IONBF, 0);
          printf("[boot] vehicle log opened (stdout)\n");
      }
  }
  static void log_line(const char *fmt, ...)
  {
      log_open();
      va_list ap; va_start(ap, fmt);
      vprintf(fmt, ap);
      va_end(ap);
      putchar('\n');
  }
#endif

// Win32 GetAsyncKeyState 用于实时查询键盘状态（GLUT keyboard 回调依赖按键重复）。
#ifdef _WIN32
  #define WIN32_LEAN_AND_MEAN
  #include <windows.h>
  static inline bool isKeyDown(int vk)
  {
      return (GetAsyncKeyState(vk) & 0x8000) != 0;
  }
#else
  static inline bool isKeyDown(int /*vk*/) { return false; }
#endif

// ===========================================================================
// 手感参数（开箱即玩；改这里就能调手感）
// ===========================================================================

// 车身（卡丁车风格：低 + 轻）
static const cyclone::real CHASSIS_HALF_X       = (cyclone::real)1.4;   // 长度方向半尺寸
static const cyclone::real CHASSIS_HALF_Y       = (cyclone::real)0.30;  // 高度方向半尺寸
static const cyclone::real CHASSIS_HALF_Z       = (cyclone::real)0.75;  // 宽度方向半尺寸
static const cyclone::real CHASSIS_MASS         = (cyclone::real)400.0;
static const cyclone::real CHASSIS_LIN_DAMPING  = (cyclone::real)0.998;
static const cyclone::real CHASSIS_ANG_DAMPING  = (cyclone::real)0.85;

// 悬挂 / 车轮
static const cyclone::real WHEEL_RADIUS         = (cyclone::real)0.30;
static const cyclone::real WHEEL_REST_LENGTH    = (cyclone::real)0.45;
static const cyclone::real WHEEL_MAX_TRAVEL     = (cyclone::real)0.30;
static const cyclone::real SUSP_STIFFNESS       = (cyclone::real)32000.0;  // k_s
static const cyclone::real SUSP_DAMPING         = (cyclone::real)3600.0;   // k_d
static const cyclone::real SUSP_FN_MAX_SCALE    = (cyclone::real)5.0;      // Fn ≤ m*g*5

// 纵向（驱动/倒车/刹车/滚阻）
//   起步加速度参考：F_total = 2 * 6500 = 13000 N，m=400kg → a≈32.5m/s²，
//   1.5 秒内可达 ~20 m/s（满足需求 13.1）。
static const cyclone::real DRIVE_FORCE_PER_WHEEL    = (cyclone::real)6500.0;
static const cyclone::real REVERSE_FORCE_PER_WHEEL  = (cyclone::real)4000.0;
static const cyclone::real BRAKE_FORCE_PER_WHEEL    = (cyclone::real)4500.0;
static const cyclone::real MAX_FORWARD_SPEED        = (cyclone::real)28.0;
static const cyclone::real MAX_REVERSE_SPEED        = (cyclone::real)14.0;
static const cyclone::real ROLLING_K                = (cyclone::real)0.6;

// 横向（侧向抓地 / 漂移）
//   gripFront > gripRear → 转向欠饱和、过弯顺；按 Shift 后后轮抓地骤降 → 甩尾
static const cyclone::real GRIP_FRONT          = (cyclone::real)22.0;
static const cyclone::real GRIP_REAR           = (cyclone::real)17.0;
static const cyclone::real DRIFT_GRIP_SCALE    = (cyclone::real)0.30;
static const cyclone::real MAX_LATERAL_FORCE   = (cyclone::real)25000.0;

// 转向
static const cyclone::real STEER_MAX_DEG       = (cyclone::real)32.0;
static const cyclone::real STEER_RATE_DEG      = (cyclone::real)180.0;
static const cyclone::real STEER_RETURN_DEG    = (cyclone::real)260.0;
static const cyclone::real HIGH_SPEED_REF      = (cyclone::real)25.0;     // 用于高速衰减

// 防侧翻
//   P 项使 bodyUp 向 worldUp 收敛。D 项仅在偶合鲁棒性上起作用，以免压制过弯侧倾。
static const cyclone::real UPRIGHT_P           = (cyclone::real)6500.0;
static const cyclone::real UPRIGHT_D           = (cyclone::real)900.0;

// 碰撞参数
//   地面接触：作为兜底，friction/restitution 都很低，主要靠悬挂支撑
//   障碍物：低弹性中等摩擦
static const cyclone::real GROUND_FRICTION     = (cyclone::real)0.05;
static const cyclone::real GROUND_RESTITUTION  = (cyclone::real)0.0;
static const cyclone::real BOX_FRICTION        = (cyclone::real)0.4;
static const cyclone::real BOX_RESTITUTION     = (cyclone::real)0.15;
static const cyclone::real CONTACT_TOLERANCE   = (cyclone::real)0.0;

// 障碍物
static const unsigned      OBSTACLE_COUNT      = 3;

// 数值健壮性
static const cyclone::real MAX_LIN_SPEED       = (cyclone::real)80.0;
static const cyclone::real MAX_ANG_SPEED       = (cyclone::real)20.0;
static const cyclone::real WORLD_BOUND         = (cyclone::real)500.0;

// 度 / 弧度
static const cyclone::real DEG2RAD             = (cyclone::real)0.01745329252;

static inline bool isFiniteReal(cyclone::real x)
{
    return (x == x) && (x < (cyclone::real)1e30) && (x > -(cyclone::real)1e30);
}
static inline bool isFiniteVec(const cyclone::Vector3 &v)
{
    return isFiniteReal(v.x) && isFiniteReal(v.y) && isFiniteReal(v.z);
}


// ===========================================================================
// VehicleDemo
// ===========================================================================
class VehicleDemo : public RigidBodyApplication
{
    // 车身刚体 + 碰撞代理
    cyclone::CollisionBox chassisCol;
    cyclone::RigidBody    chassisBody;

    // 卡丁车工具（含 4 个 RaycastWheel）
    cyclone::kart::KartVehicle kart;

    // 静态障碍物
    cyclone::CollisionBox obstacles[OBSTACLE_COUNT];
    cyclone::RigidBody    obstacleBodies[OBSTACLE_COUNT];

    // 输入
    bool keyW, keyS, keyA, keyD, keySpace, keyShift;

    // 转向角（度）
    cyclone::real steeringDeg;

    // 出生点
    cyclone::Vector3 spawnPosition;

    // 平滑相机
    cyclone::Vector3 camPos;
    cyclone::Vector3 camLook;
    bool camInited;

protected:
    virtual void generateContacts();
    virtual void updateObjects(cyclone::real duration);
    virtual void reset();

    void initBody(cyclone::RigidBody &body, cyclone::real mass,
                  const cyclone::Matrix3 &it,
                  cyclone::real linDamp, cyclone::real angDamp,
                  bool useGravity);

    /** 把当前 steeringDeg + 高速衰减反映到每个轮子的 wheelForward/wheelRight。 */
    void updateWheelDirections();

    /** 输入 → KartVehicle::applyXxx 系列调用 */
    void applyInputs(cyclone::real duration);

    /** 数值兜底（NaN / 越界 / 速度过大 → reset） */
    bool sanityCheckAndMaybeReset(cyclone::real duration);

public:
    VehicleDemo();
    virtual ~VehicleDemo();

    virtual const char* getTitle();
    virtual void display();
    virtual void update();
    virtual void key(unsigned char key);
};


// ---------------------------------------------------------------------------
// 构造 / 析构
// ---------------------------------------------------------------------------
VehicleDemo::VehicleDemo()
    : RigidBodyApplication(),
      keyW(false), keyS(false), keyA(false), keyD(false),
      keySpace(false), keyShift(false),
      steeringDeg(0),
      // spawn.y 让车身底面 + 悬挂全展开后，轮心刚好接近地面
      spawnPosition((cyclone::real)0,
                    CHASSIS_HALF_Y + WHEEL_REST_LENGTH + (cyclone::real)0.2,
                    (cyclone::real)0),
      camPos(0, 5, -10), camLook(0, 0, 0), camInited(false)
{
    pauseSimulation = false;

    // 关联碰撞 primitive
    chassisCol.body = &chassisBody;
    chassisCol.halfSize = cyclone::Vector3(CHASSIS_HALF_X, CHASSIS_HALF_Y, CHASSIS_HALF_Z);
    chassisCol.offset = cyclone::Matrix4();

    // KartVehicle 全局参数（与上面 const 区一致）
    kart.chassis              = &chassisBody;
    kart.suspensionStiffness  = SUSP_STIFFNESS;
    kart.suspensionDamping    = SUSP_DAMPING;
    kart.maxNormalForce       = CHASSIS_MASS * (cyclone::real)9.81 * SUSP_FN_MAX_SCALE;
    kart.driveForcePerWheel   = DRIVE_FORCE_PER_WHEEL;
    kart.reverseForcePerWheel = REVERSE_FORCE_PER_WHEEL;
    kart.brakeForcePerWheel   = BRAKE_FORCE_PER_WHEEL;
    kart.maxForwardSpeed      = MAX_FORWARD_SPEED;
    kart.maxReverseSpeed      = MAX_REVERSE_SPEED;
    kart.rollingResistance    = ROLLING_K;
    kart.gripFront            = GRIP_FRONT;
    kart.gripRear             = GRIP_REAR;
    kart.driftGripScale       = DRIFT_GRIP_SCALE;
    kart.maxLateralImpulse    = MAX_LATERAL_FORCE;
    kart.steerMaxRad          = STEER_MAX_DEG * DEG2RAD;
    kart.highSpeedRef         = HIGH_SPEED_REF;
    kart.uprightP             = UPRIGHT_P;
    kart.uprightD             = UPRIGHT_D;
    kart.currentRearGripScale = (cyclone::real)1.0;

    // 4 个 RaycastWheel：约定 +X 为车头方向、+Y 为上、+Z 为右
    // FL=0, FR=1, RL=2, RR=3
    cyclone::real ax_front =  CHASSIS_HALF_X * (cyclone::real)0.78;
    cyclone::real ax_rear  = -CHASSIS_HALF_X * (cyclone::real)0.78;
    cyclone::real az_left  = -CHASSIS_HALF_Z * (cyclone::real)1.05;
    cyclone::real az_right =  CHASSIS_HALF_Z * (cyclone::real)1.05;
    cyclone::real ay       = -CHASSIS_HALF_Y * (cyclone::real)0.4; // 挂点在车身下半部
    struct WheelDef { cyclone::real x, y, z; bool drive; bool steer; };
    WheelDef defs[4] = {
        { ax_front, ay, az_left,  false, true  }, // FL
        { ax_front, ay, az_right, false, true  }, // FR
        { ax_rear,  ay, az_left,  true,  false }, // RL
        { ax_rear,  ay, az_right, true,  false }  // RR
    };
    for (int i = 0; i < 4; ++i)
    {
        cyclone::kart::RaycastWheel &w = kart.wheels[i];
        w.anchorLocal = cyclone::Vector3(defs[i].x, defs[i].y, defs[i].z);
        w.restLength  = WHEEL_REST_LENGTH;
        w.maxTravel   = WHEEL_MAX_TRAVEL;
        w.radius      = WHEEL_RADIUS;
        w.isDrive     = defs[i].drive;
        w.isSteer     = defs[i].steer;
        w.steerAngle  = 0;
        w.rollAngle   = 0;
    }

    // 障碍物
    for (unsigned i = 0; i < OBSTACLE_COUNT; i++)
    {
        obstacles[i].body = &obstacleBodies[i];
        obstacles[i].halfSize = cyclone::Vector3(
            (cyclone::real)1.0, (cyclone::real)0.5, (cyclone::real)1.0);
        obstacles[i].offset = cyclone::Matrix4();
    }

    reset();
}

VehicleDemo::~VehicleDemo()
{
}


// ---------------------------------------------------------------------------
// 工具
// ---------------------------------------------------------------------------
void VehicleDemo::initBody(cyclone::RigidBody &body, cyclone::real mass,
                           const cyclone::Matrix3 &it,
                           cyclone::real linDamp, cyclone::real angDamp,
                           bool useGravity)
{
    body.setMass(mass);
    body.setInertiaTensor(it);
    body.setLinearDamping(linDamp);
    body.setAngularDamping(angDamp);
    if (useGravity) body.setAcceleration(cyclone::Vector3::GRAVITY);
    else body.setAcceleration(cyclone::Vector3(0, 0, 0));
    body.setVelocity(0, 0, 0);
    body.setRotation(0, 0, 0);
    body.clearAccumulators();
    body.setAwake(true);
    body.setCanSleep(false);
    body.calculateDerivedData();
}


// ---------------------------------------------------------------------------
// reset
// ---------------------------------------------------------------------------
void VehicleDemo::reset()
{
    keyW = keyS = keyA = keyD = keySpace = keyShift = false;
    steeringDeg = 0;
    kart.currentRearGripScale = (cyclone::real)1.0;

    // 车身
    chassisBody.setPosition(spawnPosition);
    chassisBody.setOrientation(1, 0, 0, 0);
    cyclone::Matrix3 it;
    it.setBlockInertiaTensor(
        cyclone::Vector3(CHASSIS_HALF_X, CHASSIS_HALF_Y, CHASSIS_HALF_Z),
        CHASSIS_MASS);
    initBody(chassisBody, CHASSIS_MASS, it,
             CHASSIS_LIN_DAMPING, CHASSIS_ANG_DAMPING, true);
    chassisCol.calculateInternals();

    // 重置每个轮的运行时缓存
    for (int i = 0; i < 4; ++i)
    {
        cyclone::kart::RaycastWheel &w = kart.wheels[i];
        w.grounded = false;
        w.compression = 0;
        w.steerAngle = 0;
        w.rollAngle = 0;
        w.hitPoint = cyclone::Vector3(0, 0, 0);
        w.hitNormal = cyclone::Vector3(0, 1, 0);
        w.wheelForward = cyclone::Vector3(1, 0, 0);
        w.wheelRight   = cyclone::Vector3(0, 0, 1);
    }

    // 障碍物
    cyclone::Vector3 obstaclePositions[OBSTACLE_COUNT] = {
        cyclone::Vector3((cyclone::real)15.0,  (cyclone::real)0.5, (cyclone::real)0.0),
        cyclone::Vector3((cyclone::real)-10.0, (cyclone::real)0.5, (cyclone::real)5.0),
        cyclone::Vector3((cyclone::real)0.0,   (cyclone::real)0.5, (cyclone::real)20.0)
    };
    for (unsigned i = 0; i < OBSTACLE_COUNT; i++)
    {
        obstacleBodies[i].setPosition(obstaclePositions[i]);
        obstacleBodies[i].setOrientation(1, 0, 0, 0);
        obstacleBodies[i].setVelocity(0, 0, 0);
        obstacleBodies[i].setRotation(0, 0, 0);
        obstacleBodies[i].setInverseMass(0);
        cyclone::Matrix3 zeroIT;
        zeroIT.setInertiaTensorCoeffs(0, 0, 0);
        obstacleBodies[i].setInverseInertiaTensor(zeroIT);
        obstacleBodies[i].setLinearDamping((cyclone::real)0.99);
        obstacleBodies[i].setAngularDamping((cyclone::real)0.99);
        obstacleBodies[i].setAcceleration(0, 0, 0);
        obstacleBodies[i].clearAccumulators();
        obstacleBodies[i].setAwake(true);
        obstacleBodies[i].setCanSleep(false);
        obstacleBodies[i].calculateDerivedData();
        obstacles[i].calculateInternals();
    }

    cData.contactCount = 0;
    camInited = false;
}


// ---------------------------------------------------------------------------
// 转向：由 steeringDeg 反映到 4 个 RaycastWheel 的 wheelForward/Right
// ---------------------------------------------------------------------------
void VehicleDemo::updateWheelDirections()
{
    cyclone::Matrix4 tm = chassisBody.getTransform();
    cyclone::Vector3 bodyForward = tm.transformDirection(cyclone::Vector3(1, 0, 0));
    cyclone::Vector3 bodyRight   = tm.transformDirection(cyclone::Vector3(0, 0, 1));
    cyclone::Vector3 bodyUp      = tm.transformDirection(cyclone::Vector3(0, 1, 0));
    bodyForward.normalise();
    bodyRight.normalise();
    bodyUp.normalise();

    // 高速衰减
    cyclone::Vector3 vCM = chassisBody.getVelocity();
    cyclone::real fwdSpeed = vCM * bodyForward;
    cyclone::real fwdAbs = real_abs(fwdSpeed);
    cyclone::real t = fwdAbs / HIGH_SPEED_REF;
    if (t > (cyclone::real)1.0) t = (cyclone::real)1.0;
    cyclone::real steerScale = (cyclone::real)1.0 + ((cyclone::real)0.5 - (cyclone::real)1.0) * t; // lerp(1, 0.5, t)

    cyclone::real steerEffectiveRad = steeringDeg * DEG2RAD * steerScale;
    cyclone::real cs = real_cos(steerEffectiveRad);
    cyclone::real sn = real_sin(steerEffectiveRad);

    for (int i = 0; i < 4; ++i)
    {
        cyclone::kart::RaycastWheel &w = kart.wheels[i];
        if (w.isSteer)
        {
            // 右手系绕车身本地 +Y 旋转 +θ（A 键 → 左转 → +θ）：
            //   fwd'   = cos·fwd + sin·right
            //   right' = -sin·fwd + cos·right
            // 之前用了左手系公式（符号反了），导致前轮 wheelRight 朝着前向方向偏，
            // 一前进就把"前向速度"算成了"侧向速度"，瞬间钳到 25 kN，让车横甩+抽搐。
            w.wheelForward = bodyForward * cs + bodyRight * sn;
            w.wheelRight   = bodyRight   * cs - bodyForward * sn;
            w.steerAngle   = steerEffectiveRad;
        }
        else
        {
            w.wheelForward = bodyForward;
            w.wheelRight   = bodyRight;
            w.steerAngle   = 0;
        }
    }
}


// ---------------------------------------------------------------------------
// 处理输入 + 调 KartVehicle 的力施加函数
// ---------------------------------------------------------------------------
void VehicleDemo::applyInputs(cyclone::real duration)
{
    // 1) 转向角度更新（A 增大、D 减小、A/D 都未按则回正）
    if (keyA && !keyD)      steeringDeg += STEER_RATE_DEG * duration;
    else if (keyD && !keyA) steeringDeg -= STEER_RATE_DEG * duration;
    else
    {
        cyclone::real ret = STEER_RETURN_DEG * duration;
        if (steeringDeg >  ret)       steeringDeg -= ret;
        else if (steeringDeg < -ret)  steeringDeg += ret;
        else                          steeringDeg  = 0;
    }
    if (steeringDeg >  STEER_MAX_DEG) steeringDeg =  STEER_MAX_DEG;
    if (steeringDeg < -STEER_MAX_DEG) steeringDeg = -STEER_MAX_DEG;

    // 2) 把 steeringDeg 反映到 wheelForward/Right
    updateWheelDirections();

    // 3) 准备障碍物 box 数组（用于 raycast）
    const cyclone::CollisionBox *boxPtrs[OBSTACLE_COUNT];
    for (unsigned i = 0; i < OBSTACLE_COUNT; ++i) boxPtrs[i] = &obstacles[i];

    cyclone::CollisionPlane ground;
    ground.direction = cyclone::Vector3(0, 1, 0);
    ground.offset    = 0;

    // 4) 射线探测（在物理子步外，渲染帧只调一次）
    kart.updateRaycasts(&ground, boxPtrs, OBSTACLE_COUNT);

    // 5) 施加悬挂支撑力
    kart.applySuspension();

    // 6) 输入打包
    cyclone::kart::KartInput in;
    in.throttle   = keyW;
    in.reverse    = keyS;
    in.brake      = keySpace;
    in.steerLeft  = keyA;
    in.steerRight = keyD;
    in.drift      = keyShift;

    // 7) 纵向 / 横向力
    kart.applyLongitudinal(in, duration);
    kart.applyLateral(in, duration);

    // 8) 防侧翻 / 角阻尼兜底
    kart.applyStabilization();

    // 9) 视觉滚动角积分（仅渲染用）
    cyclone::Matrix4 tm = chassisBody.getTransform();
    for (int i = 0; i < 4; ++i)
    {
        cyclone::kart::RaycastWheel &w = kart.wheels[i];
        if (w.grounded)
        {
            cyclone::Vector3 vp = kart.getPointVelocityWorld(w.hitPoint);
            cyclone::Vector3 fwdProj = w.wheelForward;
            cyclone::real vLong = vp * fwdProj;
            w.rollAngle += vLong / w.radius * duration;
            // wrap
            if (w.rollAngle > (cyclone::real)6.2831853) w.rollAngle -= (cyclone::real)6.2831853;
            if (w.rollAngle < -(cyclone::real)6.2831853) w.rollAngle += (cyclone::real)6.2831853;
        }
    }
}


// ---------------------------------------------------------------------------
// 数值兜底
// ---------------------------------------------------------------------------
bool VehicleDemo::sanityCheckAndMaybeReset(cyclone::real /*duration*/)
{
    cyclone::Vector3 p = chassisBody.getPosition();
    cyclone::Vector3 v = chassisBody.getVelocity();
    cyclone::Vector3 w = chassisBody.getRotation();
    if (!isFiniteVec(p) || !isFiniteVec(v) || !isFiniteVec(w)) { reset(); return true; }
    if (real_abs(p.x) > WORLD_BOUND ||
        real_abs(p.y) > WORLD_BOUND ||
        real_abs(p.z) > WORLD_BOUND) { reset(); return true; }

    // 单帧位移过大 → 速度钳位
    if (v.squareMagnitude() > MAX_LIN_SPEED * MAX_LIN_SPEED)
    {
        v *= MAX_LIN_SPEED / v.magnitude();
        chassisBody.setVelocity(v);
    }
    if (w.squareMagnitude() > MAX_ANG_SPEED * MAX_ANG_SPEED)
    {
        w *= MAX_ANG_SPEED / w.magnitude();
        chassisBody.setRotation(w);
    }
    return false;
}


// ---------------------------------------------------------------------------
// updateObjects（每个物理子步调一次）
// ---------------------------------------------------------------------------
void VehicleDemo::updateObjects(cyclone::real duration)
{
    chassisBody.clearAccumulators();
    for (unsigned i = 0; i < OBSTACLE_COUNT; i++) obstacleBodies[i].clearAccumulators();

#if VEHICLE_LOG
    log_open();
    bool verbose = (g_logFrame < g_logVerboseFrames);
    // 按键状态变化时打一条醒目分隔线，方便在长日志中快速定位
    static int prevW=0, prevS=0, prevA=0, prevD=0, prevSp=0, prevSh=0;
    if (keyW!=prevW || keyS!=prevS || keyA!=prevA || keyD!=prevD || keySpace!=prevSp || keyShift!=prevSh)
    {
        log_line("================ INPUT CHANGE  W%d S%d A%d D%d Sp%d Sh%d  (was W%d S%d A%d D%d Sp%d Sh%d)  frame=%d ================",
                 keyW,keyS,keyA,keyD,keySpace,keyShift, prevW,prevS,prevA,prevD,prevSp,prevSh, g_logFrame);
        prevW=keyW; prevS=keyS; prevA=keyA; prevD=keyD; prevSp=keySpace; prevSh=keyShift;
    }
    cyclone::Vector3 preP = chassisBody.getPosition();
    cyclone::Vector3 preV = chassisBody.getVelocity();
    cyclone::Vector3 preW = chassisBody.getRotation();
    cyclone::Matrix4 preTM = chassisBody.getTransform();
    cyclone::Vector3 preBodyY(preTM.data[1], preTM.data[5], preTM.data[9]);
    if (verbose)
    {
        log_line("---- frame %d  dt=%.4f  in(W%d S%d A%d D%d Sp%d Sh%d) steer=%+6.2f ----",
                 g_logFrame, (double)duration,
                 keyW, keyS, keyA, keyD, keySpace, keyShift,
                 (double)steeringDeg);
        log_line("  PRE  P=(%+6.3f,%+6.3f,%+6.3f) V=(%+6.3f,%+6.3f,%+6.3f) W=(%+6.3f,%+6.3f,%+6.3f) bodyY=(%+5.2f,%+5.2f,%+5.2f)",
                 (double)preP.x,(double)preP.y,(double)preP.z,
                 (double)preV.x,(double)preV.y,(double)preV.z,
                 (double)preW.x,(double)preW.y,(double)preW.z,
                 (double)preBodyY.x,(double)preBodyY.y,(double)preBodyY.z);
    }
#endif

    // 输入 → 力
    applyInputs(duration);

#if VEHICLE_LOG
    if (verbose)
    {
        // 4 轮当前状态
        for (int i = 0; i < 4; ++i)
        {
            const cyclone::kart::RaycastWheel &wh = kart.wheels[i];
            log_line("  w%d %s comp=%5.3f hp=(%+6.3f,%+6.3f,%+6.3f) hn=(%+5.2f,%+5.2f,%+5.2f) wfwd=(%+5.2f,%+5.2f,%+5.2f) wrt=(%+5.2f,%+5.2f,%+5.2f)",
                     i, wh.grounded?"GND":"AIR", (double)wh.compression,
                     (double)wh.hitPoint.x,(double)wh.hitPoint.y,(double)wh.hitPoint.z,
                     (double)wh.hitNormal.x,(double)wh.hitNormal.y,(double)wh.hitNormal.z,
                     (double)wh.wheelForward.x,(double)wh.wheelForward.y,(double)wh.wheelForward.z,
                     (double)wh.wheelRight.x,(double)wh.wheelRight.y,(double)wh.wheelRight.z);
        }
        // applyInputs 之后 / integrate 之前的合力 / 合力矩
        cyclone::Vector3 fAcc = chassisBody.getLastFrameAcceleration() * (cyclone::real)0; // 占位，不易直接拿合力
        // 这里不打印合力，因为没有 public getter；用 V/W 在 integrate 后变化代表
    }
#endif

    // 速度钳位（积分前）
    {
        cyclone::Vector3 v = chassisBody.getVelocity();
        cyclone::real vm2 = v.squareMagnitude();
        if (vm2 > MAX_LIN_SPEED * MAX_LIN_SPEED)
        {
            v *= MAX_LIN_SPEED / real_sqrt(vm2);
            chassisBody.setVelocity(v);
        }
        cyclone::Vector3 ww = chassisBody.getRotation();
        cyclone::real wm2 = ww.squareMagnitude();
        if (wm2 > MAX_ANG_SPEED * MAX_ANG_SPEED)
        {
            ww *= MAX_ANG_SPEED / real_sqrt(wm2);
            chassisBody.setRotation(ww);
        }
    }

    chassisBody.integrate(duration);
    chassisCol.calculateInternals();
    for (unsigned i = 0; i < OBSTACLE_COUNT; i++) obstacles[i].calculateInternals();

#if VEHICLE_LOG
    if (verbose)
    {
        cyclone::Vector3 postP = chassisBody.getPosition();
        cyclone::Vector3 postV = chassisBody.getVelocity();
        cyclone::Vector3 postW = chassisBody.getRotation();
        cyclone::Matrix4 postTM = chassisBody.getTransform();
        cyclone::Vector3 postBodyY(postTM.data[1], postTM.data[5], postTM.data[9]);
        log_line("  POST P=(%+6.3f,%+6.3f,%+6.3f) V=(%+6.3f,%+6.3f,%+6.3f) W=(%+6.3f,%+6.3f,%+6.3f) bodyY=(%+5.2f,%+5.2f,%+5.2f)",
                 (double)postP.x,(double)postP.y,(double)postP.z,
                 (double)postV.x,(double)postV.y,(double)postV.z,
                 (double)postW.x,(double)postW.y,(double)postW.z,
                 (double)postBodyY.x,(double)postBodyY.y,(double)postBodyY.z);
        // dV / dW
        cyclone::Vector3 dV = postV - preV;
        cyclone::Vector3 dW = postW - preW;
        log_line("  dV=(%+6.3f,%+6.3f,%+6.3f) dW=(%+6.3f,%+6.3f,%+6.3f) |dW|=%.2f rad/s",
                 (double)dV.x,(double)dV.y,(double)dV.z,
                 (double)dW.x,(double)dW.y,(double)dW.z,
                 (double)dW.magnitude());
    }
    g_logFrame++;
#endif

    bool wasReset = sanityCheckAndMaybeReset(duration);
#if VEHICLE_LOG
    if (verbose && wasReset)
        log_line("  !!! sanityCheck triggered RESET");
#endif
}


// ---------------------------------------------------------------------------
// 碰撞：仅车身 vs 地面 / 障碍物
// ---------------------------------------------------------------------------
void VehicleDemo::generateContacts()
{
    cData.reset(maxContacts);
    cData.tolerance = CONTACT_TOLERANCE;

    cyclone::CollisionPlane ground;
    ground.direction = cyclone::Vector3(0, 1, 0);
    ground.offset    = 0;

    // 车身 vs 地面（friction/restitution 都低，作为兜底）
    if (cData.hasMoreContacts())
    {
        cData.friction = GROUND_FRICTION;
        cData.restitution = GROUND_RESTITUTION;
        cyclone::CollisionDetector::boxAndHalfSpace(chassisCol, ground, &cData);
    }

    // 车身 vs 障碍物
    cData.friction = BOX_FRICTION;
    cData.restitution = BOX_RESTITUTION;
    for (unsigned i = 0; i < OBSTACLE_COUNT; i++)
    {
        if (!cData.hasMoreContacts()) return;
        cyclone::CollisionDetector::boxAndBox(chassisCol, obstacles[i], &cData);
    }
}


// ---------------------------------------------------------------------------
// 每帧 update：查询键盘 → 让基类驱动子步
// ---------------------------------------------------------------------------
void VehicleDemo::update()
{
#ifdef _WIN32
    keyW     = isKeyDown('W');
    keyS     = isKeyDown('S');
    keyA     = isKeyDown('A');
    keyD     = isKeyDown('D');
    keySpace = isKeyDown(VK_SPACE);
    keyShift = isKeyDown(VK_SHIFT);
#endif
    RigidBodyApplication::update();
}


// ---------------------------------------------------------------------------
// 键盘（GLUT 一次性热键）
// ---------------------------------------------------------------------------
void VehicleDemo::key(unsigned char key)
{
    switch (key)
    {
    case 'r':
    case 'R':
        reset();
        return;
    default:
        break;
    }
    RigidBodyApplication::key(key);
}


// ===========================================================================
// 渲染
// ===========================================================================
const char* VehicleDemo::getTitle()
{
    return "Cyclone > Vehicle Demo (raycast kart)";
}

static void drawGroundGrid(int cx, int cz)
{
    glColor3f(0.6f, 0.6f, 0.6f);
    glBegin(GL_LINES);
    for (int x = -25; x <= 25; x++)
    {
        glVertex3f((float)(cx + x), 0.0f, (float)(cz - 25));
        glVertex3f((float)(cx + x), 0.0f, (float)(cz + 25));
    }
    for (int z = -25; z <= 25; z++)
    {
        glVertex3f((float)(cx - 25), 0.0f, (float)(cz + z));
        glVertex3f((float)(cx + 25), 0.0f, (float)(cz + z));
    }
    glEnd();
}

// 用 OpenGL 直接画一个扁圆柱（车轮渲染）。沿 Z 轴方向（高=厚度）。
static void drawWheelCylinder(float radius, float thickness, int slices)
{
    float half = thickness * 0.5f;
    // 上下圆盘
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(0, 0, +half);
    for (int i = 0; i <= slices; ++i)
    {
        float a = (float)i / (float)slices * 6.2831853f;
        glVertex3f(cosf(a) * radius, sinf(a) * radius, +half);
    }
    glEnd();
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(0, 0, -half);
    for (int i = slices; i >= 0; --i)
    {
        float a = (float)i / (float)slices * 6.2831853f;
        glVertex3f(cosf(a) * radius, sinf(a) * radius, -half);
    }
    glEnd();
    // 侧壁
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= slices; ++i)
    {
        float a = (float)i / (float)slices * 6.2831853f;
        float cx = cosf(a) * radius, cy = sinf(a) * radius;
        glVertex3f(cx, cy, +half);
        glVertex3f(cx, cy, -half);
    }
    glEnd();
}

void VehicleDemo::display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    // ---- 平滑跟随相机 ----
    cyclone::Vector3 pos = chassisBody.getPosition();
    cyclone::Matrix4 tm = chassisBody.getTransform();
    cyclone::Vector3 fwd = tm.transformDirection(cyclone::Vector3(1, 0, 0));
    fwd.y = 0;
    if (fwd.squareMagnitude() < (cyclone::real)1e-6) fwd = cyclone::Vector3(1, 0, 0);
    fwd.normalise();

    cyclone::Vector3 desiredCamPos = pos + fwd * (cyclone::real)(-7.5) +
                                     cyclone::Vector3(0, (cyclone::real)3.0, 0);
    cyclone::Vector3 desiredLook   = pos + fwd * (cyclone::real)2.0;

    if (!camInited)
    {
        camPos = desiredCamPos;
        camLook = desiredLook;
        camInited = true;
    }
    else
    {
        // 简单 lerp（动画速度感）
        cyclone::real alpha = (cyclone::real)0.15;
        camPos = camPos + (desiredCamPos - camPos) * alpha;
        camLook = camLook + (desiredLook - camLook) * alpha;
    }
    gluLookAt(camPos.x, camPos.y, camPos.z,
              camLook.x, camLook.y, camLook.z,
              0.0, 1.0, 0.0);

    // ---- 地面网格 ----
    drawGroundGrid((int)pos.x, (int)pos.z);

    // ---- 车身 ----
    {
        GLfloat mat[16];
        chassisBody.getGLTransform(mat);
        glColor3f(0.2f, 0.4f, 0.85f);
        glPushMatrix();
        glMultMatrixf(mat);
        glScalef((float)(CHASSIS_HALF_X * 2),
                 (float)(CHASSIS_HALF_Y * 2),
                 (float)(CHASSIS_HALF_Z * 2));
        glutSolidCube(1.0f);
        glPopMatrix();
    }

    // ---- 4 个车轮（按车身变换 * (anchorLocal 沿 -Y 平移压缩量) * 转向 * 滚动） ----
    glColor3f(0.12f, 0.12f, 0.14f);
    {
        GLfloat chassisMat[16];
        chassisBody.getGLTransform(chassisMat);
        for (int i = 0; i < 4; ++i)
        {
            const cyclone::kart::RaycastWheel &w = kart.wheels[i];
            // 车轮中心相对挂点：沿车身本地 -Y 平移 (restLength - compression)
            cyclone::real downOffset = w.restLength - w.compression;
            glPushMatrix();
            glMultMatrixf(chassisMat);
            glTranslatef((float)w.anchorLocal.x,
                         (float)(w.anchorLocal.y - downOffset),
                         (float)w.anchorLocal.z);
            // 转向：仅前轮，绕本地 +Y 旋转 steerAngle
            if (w.isSteer)
            {
                glRotatef((float)(w.steerAngle / DEG2RAD), 0, 1, 0);
            }
            // 滚动：drawWheelCylinder 的轮轴沿 +Z（= 车身右方向）。
            // 车轮在地面上向前滚动（车前向 = +X）时，圆盘顶部应该向 +X 转动；
            // 在 OpenGL 右手系下，这等价于绕 -Z 旋转（或绕 +Z 旋转 -rollAngle）。
            glRotatef((float)(-w.rollAngle / DEG2RAD), 0, 0, 1);
            drawWheelCylinder((float)w.radius, (float)0.18, 16);
            glPopMatrix();
        }
    }

    // ---- 障碍物 ----
    glColor3f(0.85f, 0.2f, 0.2f);
    for (unsigned i = 0; i < OBSTACLE_COUNT; i++)
    {
        GLfloat mat[16];
        obstacleBodies[i].getGLTransform(mat);
        glPushMatrix();
        glMultMatrixf(mat);
        glScalef((float)(obstacles[i].halfSize.x * 2),
                 (float)(obstacles[i].halfSize.y * 2),
                 (float)(obstacles[i].halfSize.z * 2));
        glutSolidCube(1.0f);
        glPopMatrix();
    }

    RigidBodyApplication::drawDebug();

    // ---- HUD ----
    char buffer[256];
    glColor3f(0.0f, 0.0f, 0.0f);

    cyclone::Vector3 vCM = chassisBody.getVelocity();
    cyclone::real fwdSpeed = vCM * fwd;
    cyclone::real speedKmh = vCM.magnitude() * (cyclone::real)3.6;

    sprintf(buffer, "Speed: %+5.2f m/s  (%5.1f km/h)",
            (double)fwdSpeed, (double)speedKmh);
    renderText(10.0f, (float)height - 14.0f, buffer);

    sprintf(buffer, "Steering: %+5.1f deg  (rear grip x %.2f)",
            (double)steeringDeg,
            (double)kart.currentRearGripScale);
    renderText(10.0f, (float)height - 28.0f, buffer);

    const char* in = "Idle";
    if (keySpace) in = "Brake";
    else if (keyW) in = "Throttle";
    else if (keyS) in = "Reverse";
    sprintf(buffer, "Input: %s%s", in, keyShift ? " + Drift" : "");
    renderText(10.0f, (float)height - 42.0f, buffer);

    // 4 轮状态
    for (int i = 0; i < 4; ++i)
    {
        const cyclone::kart::RaycastWheel &w = kart.wheels[i];
        const char* name = (i == 0 ? "FL" : i == 1 ? "FR" : i == 2 ? "RL" : "RR");
        cyclone::real pct = (w.restLength > 0) ? (w.compression / w.restLength * (cyclone::real)100.0) : 0;
        sprintf(buffer, "%s: %s  comp %5.1f%%",
                name, w.grounded ? "GND" : "AIR", (double)pct);
        renderText(10.0f, (float)height - 60.0f - i * 14.0f, buffer);
    }

    // 操作帮助
    renderText(10.0f, 24.0f,
        "W/S accel/reverse  A/D steer  Space brake  Shift drift");
    renderText(10.0f, 10.0f,
        "R reset  C contacts  P pause  Q quit");
}


// ===========================================================================
// 入口
// ===========================================================================
Application* getApplication()
{
    return new VehicleDemo();
}