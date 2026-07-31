/*
 * 卡丁车（raycast wheel）模块头文件
 *
 * 该模块为 cyclone 物理引擎提供"光线投射式车轮"工具，便于上层
 * demo 实现 车身刚体 + 4 车轮探测点 的多刚体卡丁车。
 *
 * 设计要点：
 *   - 车轮本身不是 RigidBody，也不参与碰撞解算；
 *   - 每帧从车身挂点沿"车身本地 -Y"方向投射射线；
 *   - 射线击中地面/障碍物 → 计算压缩量 → 弹簧+阻尼支撑力；
 *   - 纵向/横向轮胎力由上层 demo 在 KartInput 驱动下显式施加。
 *
 * 仅依赖 cyclone 既有的公共类型（Vector3、RigidBody、Matrix4、CollisionBox）。
 */
#ifndef CYCLONE_VEHICLE_KART_H
#define CYCLONE_VEHICLE_KART_H

#include "core.h"
#include "body.h"
#include "collide_fine.h"

namespace cyclone {
namespace kart {

    /**
     * 一个 raycast 车轮探测点。
     *
     * 不持有 RigidBody，只缓存挂点几何数据 + 每帧射线探测结果，
     * 以及一些渲染辅助信息（滚动角、当前转向角）。
     */
    class RaycastWheel
    {
    public:
        /* ===== 静态参数（由上层 demo 在初始化时填写） ===== */

        /** 在车身本地坐标系下的悬挂挂点（即弹簧上端固定在车身上的位置）。 */
        Vector3 anchorLocal;

        /** 静态悬挂长度（车轮挂着无负载时挂点到轮心的距离）。 */
        real restLength;

        /** 最大悬挂行程（压缩量上限）。 */
        real maxTravel;

        /** 车轮半径。 */
        real radius;

        /** 是否为驱动轮（驱动力施加在该子集）。 */
        bool isDrive;

        /** 是否为转向轮（转向角施加在该子集）。 */
        bool isSteer;

        /* ===== 运行时缓存（由 KartVehicle / 上层 demo 每帧填写） ===== */

        /** 当前是否触地（射线是否命中几何）。 */
        bool grounded;

        /** 射线命中距离（从挂点到 hitPoint 的距离）。仅在 grounded=true 时有效。 */
        real hitDistance;

        /** 命中点世界坐标。 */
        Vector3 hitPoint;

        /** 命中面法线（指向半空间外、或 box 被击中面外法线）。 */
        Vector3 hitNormal;

        /** 当前压缩量 = clamp(restLength - (hitDistance - radius), 0, maxTravel)。 */
        real compression;

        /** 当前转向角（弧度），仅 isSteer=true 时有效。由上层 demo 写入。 */
        real steerAngle;

        /** 视觉滚动角（弧度，绕车轮本地 Z 轴）。仅供渲染。 */
        real rollAngle;

        /** 当前车轮前向（世界方向，已含转向旋转）。 */
        Vector3 wheelForward;

        /** 当前车轮右向（世界方向，已含转向旋转）。 */
        Vector3 wheelRight;

        RaycastWheel();
    };

    /**
     * 描述一帧驾驶员输入。由上层 demo 每帧填充并传给 KartVehicle 的力施加函数。
     */
    struct KartInput
    {
        bool throttle;   //!< W
        bool brake;      //!< Space
        bool reverse;    //!< S
        bool steerLeft;  //!< A
        bool steerRight; //!< D
        bool drift;      //!< Shift

        KartInput()
            : throttle(false), brake(false), reverse(false),
              steerLeft(false), steerRight(false), drift(false) {}
    };

    /**
     * 卡丁车整车工具类。
     *
     * 持有：
     *   - 1 个车身 RigidBody*（外部拥有，不负责释放）
     *   - 4 个 RaycastWheel
     *   - 一组手感参数（在构造后由上层 demo 直接修改）
     *
     * 提供每帧调用的几个入口：
     *   - updateRaycasts(plane, boxes, boxCount)：刷新 4 轮射线探测结果
     *   - applySuspension()：施加悬挂法向支撑力
     *   - applyLongitudinal(input, dt)：施加驱动/刹车/滚阻
     *   - applyLateral(input, dt)：施加横向（侧向抓地/漂移）力
     *   - applyStabilization()：防侧翻 + 角阻尼兜底
     *
     * 注：所有"力"通过 chassis->addForceAtPoint / addForceAtBodyPoint 累加；
     *     真正的物理积分由上层 World/Application 完成。
     */
    class KartVehicle
    {
    public:
        /* ===== 几何 ===== */
        RigidBody *chassis;
        RaycastWheel wheels[4];

        /* ===== 手感参数（由上层 demo 在 reset 中赋值） ===== */
        // 悬挂
        real suspensionStiffness;   //!< k_s
        real suspensionDamping;     //!< k_d
        real maxNormalForce;        //!< Fn_max（钳位）

        // 纵向
        real driveForcePerWheel;
        real reverseForcePerWheel;
        real brakeForcePerWheel;
        real maxForwardSpeed;
        real maxReverseSpeed;
        real rollingResistance;     //!< k_roll

        // 横向 / 漂移
        real gripFront;
        real gripRear;
        real driftGripScale;
        real maxLateralImpulse;

        // 转向
        real steerMaxRad;
        real highSpeedRef;          //!< 用于高速衰减

        // 防侧翻
        real uprightP;
        real uprightD;

        // 漂移平滑
        real currentRearGripScale;  //!< 当前后轮抓地系数（驱动键放开后逐步恢复到 1.0）

        KartVehicle();

        /**
         * 主入口：4 轮射线探测。
         * @param ground   半空间地面（normal + offset），可为 NULL 表示无地面
         * @param boxes    可选的 box 障碍物数组（外部拥有）
         * @param boxCount boxes 数组长度
         */
        void updateRaycasts(const CollisionPlane *ground,
                            const CollisionBox  * const *boxes,
                            unsigned boxCount);

        /** 对 4 个触地轮施加沿命中法线方向的弹簧+阻尼支撑力。 */
        void applySuspension();

        /** 对 4 个触地驱动轮 / 全部触地轮施加纵向力（驱动/倒车/刹车/滚阻）。 */
        void applyLongitudinal(const KartInput &input, real dt);

        /** 对 4 个触地轮施加横向（侧向抓地/漂移）力。 */
        void applyLateral(const KartInput &input, real dt);

        /** 对车身施加防侧翻扭矩；4 轮触地时启用强一点的回正。 */
        void applyStabilization();

        /** 工具：求车身在某世界点的速度。 */
        Vector3 getPointVelocityWorld(const Vector3 &worldPoint) const;
    };

    /* ===================================================================== */
    /* Raycast 工具函数                                                        */
    /* ===================================================================== */

    /**
     * 射线 vs. 半空间地面。
     *
     * 半空间定义：{ p | dot(p, planeNormal) <= planeOffset }
     * 即 planeNormal 指向"半空间外"，命中条件为 origin 处于 / 即将进入半空间。
     *
     * @param origin       射线起点（世界坐标）
     * @param dir          射线方向（必须已归一化）
     * @param maxDist      最大检测距离
     * @param planeNormal  地面法线（已归一化、指向 up 方向）
     * @param planeOffset  地面偏移（沿 planeNormal）
     * @param outDist      命中时输出距离 [0, maxDist]
     * @param outNormal    命中时输出法线（= planeNormal）
     * @return 命中返回 true；与平面平行返回 false；起点在背面返回 outDist=0 命中。
     */
    bool raycastPlane(const Vector3 &origin,
                      const Vector3 &dir,
                      real maxDist,
                      const Vector3 &planeNormal,
                      real planeOffset,
                      real &outDist,
                      Vector3 &outNormal);

    /**
     * 射线 vs. OBB（CollisionBox）。
     *
     * 使用 slab 法（在 box 局部空间求 t_near / t_far）。
     *
     * @param origin   射线起点（世界坐标）
     * @param dir      射线方向（必须已归一化，世界坐标）
     * @param maxDist  最大检测距离
     * @param box      OBB（须先调用 calculateInternals 使 transform 有效）
     * @param outDist  命中时输出距离 [0, maxDist]
     * @param outNormal 命中时输出法线（被击中面外法线，世界坐标）
     * @return 命中 true，错过 false；起点在 box 内则 outDist=0 命中。
     */
    bool raycastBox(const Vector3 &origin,
                    const Vector3 &dir,
                    real maxDist,
                    const CollisionBox &box,
                    real &outDist,
                    Vector3 &outNormal);

} // namespace kart
} // namespace cyclone

#endif // CYCLONE_VEHICLE_KART_H
