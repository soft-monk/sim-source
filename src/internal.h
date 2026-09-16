// src/internal.h · sim-source 内部实现细节
//
// **宿主 MUST NOT 包含本文件**（唯一公开面是 include/sim_source/sim_source.h）。
//
// 设计要点：
//   · 几何一律在**局部平面**（东 x / 北 y，米）上算：webMap 尺度（数十公里）内，
//     等距圆柱投影的误差远小于绕行缓冲（默认数十米），且让"A* 绕行 + 线面相交判定"
//     退化成干净的二维问题（README 只要求"避让 hard 禁飞区"，不要求大地测量精度）。
//   · 局部平面原点由场景包围盒中心决定，**只依赖输入数据**（与当前工作目录、挂钟、
//     随机数无关）→ 同一场景双跑的浮点结果逐位一致（SIM-NFR-01）。
#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "sim_source/sim_source.h"

namespace sim_source {
namespace detail {

// ============================================================================
// 局部平面（东 x / 北 y，米）与 WGS84 的互转
// ============================================================================

inline constexpr double kPi = 3.14159265358979323846;

struct Vec2 {
    double x = 0.0;  // 东，米
    double y = 0.0;  // 北，米

    Vec2() = default;
    Vec2(double x_, double y_) : x(x_), y(y_) {}
    Vec2 operator+(const Vec2& o) const { return Vec2(x + o.x, y + o.y); }
    Vec2 operator-(const Vec2& o) const { return Vec2(x - o.x, y - o.y); }
    Vec2 operator*(double k) const { return Vec2(x * k, y * k); }
    double dot(const Vec2& o) const { return x * o.x + y * o.y; }
    double cross(const Vec2& o) const { return x * o.y - y * o.x; }
    double len() const;
};

/// WGS84 ⇄ 局部平面。原点为参考点；`cosLat0` 取参考纬度。
struct LocalFrame {
    double refLng = 0.0;
    double refLat = 0.0;
    double cosLat0 = 1.0;

    static LocalFrame fromCenter(double lng, double lat);
    Vec2 toLocal(double lng, double lat) const;
    std::pair<double, double> toWgs(const Vec2& p) const;

    static constexpr double kMetersPerDegLat = 111320.0;
    static constexpr double kMetersPerDegLngAtEquator = 111320.0;
};

using Poly = std::vector<Vec2>;

// ============================================================================
// 二维几何（全部纯函数）
// ============================================================================

/// 多边形有向面积的两倍（正 = 逆时针）
double polygonArea2(const Poly& p);
/// 点是否在多边形内（射线法；边界上算**在**内）
bool pointInPolygon(const Poly& p, const Vec2& q);
/// 点是否在多边形**内部**（边界上的点算在外）—— 绕行判定用这个口径
bool pointStrictlyInPolygon(const Poly& p, const Vec2& q);
/// 点到线段最短距离（米）
double pointSegmentDistance(const Vec2& q, const Vec2& a, const Vec2& b);
/// 线段是否**穿入**多边形：任何一次边界穿越、或端点/中点落在内部 → true
bool segmentCrossesPolygon(const Poly& p, const Vec2& a, const Vec2& b);
/// 折线是否从未穿入任何给定多边形
bool polylineCrossesAny(const std::vector<Vec2>& line, const std::vector<Poly>& obstacles);
/// 多边形按 `margin` 向外膨胀：逆时针环沿边法线外移、再沿顶点方向补齐
/// （凸角处生成单点；凹角不会出现，因为缓冲远小于多边形尺度）
Poly inflatePolygon(const Poly& p, double margin);
/// 环是否闭合有效（>= 3 点且面积非零）
bool validRing(const Poly& p);
/// 折线总长
double polylineLength(const std::vector<Vec2>& line);

// ============================================================================
// 确定性格式化（不依赖 locale / 默认精度）
// ============================================================================

/// 定点格式化：`decimals` 位小数，四舍五入由 ostringstream 决定（locale 无关）。
/// 用途：事件坐标按固定小数位输出，保证"同一输入双跑逐字节一致"可跨平台复核。
std::string formatFixed(double v, int decimals);
/// JSON 数值（有限值 → 数值；NaN/Inf → null，绝不产出非法 JSON 字面量）
json numOrNull(double v);
/// JSON 数值（定点小数值）
json fixedNum(double v, int decimals);

/// 航向归一化到 [0, 360)
double normalizeHeading(double deg);

// ============================================================================
// 规划与运行的内部结构
// ============================================================================

/// 一个已膨胀的硬禁飞区（绕行用）
struct Obstacle {
    std::string key;
    Poly ring;  // 已按 safetyMarginM 外扩
};

/// 规划中间件：一个平台的航路（局部平面坐标）
struct PlannedRoute {
    std::string platformId;
    std::string groupKey;
    std::vector<Vec2> points;
    bool detoured = false;
};

/// 运行期实体（平台与目标共用一套运动学）
struct Entity {
    std::string id;
    std::string deviceType;
    std::string kind;
    bool isTarget = false;
    std::string groupKey;
    std::string typeKey;      // 目标的 typeKey（仅观测用）
    double confidence = 1.0;  // 目标；仅透传
    int targetNo = 0;

    std::vector<Vec2> route;  // 局部平面航路（至少 1 点）
    std::vector<Vec2> taskRing;  // 任务区环（"到了没到"的判定用；空 = 不判定）
    std::size_t wp = 0;          // 当前航段起点下标（= arc 所在段）
    std::vector<double> arcCum;  // 航路累计里程表（与 route 等长；arcCum[i] = 到第 i 点的里程）
    double arc = 0.0;            // **里程参数**：位置由它唯一决定（回环取模、终点截断）
    Vec2 pos;
    double altM = 0.0;
    double speedNominal = 0.0;  // 巡航速度（m/s）
    double heading = 0.0;
    double battery = 100.0;
    double traveledM = 0.0;
    double routeLenM = 0.0;
    int64_t seq = 0;
    bool loop = false;
    bool spawned = true;        // popup 偏移未到 → false（不出现、不发事件）
    int64_t spawnAtMs = 0;      // 绝对仿真 epoch 毫秒
    bool arrived = false;
    bool visible = true;
    MotionState state = MotionState::Pending;

    // 区域归属（每次产出事件前重算；只读快照用）
    bool inTaskArea = false;
    bool inHardZone = false;
    bool inSoftZone = false;

    bool isPopup = false;
    int64_t popupOffsetMs = 0;

    // 传感器挂接（平台才有）
    std::string sensorId;
    double sensorRangeM = 0.0;
    int64_t sensorIntervalMs = 0;
    int64_t lastSenseMs = 0;
    bool hasSensor = false;
};

/// 运动学的推进结果（一个子步）
struct AdvanceOutcome {
    bool arrivedNow = false;
};

/// 里程定位结果：位置 + 所在段 + 段内偏移
struct PosAt {
    Vec2 p;
    double segPos = 0.0;
    double segLen = 0.0;
};

/// 建立航路累计里程表（弧长参数化的前提）
void buildArc(const std::vector<Vec2>& route, std::vector<double>& cum, double& total);

/// 由里程 `arc` 求位置（`loop` 时对总长取模；否则截断在终点）。
/// 段下标写入 `segIdx`（回环闭环段的下标 = route.size()-1）。
PosAt locate(const std::vector<Vec2>& route, const std::vector<double>& cum, double total,
             double arc, bool loop, std::size_t& segIdx);

/// 按 `dtMs` 推进一个实体（纯运动学；不含事件、不含 sink 调用）。
///
/// 口径：
///   · 到达航点则把**剩余距离**带入下一段（不丢里程，不跳时间）；
///   · 走完最后一段 → `arrived=true`、`speed=0`、位置停在终点；
///   · `loop=true` 的目标走完最后一段回到第 0 点继续；
///   · 电量按 `drainPctPerSec × dt` 递减，夹在 [0, 100]。
AdvanceOutcome advanceEntity(Entity& e, double dtSec, double drainPctPerSec);

/// 把实体的当前位姿整理成 `EntityPose`
EntityPose poseOf(const Entity& e, const LocalFrame& frame);

/// 由局部航路建运行期实体（`loop=true` 时自动闭环，见 engine.cc 的 loopClose）
Entity makeEntity(const std::string& id, const std::string& deviceType, const std::string& kind,
                  bool isTarget, const std::string& groupKey, const std::vector<Vec2>& route,
                  double altM, double speedMps, double battery, bool loop,
                  const LocalFrame& frame);

/// 环线闭环：补一条"从终点回到起点"的边（已闭合则不动）
void loopClose(std::vector<Vec2>& route);

/// 去掉连续重复点（零长度段会让"里程 → 位置"插值除零）
void dropDuplicatePoints(std::vector<Vec2>& route);

/// 场景 → 局部平面：收集 hard / soft 禁飞区
void collectZones(const SimScenario& s, const LocalFrame& frame, std::vector<Obstacle>& hard,
                  std::vector<Poly>& soft);

/// 部署区 / 任务区质心（局部平面）；不存在 → false
bool areaCentroidLocal(const SimScenario& s, const LocalFrame& frame, const std::string& key,
                       Vec2& out);

/// **规划核心**：从 `start` 到 `goal`，绕开 `hard`（已膨胀，`clearanceM` 为绕行点外扩）。
/// 失败（无路可走）时返回 false，`out` 保持直连（调用方负责告警）。
bool planDetour(const Vec2& start, const Vec2& goal, const std::vector<Obstacle>& hard,
                double clearanceM, std::vector<Vec2>& out);

/// 规划一条基航路（部署区 → 任务区），并给出是否绕行
bool planBaseRoute(const SimScenario& s, const LocalFrame& frame, const std::string& homeAreaKey,
                   const std::string& taskAreaKey, const std::vector<Obstacle>& hard,
                   double clearanceM, std::vector<Vec2>& out, bool& detoured);

/// 把基航路按编队槽位偏移成成员航路（保持相对位形）；`slot` 为 0 时原样返回
std::vector<Vec2> offsetRoute(const std::vector<Vec2>& base, const FormationSlot& slot);

/// 默认 formation 槽位（编组未声明位形时：同线跟随）
std::vector<FormationSlot> defaultFormation(int count);

/// 场景包围盒（WGS84）→ 局部平面参考点；空场景回落 (0,0)
LocalFrame frameFor(const SimScenario& s);

/// FNV-1a 64 位摘要（16 位小写十六进制）：同一份场景字节 → 同一 digest
std::string fnv1a64(const std::string& bytes);

}  // namespace detail
}  // namespace sim_source
