// sim-source · sim_source.h —— 唯一公开头文件
//
// 一句话边界（README「做 / 不做」）：
//   本模块只管「实体怎么动」；「传感器看见了什么」交 sensor-model；「数据怎么上路」交 device-ingest。
//   三者互不 import，靠注入的接口与中立结构对接。
//
// 权威依据（本文件不新增业务取值）：
//   · 本仓 README.md「做 / 不做」表 +「一句话边界」
//   · ../device-ingest/include/device_ingest/device_source.h —— 该文件把边界写死：
//       「模块只管把设备报文收进来、解析、归一、发出去；
//         "演示数据长什么样、任务进度怎么走"是主仓的业务模拟（BusinessSim）」
//     并给出反向接口 IDeviceSource。本模块正是**落实**那个边界：把它声明为"主仓业务模拟"
//     的那部分独立出来，接口形状逐字段对齐它的中立结构。
//   · ../device-ingest/docs/契约/外设接入契约.md §2 归一化对象
//       {deviceId, deviceType, kind, lng, lat, alt, heading, speed, battery, seq, ts}
//   · 参考版式（只读）：../resource-alloc（C++17 + CMake + include/src/examples/tests/scripts）
//
// 五条不可协商的口径（验收会逐条机检）：
//   1. **只吃中立结构**：入参是 SimScenario（区域 / 编组 / 平台 / 目标），
//      MUST NOT 查业务表、MUST NOT 出现 SQL、MUST NOT 出现任何业务取值。
//      边界声明：被禁止的取值包括"型号名、场景键，以及任何与阶段/评级相关的词"——
//      这些词只出现在这一行（作为**禁止清单**），实现里一个都没有。
//      deviceType / kind / role / 区域 key **一律由宿主注入**，引擎只当字符串用。
//   2. **吐归一化事件**：SimEvent 的 11 个字段与《外设接入契约》§2 逐字段同名同型，
//      主键序与契约同序（见 SimEvent::toJson）。
//   3. **MUST NOT 内置探测模型**：本模块只把平台位姿交给注入的 ISensorModel，
//      再把它回传的观测交给 sink。**接口在这里定义，算法不在这里**（没有发现概率、没有判定）。
//   4. **反向接口**：事件出口是 ISimSink（宿主实现）。本模块 MUST NOT 自己发 UDP、
//      MUST NOT 自己广播、MUST NOT 打开任何套接字。
//   5. **时间必须可注入**：仿真时钟只由 step()/tick() 推进；IClock 只在"驱动器缺省时间源"处
//      被读取一次（SimOptions::useClockWhenTickArgMissing）。同一输入序列 + 同一假时钟
//      → 事件序列逐字节一致（SIM-NFR-01）。
//
// 宿主只允许 #include <sim_source/sim_source.h>；其余头文件是内部实现细节。
#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace sim_source {

/// 本模块的 JSON 类型。
///
/// 为什么是 `ordered_json`（而不是 nlohmann 默认的 `json`）：
/// 归一化对象的键序与《外设接入契约》§2 同序是**可机检的契约**（验收脚本逐字段比对），
/// 且确定性要求"同输入双跑 → 逐字节一致"（SIM-NFR-01）。nlohmann 默认的 `json`
/// 以 `std::map` 为后端，会按键名字典序重排；`ordered_json`（`std::vector` 后端）保留插入
/// 顺序。两者取值语义、`dump()`、`parse()` 行为一致。
using json = nlohmann::ordered_json;

/// 模块版本（capabilities 自述用）
inline constexpr const char* kModuleVersion = "0.1.0";

// ============================================================================
// §1 归一化事件（出口形状 —— 逐字段等于 device-ingest 归一化对象）
// ============================================================================

/// 一条归一化事件。**形状冻结**：字段名与类型逐字段等于《外设接入契约》§2
/// `{deviceId, deviceType, kind, lng, lat, alt, heading, speed, battery, seq, ts}`。
///
/// 为什么本模块重新定义而不是 include `device_ingest/device_source.h`：
/// 跨仓 import MUST NOT 发生（README「三者互不 import」/ resource-alloc P1 同口径）。
/// 本结构只是那一个形状的**本地镜像**，不含任何业务取值；`kind` 由宿主注入。
///
/// 与 device-ingest 侧的关系：`SimEvent` 就是"设备上报"的**合成来源**。
/// 宿主把 `toJson()` 的结果（或转换后的 SimEvent）喂给 device-ingest 的
/// `NormalizedEvent` / 解析器入口即可，二者字段一一对应，无需映射表。
///
/// 可选字段（alt/heading/speed/battery/seq）的存在位：契约允许"字段缺失"，例如
/// 坐标非法时整字段丢弃。合成器**总是**算得出这些量，故默认全部存在；
/// 保留存在位是为了让宿主能表达"这次确实没有该量"，且让 JSON 往返逐字段可比对。
struct SimEvent {
    /// 可选字段的存在位（bit 取值见 kEvPresent* 常量）
    enum : unsigned {
        PresAlt = 1u << 0,
        PresHeading = 1u << 1,
        PresSpeed = 1u << 2,
        PresBattery = 1u << 3,
        PresSeq = 1u << 4,
        PresAll = PresAlt | PresHeading | PresSpeed | PresBattery | PresSeq,
    };

    // ---- MUST 字段（契约 §2）----
    std::string deviceId;    // 接入层内稳定唯一
    std::string deviceType;  // 设备类型（宿主注入；引擎不解释）
    std::string kind;        // 事件种类（宿主注入；引擎不解释）
    int64_t ts = 0;          // epoch 毫秒（仿真时间）

    // ---- 坐标（契约 §2：WGS84，[经度, 纬度] 惯例；越界则整字段缺失）----
    double lng = 0.0;
    double lat = 0.0;

    // ---- 可选字段 ----
    double alt = 0.0;        // 米
    double heading = 0.0;    // 0–360 度（北 0，顺时针）
    double speed = 0.0;      // m/s
    int battery = 0;         // 0–100 整数
    int64_t seq = 0;         // 设备序号（本模块按实体单调递增）

    unsigned presence = PresAll;

    // ---- 宿主扩展位（**契约 §2 之外的键**，默认不输出）----
    //
    // 契约 §2 里还有 3 个 MUST 字段（`tsSource` / `recvAt` / `source`）属于**接入层**语义：
    //   · `recvAt`（服务端到达时刻）只有 device-ingest 收到包的那一刻才知道；
    //   · `source`（接入点 id / peer / bytes）只有接入点才知道。
    // 合成器凭空"补"这些字段就是**臆造**：它会与接入层真实值冲突。
    // 因此本模块只负责 §2 的 11 字段，其余键由宿主在此显式注入（键按字典序稳定输出）。
    // 这一处是**待裁决问题 T1**（见 docs/实现报告.md §5）。
    json extensions = json::object();

    bool hasAlt() const { return (presence & PresAlt) != 0; }
    bool hasHeading() const { return (presence & PresHeading) != 0; }
    bool hasSpeed() const { return (presence & PresSpeed) != 0; }
    bool hasBattery() const { return (presence & PresBattery) != 0; }
    bool hasSeq() const { return (presence & PresSeq) != 0; }

    /// 序列化为归一化对象。**主键序与《外设接入契约》§2 同序**：
    /// deviceId / deviceType / kind / lng / lat / alt / heading / speed / battery / seq / ts
    /// （可选字段不存在时省略该键；extensions 追加在主键之后，按字典序）。
    json toJson() const;
};

/// 解析归一化对象（往返用）。未知字段**忽略**（对齐契约的降级约定）；
/// 缺 MUST 字段 → `ok=false` + 可读原因（不抛异常）。
struct EventParseResult {
    bool ok = false;
    std::string reason;
    SimEvent event;
};

EventParseResult parseSimEvent(const json& j);

/// 契约 §2 的 11 个字段名，**按契约键序**（验收脚本据此逐字段比对）。
const std::vector<std::string>& normalizedFieldOrder();

// ============================================================================
// §2 中立输入结构（**不是业务表**：没有表、没有外键、没有 SQL）
// ============================================================================
/// 区域角色。部署区 / 任务区 / 禁飞区**统一表达**，靠 role 区分——
/// 这样"区域"只有一种形状，宿主换场景不需要换结构（SIM-IN-01）。
///
/// `role` 的**取值**由宿主注入（引擎只做三件事：认 role、算几何、按 key 引用）。
/// 为免宿主每次都要造字符串，这里给三个**中立语义**的常量：
enum class AreaRole { Deploy, Task, NoFly };

/// 禁飞区硬度。只有 `hard` 参与绕行；`soft` 只记录、只报告，不改变航路（SIM-ROUTE-02）。
enum class Hardness { Hard, Soft };

/// 一个区域：key 唯一；polygon 为 WGS84 `[lng, lat]` 顶点环（首尾不重复，按序闭合）。
struct Area {
    std::string key;
    std::string name;
    AreaRole role = AreaRole::NoFly;
    std::vector<std::pair<double, double>> polygon;
    Hardness hardness = Hardness::Hard;
    bool hasHardness = false;  // 是否显式给了 `hardness`

    double centroidLng() const;
    double centroidLat() const;
};

/// 一个编组（编队）。成员相对**领队**保持位形（SIM-ROUTE-03）：
/// 领队 = 该编组中 deviceId 字典序最小的平台（去掉对平台声明顺序的依赖，保证可复现）。
struct Group {
    std::string key;
    std::string name;
    std::string role;  // 宿主语义（引擎不解释）
};

/// 「编队位形」的一个槽位：编组内第 n 个成员相对领队的右向 / 前向偏移（米）。
///
/// 索引 0 = 领队自身（偏移应为 0）。**成员按 deviceId 字典序取槽位**（去掉对声明顺序的
/// 依赖，保证可复现），槽位不足时回落 0（= 与领队同线）。
///
/// 语义（SIM-ROUTE-03）：偏移是**刚体平移**，方向基准是**整条航路的初始航向**
/// （= 基航路第一段的方向）。这样"相对位形"逐航点恒定、可机检；代价是编队不随航向
/// 转弯而旋转（要转弯编队请给出自己的航路）。**待裁决问题 T3** 见 docs/实现报告.md §5。
struct FormationSlot {
    double rightM = 0.0;  // 右向偏移（+ = 初始航向的右手侧）
    double fwdM = 0.0;    // 前向偏移（+ = 初始航向的前方）
};

/// 一个平台（= 会动的实体之一：无人机 / 车辆 / 舰船……类型由宿主注入）。
struct Platform {
    std::string deviceId;
    std::string deviceType;  // 宿主注入（如 "uav"）；引擎不解释
    std::string kind;        // 事件 kind（宿主注入）；空 = SimOptions::defaultKind
    std::string groupKey;    // 空 = 不参与编队，单独成组
    std::string homeAreaKey; // 部署区
    std::string taskAreaKey; // 任务区
    double altM = 0.0;       // 巡航高度（米）
    double speedMps = 0.0;   // 巡航速度（m/s）；<= 0 → 用 SimOptions::defaultSpeedMps
    double battery = 100.0;  // 初始电量 0–100
    /// 出发点相对部署区质心的偏移（米）：让编队在地面就摆好位形。
    FormationSlot startOffset;
};

/// 目标的运动方式（冻结三取值 —— SIM-TGT-01）。
enum class TargetMotion {
    Static,   // "static"  ：原地不动
    Dynamic,  // "dynamic" ：按 route + speedMps 走航线，loop 决定是否循环
    Popup,    // "popup"   ：在 startOffsetMs 之后才出现，出现后按 route 运动
};

/// 一个目标（= 会动的实体之二）。
///
/// `route` 为空时：`position` 若给了 → 原地静止；否则回落 `startAreaKey` 的质心
/// （再否则 = 原点，并在 validate 里报可读问题）。
struct Target {
    int no = 0;              // 目标编号（宿主语义；进不了归一化事件，扩展位可带）
    std::string id;          // 事件 deviceId；空 = 由 no 生成
    std::string typeKey;     // 宿主注入的类型键（引擎不解释）
    std::string deviceType;  // 事件 deviceType；空 = SimOptions::defaultTargetDeviceType
    std::string kind;        // 事件 kind；空 = SimOptions::defaultKind
    std::string name;
    TargetMotion motion = TargetMotion::Static;

    bool hasPosition = false;                 // 是否显式给了 `position`
    std::pair<double, double> position{0, 0}; // WGS84 起点
    std::vector<std::pair<double, double>> route;  // WGS84 航路（含起点）
    double speedMps = 0.0;
    bool loop = false;
    int64_t startOffsetMs = 0;  // popup：相对仿真时间 0 的出现偏移
    double confidence = 1.0;    // 宿主语义；本模块只透传进扩展位（**不做任何判定**）
};

/// 中立场景（**唯一的输入**）。引擎不认识任何具体取值。
struct SimScenario {
    std::string scenarioKey;  // 宿主语义（引擎不解释）
    std::vector<Area> areas;
    std::vector<Group> groups;
    std::vector<Platform> platforms;
    std::vector<Target> targets;

    const Area* findArea(const std::string& key) const;
};

// ============================================================================
// §3 反向接口（宿主 MUST 实现；模块 MUST NOT 自带业务实现）
// ============================================================================

// 观测与位姿形状在 §4 定义；接口先声明，故此处前置声明
struct SimObservation;
struct SensorPose;
struct EntityPose;

/// 仿真时钟注入（epoch 毫秒）。
///
/// **口径**：仿真时间**只**由 `SimSource::step()` / `tick()` 推进，永远不取挂钟。
/// 本接口只在"驱动器没给时间参数"时被读一次（`tick()` 无参重载 + SimOptions 开关），
/// 使真实时间驱动器可注入而自测仍可完全复现（SIM-NFR-01）。
///
/// 默认实现 SystemClock **不实现**（见 §7）：本模块不提供取挂钟的实现体，
/// 由宿主注入；这与 resource-alloc 的 SystemClock 不同是**故意的**——
/// 少了它，任何人都不可能无意中把挂钟带进仿真回路。
class IClock {
public:
    virtual ~IClock() = default;
    virtual int64_t nowMs() const = 0;
};

/// 事件出口。**MUST 立即返回**（不在调用路径上做网络 IO / 等锁 / 落库）。
///
/// 两类事件：
///   · `onEvent`      —— 归一化事件（形状见 §1；kind 由宿主注入）。
///                      宿主若走 device-ingest，就把它当作"设备刚上报的一包"喂进去。
///   · `onObservation`—— 传感器观测事件（形状见 SimObservation）。
///                      它是**探测结果**，不是设备上报；进不进 device-ingest 由宿主决定。
class ISimSink {
public:
    virtual ~ISimSink() = default;
    virtual void onEvent(const SimEvent& e) = 0;
    virtual void onObservation(const SimObservation& o) = 0;
};

/// 传感器模型的**抽象接口**（本模块只定义、只调用，MUST NOT 实现 —— README「不做」列
/// 「不内置探测模型（交 sensor-model）」）。
///
/// 契约：
///   · 引擎每 tick 在有挂接且观测间隔到期时调用一次；
///   · `pose.platformId` 是**探测平台**，`pose.candidates` 是"此刻可见的实体位姿"
///     （目标 + 平台；执行者已剔除，避免自我探测）。候选集是**穷举**的：
///     谁被发现了由 ISensorModel 判定，本模块**不做任何可见性/概率/距离筛选**。
///   · 返回的每条观测由 sink 的 `onObservation` 发出；引擎不解释其内容。
///   · 抛异常 → 计入 metrics.sensorErrors，仿真继续（MUST NOT 让一次探测失败停掉时间推进）。
class ISensorModel {
public:
    virtual ~ISensorModel() = default;
    virtual std::string id() const = 0;          // 如宿主注册的模型键（引擎不解释）
    virtual std::string deviceType() const = 0;  // 该模型服务的设备类型（引擎不解释）
    /// 探测：一次调用返回 0..N 条观测。**无状态、可重入**（与 device-ingest 的 IParser 同口径）。
    virtual std::vector<SimObservation> sense(const SensorPose& pose) = 0;
};

/// 把 ISensorModel 挂到某个平台。`intervalMs` = 调用间隔（仿真时间毫秒），<= 0 → 每 tick。
struct SensorAttachment {
    std::string platformId;
    std::string sensorId;  // 接入层里这个"传感器"的稳定 id（宿主注入）
    double rangeM = 0.0;   // 0 = 不筛（引擎本就不筛；此字段只作为事实透传给模型）
    int64_t intervalMs = 0;
};

// ============================================================================
// §4 观测与位姿（ISensorModel 的入参 / 出参）
// ============================================================================

/// 一个实体的位姿快照（交出去的事实，不含任何判定）。
struct EntityPose {
    std::string platformId;  // 实体 id（平台或目标）
    std::string deviceType;  // 宿主注入
    double lng = 0.0;
    double lat = 0.0;
    double altM = 0.0;
    double heading = 0.0;
    double speedMps = 0.0;
    bool isTarget = false;  // 目标（true）/ 平台（false）
};

/// 一次探测调用的入参：探测平台位姿 + 此刻可见的候选实体位姿（穷举）。
struct SensorPose {
    std::string sensorId;
    std::string deviceId;    // = attachment.platformId（兼容接入层的 deviceId 口径）
    std::string deviceType;
    double rangeM = 0.0;     // 透传 attachment.rangeM
    int64_t ts = 0;          // 仿真时间
    EntityPose self;                        // 探测平台自身位姿
    std::vector<EntityPose> candidates;     // 可见候选（不含 self）
};

/// 传感器观测事件（**由 ISensorModel 产出，本模块只转发**）。
///
/// `targetId` 是被发现实体的 id，`sensorId` 是**做出这次观测的传感器**——
/// 二者名字刻意不同：归一化对象的 `deviceId` 是"谁在上报"，
/// 若把目标 id 填进去，device-ingest 侧会把目标当成一台设备，台账就串了。
struct SimObservation {
    std::string sensorId;    // 上报方（传感器）
    std::string deviceType;  // 上报方类型（宿主注入）
    std::string kind;        // 观测种类（宿主注入；引擎不解释）
    std::string targetId;    // 被发现实体
    std::string targetType;  // 被发现实体类型（宿主注入）
    double lng = 0.0;
    double lat = 0.0;
    double altM = 0.0;
    double heading = 0.0;
    double speedMps = 0.0;
    double confidence = 0.0;  // 模型自报置信度（引擎不解释、不校验、不判定）
    int64_t ts = 0;
    json extensions = json::object();  // 模型自定义字段透传位

    json toJson() const;
};

// ============================================================================
// §5 结果与自述
// ============================================================================

struct ScenarioIssue {
    std::string path;    // 如 `platforms[2].homeAreaKey`
    std::string field;
    std::string reason;
};

/// 场景校验结果（**纯校验**，不改状态；不抛异常）
struct ValidationResult {
    bool ok = true;
    std::vector<ScenarioIssue> issues;
    json toJson() const;
};

/// 静态路径规划的结果（**规划与运行解耦**：宿主可只规划不运行）
struct PlanResult {
    bool ok = false;
    std::string message;
    std::vector<std::string> platformIds;                        // 与 routes 同序
    std::vector<std::vector<std::pair<double, double>>> routes;  // 逐平台 waypoints（WGS84）
    std::vector<std::string> groupKeys;
    int hardZones = 0;      // 参与绕行的 hard 禁飞区数
    int softZones = 0;      // 只记录不绕行的 soft 禁飞区数
    int detoured = 0;       // 需要绕行的平台数
    std::vector<std::string> warnings;

    /// 便捷取值（不存在 → 空）
    const std::vector<std::pair<double, double>>* routeOf(const std::string& platformId) const;
    json toJson() const;
};

/// 节拍控制：`setSpeed` 只接受这三个倍率（README「节拍控制：实时 / 倍速（1× 8× 60×）」）
std::vector<int> allowedSpeedMultipliers();

/// 实体运行状态（冻结取值）
enum class MotionState {
    Enroute,   // 在航路上
    Arrived,   // 到达任务区（航路走完）
    Holding,   // 暂停中（节拍 pause）
    Pending,   // 还没出现（popup 偏移未到）
};

/// 一个实体的快照（查询用；**不是事件**）
struct EntityStatus {
    std::string id;
    std::string deviceType;
    std::string kind;
    bool isTarget = false;
    std::string groupKey;
    MotionState state = MotionState::Pending;
    double lng = 0.0;
    double lat = 0.0;
    double altM = 0.0;
    double heading = 0.0;
    double speedMps = 0.0;
    double battery = 0.0;
    int64_t seq = 0;
    int waypointIndex = 0;  // 当前航段起点下标
    bool visible = false;   // 是否已出现（popup 偏移前 = false）
    bool inTaskArea = false;    // 是否已在任务区内（SIM-ROUTE-05）
    bool inHardZone = false;    // 是否落在 hard 禁飞区内（**MUST 恒为 false**，SIM-ROUTE-01）
    bool inSoftZone = false;    // 是否落在 soft 禁飞区内（只报告）
};

/// 注入项：**只有依赖 + 工程口径**，没有业务配置（业务全在 SimScenario）
struct SimOptions {
    /// 倍速：1 / 8 / 60（SIM-TICK-02）。非法值由 setSpeed 拒绝。
    int speedMultiplier = 1;
    /// `tick()` 的仿真步长子步（毫秒）：一次大 dt 会被切成若干子步推进，
    /// 使"到达航点 / 拐弯 / 绕过角点"不因步长而失真（SIM-NFR-05）。
    int64_t substepMs = 100;
    /// 电量耗率（%/秒，仿真时间）：100 % 满电按此耗率放电。
    /// **不是业务取值**：它是运动学口径，宿主可覆盖（SIM-KIN-04）。
    double batteryDrainPctPerSec = 0.01;
    /// 平台 speedMps <= 0 时的回落速度（m/s）
    double defaultSpeedMps = 30.0;
    /// 平台 kind 为空时的回落 kind（**中立占位**，宿主应显式注入自己的 kind）
    std::string defaultKind = "sim.pos";
    /// 目标 deviceType 为空时的回落类型
    std::string defaultTargetDeviceType = "sim";
    /// 绕行缓冲（米）：hard 禁飞区多边形外扩此距离后再规划（SIM-ROUTE-01）
    double safetyMarginM = 50.0;
    /// `tick(nowMs)` 缺参且无注入时钟时是否回落挂钟。**默认 false**：
    /// 缺参 = 不推进（可复现优先）。宿主若要真实时间驱动，请注入 IClock 并把此项置 true。
    bool useClockWhenTickArgMissing = false;
};

/// 运行计数（观测口径）
struct Metrics {
    int64_t ticks = 0;            // tick() 调用次数（含未推进的）
    int64_t steps = 0;            // step() 调用次数
    int64_t substeps = 0;         // 实际推进的子步数
    int64_t eventsEmitted = 0;    // onEvent 调用次数
    int64_t observationsEmitted = 0;  // onObservation 调用次数
    int64_t sensorCalls = 0;      // ISensorModel::sense 调用次数
    int64_t sensorErrors = 0;     // sense 抛异常次数（吞掉并计数）
    int64_t sinkErrors = 0;       // sink 回调抛异常次数（吞掉并计数）
    int64_t pausedMs = 0;         // 暂停期间被丢弃的真实时间（毫秒）
    int64_t simElapsedMs = 0;     // 仿真已推进的总时长
    int64_t popupSpawned = 0;     // popup 目标已出现的次数
    int64_t arrivals = 0;         // 到达任务区的实体次数
};

/// 自述（capabilities 口径）
struct Capabilities {
    bool initialized = false;
    bool clockInjected = false;
    bool sinkInjected = false;
    bool customSensorInjected = false;
    int platforms = 0;
    int targets = 0;
    int groups = 0;
    int areas = 0;
    int hardZones = 0;
    int sensors = 0;
    int speedMultiplier = 1;
    bool paused = false;
    int64_t simElapsedMs = 0;
    std::string moduleVersion = kModuleVersion;
    std::string scenarioKey;
};

// ============================================================================
// §6 引擎
// ============================================================================

/// 仿真源：编队航路规划 + 实体运动学 + 时间推进 + 归一化事件产出。
///
/// 线程模型：**单线程**（宿主在自己的驱动线程里调 tick/step）。
/// 本模块不创建线程、不取挂钟、不开套接字（README「不做」列）。
class SimSource {
public:
    SimSource();
    explicit SimSource(const SimOptions& opts);
    ~SimSource();
    SimSource(const SimSource&) = delete;
    SimSource& operator=(const SimSource&) = delete;
    SimSource(SimSource&&) noexcept;
    SimSource& operator=(SimSource&&) noexcept;

    // ---- 初始化与查询 ----

    /// 校验场景（**纯函数**，不依赖实例状态）：宿主可先体检再 init。
    static ValidationResult validate(const SimScenario& scenario, const SimOptions& opts = SimOptions());
    /// 载入场景：规划航路（含绕行 hard 禁飞区）并生成实体。失败时保留上一次成功的局面。
    ValidationResult init(const SimScenario& scenario);
    /// 当前场景（拷贝；只读用途）
    SimScenario scenario() const;
    /// 静态规划结果（与 init 用的是同一条规划路径）
    const PlanResult& plan() const;

    // ---- 节拍控制（SIM-TICK）----

    /// 倍速：**只接受 1 / 8 / 60**（其它值返回 false，且不改变现状）。
    bool setSpeed(int multiplier);
    int speedMultiplier() const;
    /// 暂停：时间不再推进（tick 丢弃真实时间、step 不推进），也不产出事件。
    void pause();
    /// 恢复：从此刻重新计时（暂停期间的真实时间**不计入**，故不会"跳一下"）。
    void resume();
    bool paused() const;

    /// 单步：推进固定 `dtMs` **仿真毫秒**（与倍速无关）。返回本次发出的归一化事件数。
    /// 这是自测与离线复算用的入口（确定性最强：与真实时间完全无关）。
    int step(int64_t dtMs);

    /// 时间推进：用**真实时间** `nowMs`（epoch 毫秒）驱动。
    /// 本次推进的仿真时间 = (nowMs - 上次 nowMs) × speedMultiplier，再按 substepMs 切子步。
    /// 返回本次发出的归一化事件数。首次调用只记基线、不推进。
    int tick(int64_t nowMs);
    /// 同上，时间参数缺失时按 SimOptions 口径处理（默认：不推进）。
    int tick();

    /// 注入时钟（可空）。只在上面那个缺参重载里被读一次。
    void setClock(std::shared_ptr<IClock> clock);
    /// 注入事件出口（可空 → 事件被丢弃，但 metrics 仍计数）。
    void setSink(std::shared_ptr<ISimSink> sink);
    /// 挂接传感器模型（可空 → 不调用、不产出观测）。
    void setSensorModel(std::shared_ptr<ISensorModel> model);
    /// 挂接清单（覆盖式设置）
    void setSensorAttachments(const std::vector<SensorAttachment>& attachments);

    // ---- 只读观测 ----

    int64_t simElapsedMs() const;                    // 相对仿真时间 0 的偏移
    int64_t simNowMs() const;                        // 仿真 epoch 毫秒
    std::vector<EntityStatus> entities() const;      // 全部实体（含未出现的 popup）
    bool entity(const std::string& id, EntityStatus& out) const;
    std::vector<std::string> deviceIds() const;
    Metrics metrics() const;
    Capabilities capabilities() const;
    /// 把全部实体当前状态序列化为 JSON 数组（示例/排障用）
    json entitiesJson() const;

    // ---- 几何与规划自由函数（纯函数，宿主/测试可直接调用）----

    /// ● 是否落在某区域内（含边界）。多边形按射线法在**局部平面**上判定。
    static bool containsPoint(const std::vector<std::pair<double, double>>& polygon, double lng,
                              double lat);
    /// ● 两点方位角（度，0–360，北 0 顺时针）
    static double headingBetween(double lng1, double lat1, double lng2, double lat2);
    /// ● 两点距离（米）：局部等距圆柱投影（webMap 尺度下与 haversine 同量级）
    static double distanceMeters(double lng1, double lat1, double lng2, double lat2);
    /// ● 折线总长（米）
    static double routeLengthMeters(const std::vector<std::pair<double, double>>& route);
    /// ● 线段是否**穿入**某多边形（含边界接触）—— 绕行断言的判定口径（SIM-ELC-01）
    static bool segmentCrossesPolygon(const std::pair<double, double>& a,
                                      const std::pair<double, double>& b,
                                      const std::vector<std::pair<double, double>>& polygon);
    /// ● 折线是否**从未**穿入任何 hard 多边形（SIM-ELC-01）
    static bool routeClearOfHardZones(const std::vector<std::pair<double, double>>& route,
                                      const SimScenario& scenario);
    /// ● 中立场景 → JSON（示例与宿主配置转换用；未知字段忽略）
    static json toJson(const SimScenario& scenario);
    /// ● JSON → 中立场景（**不抛异常**；结构与取值问题进 issues）
    static ValidationResult fromJson(const json& j, SimScenario& out);

    /// ● 规划（纯函数）：部署区 → 任务区，硬禁飞区绕行，编队保持位形。
    /// 与 `init()` 走**同一条**实现路径 —— 不存在"示例能绕、引擎不绕"的分叉。
    static PlanResult planRoutes(const SimScenario& scenario, const SimOptions& opts = SimOptions());

    // PIMPL：公开头保持稳定，实现细节（src/internal.h）不外泄。
    struct Impl;

    /// 内部：用**调用方给定的局部平面**规划（保证规划与运行期实体在同一坐标系）。
    /// 局部平面类型属实现细节，故此处是模板；宿主用上面的纯函数版本即可。
    /// 传错类型不是运行时问题而是**链接错误**（比静默错位安全）。
    template <typename Frame>
    static PlanResult planRoutesInFrame(const SimScenario& scenario, const SimOptions& opts,
                                        const Frame& frame);

private:
    std::unique_ptr<Impl> impl_;
};

// ============================================================================
// §7 枚举 ⇄ JSON 字符串（取值冻结；非法取值 → 空串 / nullopt，MUST NOT 猜）
// ============================================================================

const char* toString(AreaRole r);
const char* toString(Hardness h);
const char* toString(TargetMotion m);
const char* toString(MotionState s);
std::optional<AreaRole> areaRoleFromString(const std::string& s);
std::optional<Hardness> hardnessFromString(const std::string& s);
std::optional<TargetMotion> targetMotionFromString(const std::string& s);

// JSON 序列化（camelCase；键序与成员声明顺序一致）
json toJson(const Area& v);
json toJson(const Group& v);
json toJson(const FormationSlot& v);
json toJson(const Platform& v);
json toJson(const Target& v);
json toJson(const ScenarioIssue& v);
json toJson(const EntityPose& v);
json toJson(const SensorPose& v);
json toJson(const SimObservation& v);
json toJson(const SimEvent& v);  // == SimEvent::toJson()
json toJson(const EntityStatus& v);
json toJson(const Metrics& v);
json toJson(const Capabilities& v);

}  // namespace sim_source
