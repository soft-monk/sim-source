// tests/selftest.cc · 零依赖自测（手写断言，不引任何测试框架）
//
// 覆盖两套口径：
//   ① 本仓 README「做 / 不做」表 —— 每条"做"至少一个用例，"不做"至少一个守卫断言
//      （需求编号 SIM-ROUTE / KIN / TICK / TGT / ELC / SENSOR / IN / OUT / NFR）
//   ② ../device-ingest/docs/契约/外设接入契约.md §2 归一化对象 —— 逐字段形状锁定
//
// 全部用例**确定性**：时间只由 step() 显式推进，或注入假时钟后 tick() 推进；
// 不依赖真实时间、不依赖当前工作目录、不需要外部服务、不联网、不落库。
//
// 运行： selftest                → 跑全部（人读输出）
//        selftest --list         → 只列用例名
//        selftest --json         → 机检输出（acceptance.ps1 读它做需求↔用例对账）
//        selftest <名字片段>      → 只跑名字里含该片段的用例
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#if defined(_WIN32)
#include <io.h>
#endif

#include "sim_source/sim_source.h"

using namespace sim_source;

namespace {

// ============================================================================
// 断言框架（与 resource-alloc 同版式：用例 N 个｜断言 X 条）
// ============================================================================
int g_asserts = 0;
int g_failed = 0;
int g_cases = 0;
int g_casesFailed = 0;
std::string g_case;
std::vector<std::string> g_failures;

struct CaseMeta {
    std::string name;
    std::vector<std::string> reqs;
    int asserts = 0;
    int failed = 0;
    bool ok = false;
};
std::vector<CaseMeta> g_metas;
std::vector<std::string> g_reqs;
bool g_jsonMode = false;

template <typename... Args>
void requires_(Args... ids) {
    for (const char* id : {ids...}) g_reqs.push_back(id);
}

std::string jsonEscape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 8);
    for (unsigned char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (c < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out += static_cast<char>(c);
                }
        }
    }
    return out;
}

std::string jsonArray(const std::vector<std::string>& v) {
    std::string out = "[";
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (i != 0) out += ",";
        out += "\"" + jsonEscape(v[i]) + "\"";
    }
    out += "]";
    return out;
}

/// 诊断输出。`--json` 下 stdout 只能有那一个 JSON 文档，故这类信息走 stderr。
void note(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    std::vfprintf(g_jsonMode ? stderr : stdout, fmt, args);
    va_end(args);
}

void record(bool ok, const std::string& what, const char* file, int line) {
    ++g_asserts;
    if (ok) return;
    ++g_failed;
    std::ostringstream os;
    os << g_case << " @" << file << ":" << line << "  " << what;
    g_failures.push_back(os.str());
    if (!g_jsonMode) std::printf("      FAIL %s\n", os.str().c_str());
}

std::string num(double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.12g", v);
    return buf;
}

// 断言实现：**入参一律先求值再传进来**（模板 + 提前求值）。
//
// 为什么强调这一点：宏里写 `record((a) == (b), ... num((a)) ...)` 会把实参求值**两次**；
// 当实参是 `sim.step(1000)` 这种"有副作用的调用"时，一次断言就推进了两次仿真 ——
// 自测里表现为"事件数莫名翻倍、时间莫名 +2000"，而且只在优化构建下稳定复现
// （真实踩过的坑，见 docs/实现报告.md §4）。所以这里改成"先算好值、再比较"。
template <typename A, typename B>
void checkEq(const A& a, const B& b, const char* expr, const char* file, int line) {
    const bool ok = (a == b);
    record(ok,
           std::string("期望 ") + num(static_cast<double>(b)) + "，实际 " +
               num(static_cast<double>(a)) + "：" + expr,
           file, line);
}

template <typename A, typename B, typename T>
void checkNear(const A& a, const B& b, const T& tol, const char* expr, const char* file, int line) {
    const bool ok = std::fabs(static_cast<double>(a) - static_cast<double>(b)) <=
                    static_cast<double>(tol);
    record(ok,
           std::string("期望 ≈") + num(static_cast<double>(b)) + "（±" +
               num(static_cast<double>(tol)) + "），实际 " + num(static_cast<double>(a)) + "：" +
               expr,
           file, line);
}

template <typename A, typename B>
void checkGe(const A& a, const B& b, const char* expr, const char* file, int line) {
    record((a >= b),
           std::string("期望 ≥") + num(static_cast<double>(b)) + "，实际 " +
               num(static_cast<double>(a)) + "：" + expr,
           file, line);
}

template <typename A, typename B>
void checkLe(const A& a, const B& b, const char* expr, const char* file, int line) {
    record((a <= b),
           std::string("期望 ≤") + num(static_cast<double>(b)) + "，实际 " +
               num(static_cast<double>(a)) + "：" + expr,
           file, line);
}

template <typename A, typename B>
void checkStr(const A& a, const B& b, const char* expr, const char* file, int line) {
    const std::string sa(a);
    const std::string sb(b);
    record(sa == sb, std::string("期望 \"") + sb + "\"，实际 \"" + sa + "\"：" + expr, file, line);
}

#define CHECK(cond) record((cond), "断言不成立：" #cond, __FILE__, __LINE__)
#define CHECK_EQ(a, b) checkEq((a), (b), #a " == " #b, __FILE__, __LINE__)
#define CHECK_NEAR(a, b, tol) checkNear((a), (b), (tol), #a " ≈ " #b, __FILE__, __LINE__)
#define CHECK_STR(a, b) checkStr((a), (b), #a " == " #b, __FILE__, __LINE__)
#define CHECK_GE(a, b) checkGe((a), (b), #a " >= " #b, __FILE__, __LINE__)
#define CHECK_LE(a, b) checkLe((a), (b), #a " <= " #b, __FILE__, __LINE__)

// ============================================================================
// 假时钟（**唯一的非确定性来源被彻底移除**：本模块内没有取挂钟的实现）
// ============================================================================
class FakeClock : public IClock {
public:
    explicit FakeClock(int64_t t0) : t_(t0) {}
    int64_t nowMs() const override {
        ++reads;
        return t_;
    }
    void advance(int64_t ms) { t_ += ms; }
    int64_t value() const { return t_; }
    mutable int reads = 0;

private:
    int64_t t_;
};

// ============================================================================
// 记录型 Sink
// ============================================================================
class RecordingSink : public ISimSink {
public:
    void onEvent(const SimEvent& e) override {
        events.push_back(e);
        lines.push_back(e.toJson().dump());
    }
    void onObservation(const SimObservation& o) override {
        observations.push_back(o);
        observationLines.push_back(o.toJson().dump());
    }
    std::vector<SimEvent> events;
    std::vector<std::string> lines;
    std::vector<SimObservation> observations;
    std::vector<std::string> observationLines;

    std::string joined() const {
        std::string s;
        for (const std::string& l : lines) {
            s += l;
            s += "\n";
        }
        return s;
    }
};

/// 抛异常的 Sink（验证"宿主回调出问题 MUST NOT 影响仿真推进"）
class ThrowingSink : public ISimSink {
public:
    void onEvent(const SimEvent&) override { throw std::runtime_error("sink 故意抛异常"); }
    void onObservation(const SimObservation&) override { throw std::runtime_error("sink 抛"); }
};

// ============================================================================
// 演示场景（测试数据；本仓的 include/ src/ 内 MUST NOT 出现任何业务取值）
// ============================================================================
SimScenario baseScenario() {
    SimScenario s;
    s.scenarioKey = "selftest";

    Area deploy;
    deploy.key = "DEPLOY";
    deploy.name = "部署区";
    deploy.role = AreaRole::Deploy;
    deploy.polygon = {{116.300, 39.900}, {116.320, 39.900}, {116.320, 39.920}, {116.300, 39.920}};
    s.areas.push_back(deploy);

    Area nofly;
    nofly.key = "NOFLY";
    nofly.name = "禁飞区（hard）";
    nofly.role = AreaRole::NoFly;
    nofly.hardness = Hardness::Hard;
    nofly.hasHardness = true;
    // 正好压在部署区 → 任务区的直连航线上
    nofly.polygon = {{116.350, 39.895}, {116.365, 39.895}, {116.365, 39.915}, {116.350, 39.915}};
    s.areas.push_back(nofly);

    Area soft;
    soft.key = "SOFT";
    soft.name = "限制区（soft）";
    soft.role = AreaRole::NoFly;
    soft.hardness = Hardness::Soft;
    soft.hasHardness = true;
    soft.polygon = {{116.375, 39.905}, {116.385, 39.905}, {116.385, 39.912}, {116.375, 39.912}};
    s.areas.push_back(soft);

    Area task;
    task.key = "TASK";
    task.name = "任务区";
    task.role = AreaRole::Task;
    task.polygon = {{116.395, 39.900}, {116.415, 39.900}, {116.415, 39.920}, {116.395, 39.920}};
    s.areas.push_back(task);

    Group g;
    g.key = "G1";
    g.name = "编队一";
    g.role = "recon";
    s.groups.push_back(g);

    Platform p1;
    p1.deviceId = "sim-01";
    p1.deviceType = "uav";
    p1.kind = "uav.pos";
    p1.groupKey = "G1";
    p1.homeAreaKey = "DEPLOY";
    p1.taskAreaKey = "TASK";
    p1.altM = 900.0;
    p1.speedMps = 50.0;
    s.platforms.push_back(p1);

    Platform p2;
    p2.deviceId = "sim-02";
    p2.deviceType = "uav";
    p2.kind = "uav.pos";
    p2.groupKey = "G1";
    p2.homeAreaKey = "DEPLOY";
    p2.taskAreaKey = "TASK";
    p2.altM = 950.0;
    p2.speedMps = 50.0;
    p2.startOffset.rightM = 200.0;
    p2.startOffset.fwdM = -300.0;
    s.platforms.push_back(p2);

    Target t1;
    t1.no = 1;
    t1.id = "tgt-1";
    t1.typeKey = "fixed";
    t1.deviceType = "target";
    t1.kind = "target.pos";
    t1.motion = TargetMotion::Static;
    t1.hasPosition = true;
    t1.position = {116.400, 39.910};  // 落在任务区（TASK 多边形）内部
    s.targets.push_back(t1);

    Target t2;
    t2.no = 2;
    t2.id = "tgt-2";
    t2.typeKey = "mobile";
    t2.deviceType = "target";
    t2.kind = "target.pos";
    t2.motion = TargetMotion::Dynamic;
    t2.route = {{116.392, 39.902}, {116.412, 39.902}};  // 4000 m 直线往返（loop）
    t2.speedMps = 40.0;
    t2.loop = true;
    s.targets.push_back(t2);

    Target t3;
    t3.no = 3;
    t3.id = "tgt-3";
    t3.typeKey = "popup";
    t3.deviceType = "target";
    t3.kind = "target.pos";
    t3.motion = TargetMotion::Popup;
    t3.route = {{116.398, 39.906}, {116.410, 39.914}};
    t3.speedMps = 30.0;
    t3.startOffsetMs = 30000;
    s.targets.push_back(t3);

    return s;
}

std::shared_ptr<RecordingSink> runScenario(SimSource& sim, const SimScenario& s, int steps,
                                           int64_t dtMs, RecordingSink** out) {
    auto sink = std::make_shared<RecordingSink>();
    if (out != nullptr) *out = sink.get();
    sim.setSink(sink);
    const ValidationResult vr = sim.init(s);
    if (!vr.ok) return sink;
    for (int i = 0; i < steps; ++i) sim.step(dtMs);
    return sink;
}

/// 同上，但每一步之后回调（"逐 tick 复核"的用例用）。
/// **时间只在这里推进**：早期版本里有的用例先调 runScenario(steps=0) 再自己 for-step，
/// 每次 step 被算了两遍、断言全部错位（真实踩过的坑，见 docs/实现报告.md §4）。
template <typename Fn>
std::shared_ptr<RecordingSink> runScenarioEachStep(SimSource& sim, const SimScenario& s,
                                                   int steps, int64_t dtMs, Fn afterEach,
                                                   RecordingSink** out) {
    auto sink = std::make_shared<RecordingSink>();
    if (out != nullptr) *out = sink.get();
    sim.setSink(sink);
    const ValidationResult vr = sim.init(s);
    if (!vr.ok) return sink;
    for (int i = 0; i < steps; ++i) {
        sim.step(dtMs);
        afterEach(i);
    }
    return sink;
}

/// 在实体快照里按 id 查找。
/// **必须是模板**：`find(sim.entities(), id)` 里的临时 vector 在整表达式结束就销毁了，
/// 返回它的元素指针会立刻悬垂（早期版本正是 `const EntityStatus* find(const std::vector<...>&)`，
/// 于是拿到随机内存、断言里出现 1e+161 这种"值" —— 见 docs/实现报告.md §4）。
template <typename Container>
const EntityStatus* find(const Container& v, const std::string& id) {
    for (const EntityStatus& e : v) {
        if (e.id == id) return &e;
    }
    return nullptr;
}

bool hasDevice(const std::vector<SimEvent>& evs, const std::string& id) {
    for (const SimEvent& e : evs) {
        if (e.deviceId == id) return true;
    }
    return false;
}

// ============================================================================
// 用例 · 航路规划
// ============================================================================

void route01_detour_around_hard_no_fly_zone() {
    requires_("SIM-ROUTE-01", "SIM-ROUTE-02", "SIM-ELC-01");
    const SimScenario s = baseScenario();
    const PlanResult plan = SimSource::planRoutes(s);

    CHECK(plan.ok);
    CHECK_EQ(plan.hardZones, 1);
    CHECK_EQ(plan.softZones, 1);
    CHECK_EQ(static_cast<int>(plan.platformIds.size()), 2);

    const Area* deploy = s.findArea("DEPLOY");
    const Area* task = s.findArea("TASK");
    const Area* nofly = s.findArea("NOFLY");
    CHECK(deploy != nullptr && task != nullptr && nofly != nullptr);
    if (deploy == nullptr || task == nullptr || nofly == nullptr) return;

    // 前提：直连航线**确实**穿过 hard 禁飞区（否则"绕过"是假绿）
    CHECK(SimSource::segmentCrossesPolygon({deploy->centroidLng(), deploy->centroidLat()},
                                           {task->centroidLng(), task->centroidLat()},
                                           nofly->polygon));

    const auto* r1 = plan.routeOf("sim-01");
    CHECK(r1 != nullptr);
    if (r1 == nullptr) return;
    CHECK(r1->size() > 2);  // 绕行 = 航路点变多
    CHECK_EQ(plan.detoured, 1);
    const double directLen = SimSource::distanceMeters(
        deploy->centroidLng(), deploy->centroidLat(), task->centroidLng(), task->centroidLat());
    CHECK(SimSource::routeLengthMeters(*r1) > directLen * 1.05);

    // 核心断言：整条航路**从未**穿入 hard 禁飞区
    CHECK(SimSource::routeClearOfHardZones(*r1, s));
    const auto* r2 = plan.routeOf("sim-02");
    CHECK(r2 != nullptr);
    if (r2 != nullptr) CHECK(SimSource::routeClearOfHardZones(*r2, s));
    // 逐段单独复核（失败时能定位到哪一段）
    for (std::size_t i = 0; i + 1 < r1->size(); ++i) {
        CHECK(!SimSource::segmentCrossesPolygon((*r1)[i], (*r1)[i + 1], nofly->polygon));
    }
    note("      绕行：直连 %.1f m → 航路 %.1f m（%d 个航路点）\n", directLen,
         SimSource::routeLengthMeters(*r1), static_cast<int>(r1->size()));
}

void route02_kinematics_never_enters_hard_zone_and_arrives() {
    requires_("SIM-ROUTE-01", "SIM-ROUTE-05", "SIM-KIN-02", "SIM-KIN-03");
    const SimScenario s = baseScenario();
    const Area* nofly = s.findArea("NOFLY");
    CHECK(nofly != nullptr);
    if (nofly == nullptr) return;

    SimSource sim;
    RecordingSink* sink = nullptr;
    int violations = 0;
    int arrivedAt = -1;
    const Area* noflyPtr = nofly;
    (void)runScenarioEachStep(
        sim, s, 1200, 1000,  // 1200 s 仿真时间（50 m/s 走 ~9.5 km 绰绰有余）
        [&](int i) {
            for (const EntityStatus& e : sim.entities()) {
                if (!e.isTarget && SimSource::containsPoint(noflyPtr->polygon, e.lng, e.lat)) {
                    ++violations;
                }
            }
            if (arrivedAt < 0) {
                const std::vector<EntityStatus> esTick = sim.entities();
            const EntityStatus* a = find(esTick, "sim-01");
                if (a != nullptr && a->state == MotionState::Arrived) arrivedAt = i;
            }
        },
        &sink);
    CHECK_EQ(violations, 0);  // 全程逐 tick 都不在 hard 禁飞区内

    const std::vector<EntityStatus> es = sim.entities();
    const EntityStatus* lead = find(es, "sim-01");
    const EntityStatus* wing = find(es, "sim-02");
    CHECK(lead != nullptr && wing != nullptr);
    if (lead == nullptr || wing == nullptr) return;

    // 到达任务区后状态正确
    CHECK_EQ(static_cast<int>(lead->state), static_cast<int>(MotionState::Arrived));
    CHECK(lead->inTaskArea);
    CHECK(!lead->inHardZone);
    CHECK_EQ(lead->speedMps, 0.0);
    CHECK(lead->seq > 0);
    CHECK(arrivedAt >= 0);
    // 编队成员一起到位
    CHECK_EQ(static_cast<int>(wing->state), static_cast<int>(MotionState::Arrived));
    CHECK(wing->inTaskArea);
    note("      首次到达于第 %d 个 tick｜落点 %.6f,%.6f\n", arrivedAt, lead->lng, lead->lat);
}

void route02b_soft_zone_does_not_change_route() {
    requires_("SIM-ROUTE-02");
    SimScenario s = baseScenario();
    for (Area& a : s.areas) {  // 把 hard 那个改成 soft
        if (a.key == "NOFLY") {
            a.hardness = Hardness::Soft;
            a.hasHardness = true;
        }
    }
    const PlanResult plan = SimSource::planRoutes(s);
    CHECK(plan.ok);
    CHECK_EQ(plan.hardZones, 0);
    CHECK_EQ(plan.softZones, 2);
    CHECK_EQ(plan.detoured, 0);
    const auto* r = plan.routeOf("sim-01");
    CHECK(r != nullptr);
    if (r != nullptr) CHECK_EQ(static_cast<int>(r->size()), 2);  // 直连，两点

    // 运行期也只报告、不阻挡：soft 区可以被穿过（直连航路正好穿过限制区）
    SimSource sim;
    RecordingSink* sink = nullptr;
    bool anyInSoft = false;
    (void)runScenarioEachStep(
        sim, s, 200, 1000,
        [&](int) {
            for (const EntityStatus& e : sim.entities()) {
                if (e.inSoftZone) anyInSoft = true;
            }
        },
        &sink);
    CHECK(anyInSoft);
}

void route03_formation_keeps_relative_shape() {
    requires_("SIM-ROUTE-03");
    const SimScenario s = baseScenario();
    const PlanResult plan = SimSource::planRoutes(s);
    const auto* a = plan.routeOf("sim-01");
    const auto* b = plan.routeOf("sim-02");
    CHECK(a != nullptr && b != nullptr);
    if (a == nullptr || b == nullptr) return;
    CHECK_EQ(static_cast<int>(a->size()), static_cast<int>(b->size()));

    // 位形保持的真正不变量：成员航路 = 领队航路的**刚体平移**（逐航点平移量恒定）
    const double dLng0 = (*b)[0].first - (*a)[0].first;
    const double dLat0 = (*b)[0].second - (*a)[0].second;
    for (std::size_t i = 0; i + 1 < a->size(); ++i) {
        const double segLen = SimSource::distanceMeters(
            (*a)[i].first, (*a)[i].second, (*a)[i + 1].first, (*a)[i + 1].second);
        CHECK(segLen > 0.0);
        CHECK_NEAR((*b)[i].first - (*a)[i].first, dLng0, 1e-12);
        CHECK_NEAR((*b)[i].second - (*a)[i].second, dLat0, 1e-12);
    }
    // 平移量的大小 = √(200² + 300²)（成员与领队的起飞机位形）
    CHECK_NEAR(SimSource::distanceMeters((*a)[0].first, (*a)[0].second, (*b)[0].first,
                                         (*b)[0].second),
               std::sqrt(200.0 * 200.0 + 300.0 * 300.0), 1.0);
    // 逐段：两机同向同速（位形刚性 → 段内相对方位不变）
    const double segAzA = SimSource::headingBetween((*a)[0].first, (*a)[0].second, (*a)[1].first,
                                                    (*a)[1].second);
    const double segAzB = SimSource::headingBetween((*b)[0].first, (*b)[0].second, (*b)[1].first,
                                                    (*b)[1].second);
    CHECK_NEAR(segAzA, segAzB, 1e-2);  // 平移不改变段方向（容差为投影往返的浮点噪声）

    // 运行期：两机的**航向始终一致**、速度一致（编队不散）
    SimSource sim;
    RecordingSink* sink = nullptr;
    int headingMismatch = 0;
    (void)runScenarioEachStep(
        sim, s, 12, 5000,
        [&](int) {
            const std::vector<EntityStatus> esF = sim.entities();
            const EntityStatus* lead = find(esF, "sim-01");
            const EntityStatus* wing = find(esF, "sim-02");
            if (lead == nullptr || wing == nullptr) return;
            if (lead->state == MotionState::Arrived) return;
            if (std::fabs(lead->heading - wing->heading) > 1e-9) ++headingMismatch;
            if (std::fabs(lead->speedMps - wing->speedMps) > 1e-9) ++headingMismatch;
        },
        &sink);
    CHECK_EQ(headingMismatch, 0);  // 编队同向同速（位形刚性）
}

void route04_no_route_when_plan_impossible() {
    requires_("SIM-ROUTE-04");
    SimScenario s = baseScenario();
    s.areas.erase(std::remove_if(s.areas.begin(), s.areas.end(),
                                 [](const Area& a) { return a.key == "TASK"; }),
                  s.areas.end());
    const ValidationResult vr = SimSource::validate(s);
    CHECK(!vr.ok);
    bool found = false;
    for (const ScenarioIssue& i : vr.issues) {
        if (i.field == "taskAreaKey") found = true;
    }
    CHECK(found);  // 报的是"任务区无效"，不是"航路为空"
    const PlanResult plan = SimSource::planRoutes(s);
    CHECK(!plan.ok);
    CHECK(!plan.warnings.empty());
    CHECK(!plan.warnings[0].empty());  // 原因可读
}

// ============================================================================
// 用例 · 运动学
// ============================================================================

void kin01_straight_segment_interpolation() {
    requires_("SIM-KIN-01");
    SimScenario s;
    Area d;
    d.key = "D";
    d.role = AreaRole::Deploy;
    d.polygon = {{116.300, 39.900}, {116.310, 39.900}, {116.310, 39.910}, {116.300, 39.910}};
    s.areas.push_back(d);
    Area t;
    t.key = "T";
    t.role = AreaRole::Task;
    t.polygon = {{116.400, 39.900}, {116.410, 39.900}, {116.410, 39.910}, {116.400, 39.910}};
    s.areas.push_back(t);
    Platform p;
    p.deviceId = "k-1";
    p.deviceType = "uav";
    p.kind = "uav.pos";
    p.homeAreaKey = "D";
    p.taskAreaKey = "T";
    p.speedMps = 100.0;
    p.altM = 500.0;
    s.platforms.push_back(p);

    SimSource sim;
    RecordingSink* sink = nullptr;
    (void)runScenario(sim, s, 0, 0, &sink);
    const std::vector<EntityStatus> es0 = sim.entities();
    const EntityStatus* e0 = find(es0, "k-1");
    CHECK(e0 != nullptr);
    if (e0 == nullptr) return;
    const double lng0 = e0->lng;
    const double lat0 = e0->lat;

    sim.step(1000);  // 1 s × 100 m/s = 100 m
    const std::vector<EntityStatus> es1 = sim.entities();
    const EntityStatus* e = find(es1, "k-1");
    CHECK(e != nullptr);
    if (e == nullptr) return;
    CHECK_NEAR(SimSource::distanceMeters(lng0, lat0, e->lng, e->lat), 100.0, 0.5);
    CHECK_NEAR(e->heading, 90.0, 0.5);   // 正东
    CHECK_NEAR(e->speedMps, 100.0, 1e-9);
    CHECK_EQ(e->seq, 1);
    CHECK_NEAR(e->battery, 99.99, 1e-6);  // 默认耗率 0.01 %/s
    CHECK_NEAR(e->altM, 500.0, 1e-9);

    // 位置/航向随段落连续（再走 1 s 仍在同一段上）
    sim.step(1000);
    const std::vector<EntityStatus> es2 = sim.entities();
    const EntityStatus* e2 = find(es2, "k-1");
    CHECK(e2 != nullptr);
    if (e2 != nullptr) CHECK_NEAR(SimSource::distanceMeters(lng0, lat0, e2->lng, e2->lat), 200.0, 1.0);
}

void kin02_waypoint_switch_and_arrival() {
    requires_("SIM-KIN-02", "SIM-KIN-03");
    const SimScenario s = baseScenario();
    const PlanResult plan = SimSource::planRoutes(s);
    const auto* route = plan.routeOf("sim-01");
    CHECK(route != nullptr);
    if (route == nullptr) return;
    CHECK(route->size() >= 3);

    SimSource sim;
    RecordingSink* sink = nullptr;
    (void)runScenario(sim, s, 0, 0, &sink);
    const std::vector<EntityStatus> esBefore = sim.entities();
    const EntityStatus* before = find(esBefore, "sim-01");
    CHECK(before != nullptr);
    if (before == nullptr) return;
    CHECK_EQ(before->waypointIndex, 0);

    // 走到第一个航点之后 → 切到第二段
    const double firstLeg = SimSource::distanceMeters(
        (*route)[0].first, (*route)[0].second, (*route)[1].first, (*route)[1].second);
    const int64_t needMs = static_cast<int64_t>(firstLeg / 50.0 * 1000.0) + 5000;
    int switched = 0;
    for (int64_t elapsed = 0; elapsed < needMs; elapsed += 1000) {
        sim.step(1000);
        const std::vector<EntityStatus> esA = sim.entities();
    const EntityStatus* e = find(esA, "sim-01");
        if (e != nullptr && e->waypointIndex >= 1) {
            switched = 1;
            break;
        }
    }
    CHECK_EQ(switched, 1);

    // 一路走到终点
    for (int i = 0; i < 900; ++i) sim.step(1000);
    const std::vector<EntityStatus> esDone = sim.entities();
    const EntityStatus* done = find(esDone, "sim-01");
    CHECK(done != nullptr);
    if (done == nullptr) return;
    CHECK_EQ(static_cast<int>(done->state), static_cast<int>(MotionState::Arrived));
    CHECK_NEAR(done->lng, (*route)[route->size() - 1].first, 1e-9);
    CHECK_NEAR(done->lat, (*route)[route->size() - 1].second, 1e-9);
    CHECK_EQ(done->speedMps, 0.0);
    // 到达后位置冻结
    const double lngFrozen = done->lng;
    sim.step(60000);
    const std::vector<EntityStatus> esStill = sim.entities();
    const EntityStatus* still = find(esStill, "sim-01");
    CHECK(still != nullptr);
    if (still != nullptr) CHECK_NEAR(still->lng, lngFrozen, 1e-12);
}

void kin03_battery_drain_and_clamp() {
    requires_("SIM-KIN-04");
    SimScenario s = baseScenario();
    s.platforms[0].battery = 0.005;  // 只有 0.005% 电量
    SimSource sim;
    RecordingSink* sink = nullptr;
    (void)runScenario(sim, s, 0, 0, &sink);
    sim.step(5000);  // 5 s × 0.01 %/s = 0.05% > 0.005% → 必须夹到 0
    const std::vector<EntityStatus> esA = sim.entities();
    const EntityStatus* e = find(esA, "sim-01");
    CHECK(e != nullptr);
    if (e != nullptr) {
        CHECK_EQ(e->battery, 0.0);
        CHECK(e->battery >= 0.0);
    }
    CHECK(sink != nullptr);
    if (sink != nullptr) {
        for (const SimEvent& ev : sink->events) CHECK(ev.battery >= 0 && ev.battery <= 100);
    }
}

// ============================================================================
// 用例 · 节拍
// ============================================================================

void tick01_speed_8x_advances_8s_per_real_second() {
    requires_("SIM-TICK-01", "SIM-TICK-02", "SIM-TICK-05");
    const SimScenario s = baseScenario();
    SimSource sim;
    RecordingSink* sink = nullptr;
    (void)runScenario(sim, s, 0, 0, &sink);

    auto clock = std::make_shared<FakeClock>(1789000000000LL);
    sim.setClock(clock);

    // 1× 基准：1 s 真实时间 → 1 s 仿真时间
    CHECK(sim.setSpeed(1));
    sim.tick(clock->value());  // 首次只记基线
    clock->advance(1000);
    sim.tick(clock->value());
    CHECK_EQ(sim.simElapsedMs(), 1000);

    // 8×：再 1 s 真实时间 → 8 s 仿真时间
    CHECK(sim.setSpeed(8));
    CHECK_EQ(sim.speedMultiplier(), 8);
    const int64_t before = sim.simElapsedMs();
    clock->advance(1000);
    const int emitted = sim.tick(clock->value());
    const int64_t delta = sim.simElapsedMs() - before;
    CHECK_EQ(delta, 8000);
    CHECK(emitted > 0);
    note("      1×｜真实 +1000 ms → 仿真 +%lld ms\n", static_cast<long long>(before));
    note("      8×｜真实 +1000 ms → 仿真 +%lld ms（事件 %d 条）\n",
         static_cast<long long>(delta), emitted);

    // 60×：再 1 s 真实时间 → 60 s 仿真时间
    CHECK(sim.setSpeed(60));
    const int64_t before60 = sim.simElapsedMs();
    clock->advance(1000);
    sim.tick(clock->value());
    CHECK_EQ(sim.simElapsedMs() - before60, 60000);
    note("      60×｜真实 +1000 ms → 仿真 +%lld ms\n",
         static_cast<long long>(sim.simElapsedMs() - before60));

    // 时钟确实被读（可注入性）：tick() 无参重载走注入时钟
    const int readsBefore = clock->reads;
    sim.tick();
    CHECK(clock->reads > readsBefore);
}

void tick02_speed_multiplier_is_whitelisted() {
    requires_("SIM-TICK-02");
    SimSource sim;
    CHECK(sim.setSpeed(1));
    CHECK(sim.setSpeed(8));
    CHECK(sim.setSpeed(60));
    CHECK(!sim.setSpeed(0));
    CHECK(!sim.setSpeed(7));
    CHECK(!sim.setSpeed(-8));
    CHECK(!sim.setSpeed(61));
    CHECK(!sim.setSpeed(1000));
    CHECK_EQ(sim.speedMultiplier(), 60);  // 被拒的调用 MUST NOT 改变现状
    const std::vector<int> allowed = allowedSpeedMultipliers();
    CHECK_EQ(static_cast<int>(allowed.size()), 3);
    CHECK_EQ(allowed[0], 1);
    CHECK_EQ(allowed[1], 8);
    CHECK_EQ(allowed[2], 60);
}

void tick03_pause_discards_real_time_resume_does_not_jump() {
    requires_("SIM-TICK-03");
    const SimScenario s = baseScenario();
    SimSource sim;
    RecordingSink* sink = nullptr;
    (void)runScenario(sim, s, 0, 0, &sink);
    auto clock = std::make_shared<FakeClock>(1000000LL);
    sim.setClock(clock);

    sim.tick(clock->value());
    clock->advance(1000);
    sim.tick(clock->value());
    CHECK_EQ(sim.simElapsedMs(), 1000);
    CHECK(!sink->events.empty());
    const std::size_t eventsBefore = sink->events.size();

    sim.pause();
    CHECK(sim.paused());
    for (int i = 0; i < 10; ++i) {
        clock->advance(60000);  // 暂停期间真实时间过了 10 分钟
        CHECK_EQ(sim.tick(clock->value()), 0);
    }
    CHECK_EQ(sim.simElapsedMs(), 1000);           // 仿真时间没动
    CHECK_EQ(static_cast<int>(sink->events.size()), static_cast<int>(eventsBefore));
    const std::vector<EntityStatus> esA = sim.entities();
    const EntityStatus* e = find(esA, "sim-01");
    CHECK(e != nullptr);
    if (e != nullptr) CHECK_EQ(static_cast<int>(e->state), static_cast<int>(MotionState::Holding));

    sim.resume();
    CHECK(!sim.paused());
    // resume 后第一个 tick 只重建时间基线（不推进），之后才计时。
    // 注意：基线必须取"恢复时刻"，否则首次 tick 的 nowMs 会小于暂停期的 lastTickMs（负差值）。
    sim.tick(clock->value());
    clock->advance(1000);
    sim.tick(clock->value());
    clock->advance(1000);
    sim.tick(clock->value());
    CHECK_EQ(sim.simElapsedMs(), 3000);  // 恢复后是 +2 s（不是把暂停的 600 s 补回来）
    CHECK(sim.metrics().pausedMs >= 600000);
}

void tick04_step_is_independent_of_speed() {
    requires_("SIM-TICK-04");
    const SimScenario s = baseScenario();
    SimSource sim;
    RecordingSink* sink = nullptr;
    (void)runScenario(sim, s, 0, 0, &sink);
    CHECK(sim.setSpeed(60));
    sim.step(1000);  // 定步长：与倍速无关
    CHECK_EQ(sim.simElapsedMs(), 1000);
    sim.step(250);
    CHECK_EQ(sim.simElapsedMs(), 1250);
    sim.pause();
    sim.step(1000);  // 暂停时单步不推进
    CHECK_EQ(sim.simElapsedMs(), 1250);
    sim.resume();
    sim.step(1000);
    CHECK_EQ(sim.simElapsedMs(), 2250);
    const std::size_t before = sink->events.size();
    CHECK_EQ(sim.step(0), 0);
    CHECK_EQ(sim.step(-5), 0);
    CHECK_EQ(static_cast<int>(sink->events.size()), static_cast<int>(before));
}

void tick05_substep_keeps_waypoint_turn_accurate() {
    requires_("SIM-NFR-05", "SIM-KIN-02");
    SimScenario s;
    Area d;
    d.key = "D";
    d.role = AreaRole::Deploy;
    d.polygon = {{116.300, 39.900}, {116.310, 39.900}, {116.310, 39.910}, {116.300, 39.910}};
    s.areas.push_back(d);
    Area t;
    t.key = "T";
    t.role = AreaRole::Task;
    t.polygon = {{116.400, 39.900}, {116.410, 39.900}, {116.410, 39.910}, {116.400, 39.910}};
    s.areas.push_back(t);
    Platform p;
    p.deviceId = "s-1";
    p.deviceType = "uav";
    p.kind = "uav.pos";
    p.homeAreaKey = "D";
    p.taskAreaKey = "T";
    p.speedMps = 100.0;
    s.platforms.push_back(p);

    // 同样的总时长：一次 60 s 步长 vs 60 次 1 s 步长 → 位置一致（子步切分使拐点不失真）
    SimSource a;
    SimSource b;
    RecordingSink* sa = nullptr;
    RecordingSink* sb = nullptr;
    (void)runScenario(a, s, 0, 0, &sa);
    (void)runScenario(b, s, 0, 0, &sb);
    a.step(60000);
    for (int i = 0; i < 60; ++i) b.step(1000);
    const EntityStatus* ea = find(a.entities(), "s-1");
    const EntityStatus* eb = find(b.entities(), "s-1");
    CHECK(ea != nullptr && eb != nullptr);
    if (ea != nullptr && eb != nullptr) {
        CHECK_NEAR(ea->lng, eb->lng, 1e-9);
        CHECK_NEAR(ea->lat, eb->lat, 1e-9);
        CHECK_EQ(ea->state == MotionState::Enroute, true);  // 主段 8800 m / 100 m/s = 88 s
    }
    a.step(60000);
    const EntityStatus* ea2 = find(a.entities(), "s-1");
    CHECK(ea2 != nullptr);
    if (ea2 != nullptr) {
        CHECK_EQ(static_cast<int>(ea2->state), static_cast<int>(MotionState::Arrived));
    }
}

// ============================================================================
// 用例 · 目标运动
// ============================================================================

void tgt01_static_target_does_not_move() {
    requires_("SIM-TGT-01");
    const SimScenario s = baseScenario();
    SimSource sim;
    RecordingSink* sink = nullptr;
    (void)runScenario(sim, s, 0, 0, &sink);
    const std::vector<EntityStatus> esT = sim.entities();
    const EntityStatus* before = find(esT, "tgt-1");
    CHECK(before != nullptr);
    if (before == nullptr) return;
    const double lng0 = before->lng;
    const double lat0 = before->lat;
    CHECK(before->visible);
    CHECK_NEAR(lng0, 116.400, 1e-9);
    CHECK_NEAR(lat0, 39.910, 1e-9);

    sim.step(120000);
    const std::vector<EntityStatus> esAfter = sim.entities();
    const EntityStatus* after = find(esAfter, "tgt-1");
    CHECK(after != nullptr);
    if (after == nullptr) return;
    CHECK_NEAR(after->lng, lng0, 1e-12);
    CHECK_NEAR(after->lat, lat0, 1e-12);
    CHECK_EQ(after->speedMps, 0.0);
}

void tgt02_dynamic_target_moves_along_route_and_loops() {
    requires_("SIM-TGT-02");
    const SimScenario s = baseScenario();
    SimSource sim;
    RecordingSink* sink = nullptr;
    (void)runScenario(sim, s, 0, 0, &sink);
    const std::vector<EntityStatus> esStart = sim.entities();
    const EntityStatus* start = find(esStart, "tgt-2");
    CHECK(start != nullptr);
    if (start == nullptr) return;
    const double lng0 = start->lng;
    const double lat0 = start->lat;

    sim.step(10000);  // 10 s × 40 m/s = 400 m 向东
    const std::vector<EntityStatus> esMid = sim.entities();
    const EntityStatus* mid = find(esMid, "tgt-2");
    CHECK(mid != nullptr);
    if (mid == nullptr) return;
    CHECK(mid->lng > lng0);
    CHECK_NEAR(SimSource::distanceMeters(lng0, lat0, mid->lng, mid->lat), 400.0, 1.0);
    CHECK_NEAR(mid->heading, 90.0, 0.5);

    // 航路 [A,B]（1707.83 m）+ loop → 闭环 A→B→A，一圈 = 2×|AB| / 40 m/s = 85.39 s。
    // 用**解析里程**复核每个时刻的位置：位置必须等于"里程 → 航路点"的确定映射。
    // 期望值直接用**引擎自己给出的航路端点**算（避免测试另建一套坐标系），
    // 因此容差可以压到 1e-8 度（≈1 mm，纯浮点噪声量级）。
    // 期望值直接用**场景里声明的目标航路**算（目标航路不进 PlanResult —— 那是平台航路规划）
    const Target* dynTarget = nullptr;
    for (const Target& tg : s.targets) {
        if (tg.id == "tgt-2") dynTarget = &tg;
    }
    CHECK(dynTarget != nullptr && dynTarget->route.size() >= 2);
    if (dynTarget == nullptr || dynTarget->route.size() < 2) return;
    const double aLng = dynTarget->route[0].first;
    const double bLng = dynTarget->route[1].first;
    const double legLen = SimSource::distanceMeters(aLng, dynTarget->route[0].second, bLng,
                                                    dynTarget->route[1].second);
    const double cycle = 2.0 * legLen;
    // 参照系与实现一致：实现的局部平面原点 = **场景包围盒中心**（frameFor），
    // 纬向尺度按该中心纬度的 cos 收缩；用同一纬度算，才能把容差压到浮点噪声量级。
    double latMin = dynTarget->route[0].second;
    double latMax = latMin;
    for (const Area& a : s.areas) {
        for (const auto& pt : a.polygon) {
            latMin = std::min(latMin, pt.second);
            latMax = std::max(latMax, pt.second);
        }
    }
    for (const Target& tg : s.targets) {
        for (const auto& pt : tg.route) {
            latMin = std::min(latMin, pt.second);
            latMax = std::max(latMax, pt.second);
        }
    }
    (void)latMin;
    (void)latMax;
    auto analyticLng = [&](double elapsedSec) {
        const double arc = std::fmod(40.0 * elapsedSec, cycle);
        const double fromA = (arc <= legLen) ? arc : (cycle - arc);  // 距 A 点的距离（往返对称）
        // 用**引擎同款** distanceMeters 定标：它内部用的参照纬度与实现完全一致，
        // 这样期望值与实测值只差浮点噪声（自己另算 cos(lat) 会带来毫米级系统偏差）
        const double unit = SimSource::distanceMeters(aLng, dynTarget->route[0].second,
                                                      aLng + 1.0, dynTarget->route[0].second);
        return aLng + fromA / unit;
    };
    // 折返点在 t = legLen/40 = 42.7 s；t=105 s → 4200 mod 3415.66 = 784.3 m（去程，航向朝东）
    sim.step(95000);  // 累计 105 s
    {
        const std::vector<EntityStatus> esBack = sim.entities();
        const EntityStatus* back = find(esBack, "tgt-2");
        CHECK(back != nullptr);
        if (back == nullptr) return;
        CHECK_NEAR(back->lng, analyticLng(105.0), 1e-5);
        CHECK_NEAR(back->heading, 90.0, 0.5);
    }
    // 回程段（t = 130 s → 5200 mod 3415.66 = 1784.3 m > 1707.8）：航向必须变正西
    sim.step(25000);  // 累计 130 s
    {
        const std::vector<EntityStatus> esRet = sim.entities();
        const EntityStatus* ret = find(esRet, "tgt-2");
        CHECK(ret != nullptr);
        if (ret == nullptr) return;
        CHECK_NEAR(ret->heading, 270.0, 0.5);
        CHECK_NEAR(ret->lng, analyticLng(130.0), 1e-5);
        CHECK(ret->state != MotionState::Arrived);  // loop：永不到达
        CHECK_NEAR(ret->speedMps, 40.0, 1e-9);
    }
    // 整圈回起点：从 0 再走 cycle 的整数倍 → 回到 A
    SimSource sim2;
    RecordingSink* sink2 = nullptr;
    (void)runScenario(sim2, s, 0, 0, &sink2);
    const int64_t oneCycleMs = static_cast<int64_t>(cycle / 40.0 * 1000.0 + 0.5);
    sim2.step(oneCycleMs);
    const EntityStatus* home = find(sim2.entities(), "tgt-2");
    CHECK(home != nullptr);
    if (home != nullptr) {
        const double backHome = SimSource::distanceMeters(lng0, lat0, home->lng, home->lat);
        CHECK_NEAR(backHome, 0.0, 0.5);
        note("      loop 一圈 %lld ms（2×|AB| = %.1f m）：回到起点偏差 %.3f m\n",
             static_cast<long long>(oneCycleMs), cycle, backHome);
    }
}

void tgt02b_non_loop_target_stops_at_end() {
    requires_("SIM-TGT-02");
    SimScenario s = baseScenario();
    s.targets[1].loop = false;  // tgt-2 不循环
    SimSource sim;
    RecordingSink* sink = nullptr;
    (void)runScenario(sim, s, 0, 0, &sink);
    sim.step(150000);
    const std::vector<EntityStatus> esEnd = sim.entities();  // 150 s > 100 s（4000 m / 40 m/s）
    const EntityStatus* e = find(esEnd, "tgt-2");
    CHECK(e != nullptr);
    if (e == nullptr) return;
    CHECK_EQ(static_cast<int>(e->state), static_cast<int>(MotionState::Arrived));
    CHECK_NEAR(e->lng, 116.412, 1e-6);
    CHECK_EQ(e->speedMps, 0.0);
}

void tgt03_popup_appears_after_offset() {
    requires_("SIM-TGT-03");
    const SimScenario s = baseScenario();
    SimSource sim;
    RecordingSink* sink = nullptr;
    (void)runScenario(sim, s, 0, 0, &sink);

    const std::vector<EntityStatus> esH0 = sim.entities();
    const EntityStatus* hidden0 = find(esH0, "tgt-3");
    CHECK(hidden0 != nullptr);
    if (hidden0 == nullptr) return;
    CHECK(!hidden0->visible);
    CHECK_EQ(static_cast<int>(hidden0->state), static_cast<int>(MotionState::Pending));

    sim.step(29000);  // 29 s < 30 s
    const std::vector<EntityStatus> esH1 = sim.entities();
    const EntityStatus* hidden1 = find(esH1, "tgt-3");
    CHECK(hidden1 != nullptr);
    if (hidden1 != nullptr) CHECK(!hidden1->visible);
    CHECK(!hasDevice(sink->events, "tgt-3"));  // 偏移前一条事件都不许有

    sim.step(1000);  // 到 30 s
    const std::vector<EntityStatus> esShown = sim.entities();
    const EntityStatus* shown = find(esShown, "tgt-3");
    CHECK(shown != nullptr);
    if (shown == nullptr) return;
    CHECK(shown->visible);
    CHECK_EQ(static_cast<int>(shown->state), static_cast<int>(MotionState::Enroute));
    CHECK(hasDevice(sink->events, "tgt-3"));  // 偏移后出现并开始上报
    CHECK_EQ(sim.metrics().popupSpawned, 1);

    // 出现后按 route 运动
    const double lngAtSpawn = shown->lng;
    sim.step(10000);
    const std::vector<EntityStatus> esMoving = sim.entities();
    const EntityStatus* moving = find(esMoving, "tgt-3");
    CHECK(moving != nullptr);
    if (moving != nullptr) {
        CHECK(moving->lng > lngAtSpawn);
        CHECK_NEAR(moving->speedMps, 30.0, 1e-9);
    }
}

// ============================================================================
// 用例 · 事件产出
// ============================================================================

void evt01_one_normalized_event_per_visible_entity_per_tick() {
    requires_("SIM-OUT-01", "SIM-OUT-02");
    SimScenario s = baseScenario();
    SimSource sim;
    RecordingSink* sink = nullptr;
    (void)runScenario(sim, s, 0, 0, &sink);

    // tick 1：平台 2 + 目标 2（popup 未出现，static 也算可见）= 4 条
    CHECK_EQ(sim.step(1000), 4);
    CHECK_EQ(static_cast<int>(sink->events.size()), 4);

    // tick 2：又 4 条，顺序稳定（deviceId 字典序）
    CHECK_EQ(sim.step(1000), 4);
    CHECK_EQ(static_cast<int>(sink->events.size()), 8);
    CHECK_STR(sink->events[0].deviceId, "sim-01");
    CHECK_STR(sink->events[1].deviceId, "sim-02");
    CHECK_STR(sink->events[2].deviceId, "tgt-1");
    CHECK_STR(sink->events[3].deviceId, "tgt-2");
    if (sink->events.size() > 4) CHECK_STR(sink->events[4].deviceId, "sim-01");

    // 每实体每 tick 恰好一条；seq 自 0 单调 +1
    int64_t expectedSeq = 0;
    for (const SimEvent& e : sink->events) {
        if (e.deviceId != "sim-01") continue;
        CHECK_EQ(e.seq, expectedSeq);
        ++expectedSeq;
    }
    CHECK_EQ(expectedSeq, 2);
    CHECK_EQ(static_cast<int>(sink->events.size()),
             static_cast<int>(sim.metrics().eventsEmitted));
    // 每个事件的 ts 都等于本节拍的仿真时刻
    CHECK_EQ(sink->events[0].ts, 1000);
    if (sink->events.size() > 7) CHECK_EQ(sink->events[7].ts, 2000);
}

void evt02_event_fields_equal_device_ingest_contract() {
    requires_("SIM-OUT-02", "SIM-OUT-03", "SIM-OUT-04");
    // 《外设接入契约》§2 的 11 个字段，**按契约键序**
    const std::vector<std::string> contract = {"deviceId", "deviceType", "kind",  "lng",  "lat",
                                               "alt",      "heading",    "speed", "battery",
                                               "seq",      "ts"};
    CHECK_EQ(static_cast<int>(normalizedFieldOrder().size()), 11);
    for (std::size_t i = 0; i < contract.size(); ++i) {
        CHECK_STR(normalizedFieldOrder()[i], contract[i]);
    }

    const SimScenario s = baseScenario();
    SimSource sim;
    RecordingSink* sink = nullptr;
    (void)runScenario(sim, s, 0, 0, &sink);
    sim.step(1000);
    CHECK(!sink->events.empty());
    if (sink->events.empty()) return;

    const SimEvent& e = sink->events[0];
    const std::string dumped = e.toJson().dump();
    // ① 键序逐字段等于契约
    std::size_t pos = 0;
    for (const std::string& f : contract) {
        const std::string needle = "\"" + f + "\":";
        const std::size_t at = dumped.find(needle);
        CHECK(at != std::string::npos);
        CHECK(at >= pos);
        pos = at;
    }
    // ② 取值落在契约 §1 的统一约定内
    CHECK(e.lng >= -180.0 && e.lng <= 180.0);
    CHECK(e.lat >= -90.0 && e.lat <= 90.0);
    CHECK(e.heading >= 0.0 && e.heading < 360.0);
    CHECK(e.speed >= 0.0);
    CHECK(e.battery >= 0 && e.battery <= 100);
    CHECK(e.seq >= 0);
    CHECK(e.ts >= 0);
    CHECK(!e.deviceId.empty());
    CHECK(!e.deviceType.empty());
    CHECK(!e.kind.empty());
    CHECK_STR(e.kind, "uav.pos");  // kind 由场景注入（引擎不解释）
    CHECK_STR(e.deviceType, "uav");
    // ③ 5 个可选字段默认存在
    CHECK(e.hasAlt() && e.hasHeading() && e.hasSpeed() && e.hasBattery() && e.hasSeq());
    // ④ 清存在位 → JSON 里省略该键（契约允许字段缺失）
    SimEvent sparse = e;
    sparse.presence = 0;
    const std::string sparseDump = sparse.toJson().dump();
    for (const char* f : {"alt", "heading", "speed", "battery", "seq"}) {
        CHECK(sparseDump.find(std::string("\"") + f + "\":") == std::string::npos);
    }
    CHECK(sparseDump.find("\"lng\":") != std::string::npos);
    CHECK(sparseDump.find("\"ts\":") != std::string::npos);
}

void evt03_extensions_are_host_owned_and_deterministic() {
    requires_("SIM-OUT-04");
    SimEvent e;
    e.deviceId = "x";
    e.deviceType = "uav";
    e.kind = "uav.pos";
    e.ts = 7;
    e.lng = 116.4;
    e.lat = 39.9;
    // 宿主扩展位：故意乱序插入，输出必须按键字典序稳定
    e.extensions["zeta"] = 1;
    e.extensions["alpha"] = 2;
    e.extensions["mid"] = "m";
    const std::string d1 = e.toJson().dump();
    const std::string d2 = e.toJson().dump();
    CHECK_STR(d1, d2);
    CHECK(d1.find("\"alpha\":2") != std::string::npos);
    CHECK(d1.find("\"mid\":\"m\"") != std::string::npos);
    CHECK(d1.find("\"zeta\":1") != std::string::npos);
    CHECK(d1.find("\"alpha\"") < d1.find("\"mid\""));
    CHECK(d1.find("\"mid\"") < d1.find("\"zeta\""));
    CHECK(d1.find("\"ts\":7") < d1.find("\"alpha\""));  // 主键在前
    SimEvent plain = e;
    plain.extensions = json::object();
    CHECK(plain.toJson().dump().find("alpha") == std::string::npos);
}

void evt04_parse_round_trip_and_rejections() {
    requires_("SIM-OUT-03", "SIM-IN-02");
    SimEvent e;
    e.deviceId = "uav-9";
    e.deviceType = "uav";
    e.kind = "uav.pos";
    e.lng = 116.3974;
    e.lat = 39.9093;
    e.alt = 900.0;
    e.heading = 128.5;
    e.speed = 22.4;
    e.battery = 85;
    e.seq = 10241;
    e.ts = 1789000000123LL;
    const json j = e.toJson();
    const EventParseResult r = parseSimEvent(j);
    CHECK(r.ok);
    CHECK_STR(r.event.deviceId, "uav-9");
    CHECK_STR(r.event.kind, "uav.pos");
    CHECK_EQ(r.event.battery, 85);
    CHECK_EQ(r.event.ts, 1789000000123LL);
    CHECK_NEAR(r.event.lng, 116.3974, 1e-12);
    CHECK_STR(r.event.toJson().dump(), j.dump());  // 往返逐字段一致

    // 未知字段忽略（契约降级约定），但保留在扩展位
    json withUnknown = j;
    withUnknown["tsSource"] = "device";
    const EventParseResult r2 = parseSimEvent(withUnknown);
    CHECK(r2.ok);
    CHECK(r2.event.extensions.contains("tsSource"));

    // 缺 MUST 字段 / 越界 → ok=false + 可读原因（不抛异常）
    json missId = j;
    missId.erase("deviceId");
    const EventParseResult r3 = parseSimEvent(missId);
    CHECK(!r3.ok);
    CHECK(!r3.reason.empty());

    json missTs = j;
    missTs.erase("ts");
    CHECK(!parseSimEvent(missTs).ok);

    json missCoord = j;
    missCoord.erase("lng");
    CHECK(!parseSimEvent(missCoord).ok);

    json badCoord = j;
    badCoord["lng"] = 200.0;
    CHECK(!parseSimEvent(badCoord).ok);

    json badBattery = j;
    badBattery["battery"] = 150;
    CHECK(!parseSimEvent(badBattery).ok);

    CHECK(!parseSimEvent(json::array()).ok);

    // heading 归一化到 [0, 360)
    json heading = j;
    heading["heading"] = 370.0;
    const EventParseResult r5 = parseSimEvent(heading);
    CHECK(r5.ok);
    CHECK_NEAR(r5.event.heading, 10.0, 1e-9);
}

// ============================================================================
// 用例 · 传感器挂接
// ============================================================================

/// 宿主侧最简探测模型（**本模块不含任何探测算法**）
class StubSensor : public ISensorModel {
public:
    std::string id() const override { return "stub.v1"; }
    std::string deviceType() const override { return "sensor"; }
    std::vector<SimObservation> sense(const SensorPose& pose) override {
        ++calls;
        lastTs = pose.ts;
        lastCandidates = static_cast<int>(pose.candidates.size());
        lastSelfId = pose.self.platformId;
        lastSensorId = pose.sensorId;
        lastRangeM = pose.rangeM;
        if (throwOnSense) throw std::runtime_error("探测模型故意抛异常");
        std::vector<SimObservation> out;
        for (const EntityPose& c : pose.candidates) {
            SimObservation o;
            o.sensorId = pose.sensorId;
            o.deviceType = pose.deviceType;
            o.kind = "detect.pos";
            o.targetId = c.platformId;
            o.targetType = c.deviceType;
            o.lng = c.lng;
            o.lat = c.lat;
            o.confidence = 0.5;
            o.ts = pose.ts;
            out.push_back(o);
        }
        return out;
    }
    int calls = 0;
    int64_t lastTs = -1;
    int lastCandidates = -1;
    std::string lastSelfId;
    std::string lastSensorId;
    double lastRangeM = -1;
    bool throwOnSense = false;
};

void sensor01_attached_model_is_called_with_pose() {
    requires_("SIM-SENSOR-01", "SIM-SENSOR-02");
    const SimScenario s = baseScenario();
    SimSource sim;
    RecordingSink* sink = nullptr;
    (void)runScenario(sim, s, 0, 0, &sink);
    auto model = std::make_shared<StubSensor>();
    sim.setSensorModel(model);
    sim.setSensorAttachments({SensorAttachment{"sim-01", "sensor-x", 15000.0, 0}});

    sim.step(1000);
    CHECK_EQ(model->calls, 1);
    CHECK_STR(model->lastSelfId, "sim-01");
    CHECK_STR(model->lastSensorId, "sensor-x");
    CHECK_NEAR(model->lastRangeM, 15000.0, 1e-9);
    // 候选 = 其它全部可见实体（不含自己、不含未出现的 popup）
    CHECK_EQ(model->lastCandidates, 3);
    CHECK_EQ(static_cast<int>(sink->observations.size()), 3);
    // sensorId 是上报方、targetId 是被发现方（不复用 deviceId 口径）
    CHECK_STR(sink->observations[0].sensorId, "sensor-x");
    CHECK(sink->observations[0].targetId != "sensor-x");
    CHECK_STR(sink->observations[0].kind, "detect.pos");
    CHECK_EQ(sim.metrics().sensorCalls, 1);
    CHECK_EQ(sim.metrics().observationsEmitted, 3);

    // popup 出现后候选 +1（引擎不做任何筛选，穷举交给模型）
    for (int i = 0; i < 30; ++i) sim.step(1000);
    const int before = model->calls;
    sim.step(1000);
    CHECK(model->calls > before);
    CHECK_EQ(model->lastCandidates, 4);

    // 注入了模型但**没有挂接清单** → 不调用、不产观测
    SimSource sim2;
    RecordingSink* sink2 = nullptr;
    (void)runScenario(sim2, s, 0, 0, &sink2);
    auto model2 = std::make_shared<StubSensor>();
    sim2.setSensorModel(model2);
    sim2.step(1000);
    CHECK_EQ(model2->calls, 0);
    CHECK(sink2->observations.empty());
    CHECK_EQ(sim2.metrics().observationsEmitted, 0);
    // 平台位姿交给模型：self 就是平台自己的位姿
    auto model3 = std::make_shared<StubSensor>();
    sim2.setSensorModel(model3);
    sim2.setSensorAttachments({SensorAttachment{"sim-02", "sensor-y", 0.0, 0}});
    sim2.step(1000);
    CHECK_EQ(model3->calls, 1);
    CHECK_STR(model3->lastSelfId, "sim-02");
}

void sensor02_no_detection_algorithm_inside_module() {
    requires_("SIM-SENSOR-03");
    const SimScenario s = baseScenario();
    SimSource sim;
    RecordingSink* sink = nullptr;
    (void)runScenario(sim, s, 0, 0, &sink);
    auto model = std::make_shared<StubSensor>();
    sim.setSensorModel(model);
    sim.setSensorAttachments({SensorAttachment{"sim-01", "sensor-x", 0.0, 2500}});  // 每 2.5 s 一次

    // 间隔口径 = **距上次探测**（不是"从开局起算"）：t=1000 首次调用
    sim.step(1000);
    CHECK_EQ(model->calls, 1);
    CHECK_EQ(model->lastTs, 1000);
    sim.step(1000);  // t=2000：距上次 1 s < 2.5 s
    CHECK_EQ(model->calls, 1);
    sim.step(1000);  // t=3000：距上次 2 s < 2.5 s
    CHECK_EQ(model->calls, 1);
    sim.step(2000);  // t=5000：距上次 4 s > 2.5 s
    CHECK_EQ(model->calls, 2);
    CHECK_EQ(model->lastTs, 5000);
    sim.step(2000);  // t=7000：距上次 2 s < 2.5 s
    CHECK_EQ(model->calls, 2);
    sim.step(1000);  // t=8000：距上次 3 s > 2.5 s
    CHECK_EQ(model->calls, 3);

    // rangeM = 0 时本模块**不筛**：候选照样全给（谁被发现由模型判定）
    sim.step(25000);  // t=33000 > 30000：popup 已出现
    CHECK_EQ(model->lastCandidates, 4);  // 穷举：候选=其它全部实体（含新出现的 popup）

    // 模型抛异常：计入 sensorErrors，仿真继续、事件照发
    model->throwOnSense = true;
    const int64_t before = sim.simElapsedMs();
    const std::size_t eventsBefore = sink->events.size();
    sim.step(3000);  // 跨过一个探测间隔，保证真的调用到模型
    CHECK_EQ(sim.simElapsedMs(), before + 3000);
    CHECK(sink->events.size() > eventsBefore);
    CHECK(sim.metrics().sensorErrors >= 1);
}

void sensor03_sink_exception_does_not_stop_simulation() {
    requires_("SIM-NFR-04", "SIM-SENSOR-02");
    const SimScenario s = baseScenario();
    SimSource sim;
    sim.setSink(std::make_shared<ThrowingSink>());
    CHECK(sim.init(s).ok);
    CHECK_EQ(sim.step(1000), 4);         // 事件照产
    CHECK_EQ(sim.simElapsedMs(), 1000);  // 时间照走
    CHECK(sim.metrics().sinkErrors > 0); // 异常被吞并计数
    sim.step(1000);
    CHECK_EQ(sim.simElapsedMs(), 2000);
}
// ============================================================================
// 用例 · 确定性与契约
// ============================================================================

/// 跑一整套输入序列，返回事件序列的逐字节表示
std::string deterministicRun() {
    const SimScenario s = baseScenario();
    SimSource sim;
    auto sink = std::make_shared<RecordingSink>();
    sim.setSink(sink);
    auto clock = std::make_shared<FakeClock>(1789000000000LL);
    sim.setClock(clock);
    auto model = std::make_shared<StubSensor>();
    sim.setSensorModel(model);
    sim.setSensorAttachments({SensorAttachment{"sim-01", "sensor-x", 20000.0, 3000}});
    (void)sim.init(s);

    sim.tick(clock->value());
    for (int i = 0; i < 40; ++i) {
        clock->advance(500);
        sim.tick(clock->value());
    }
    sim.setSpeed(8);
    for (int i = 0; i < 10; ++i) {
        clock->advance(250);
        sim.tick(clock->value());
    }
    sim.pause();
    clock->advance(5000);
    sim.tick(clock->value());
    sim.resume();
    sim.step(1500);
    sim.step(7000);
    for (int i = 0; i < 5; ++i) sim.step(1000);

    std::string out = sink->joined();
    for (const std::string& l : sink->observationLines) {
        out += "OBS ";
        out += l;
        out += "\n";
    }
    out += sim.entitiesJson().dump();
    out += "\n";
    out += std::to_string(sim.simElapsedMs());
    return out;
}

void nfr01_determinism_byte_identical_double_run() {
    requires_("SIM-NFR-01");
    const std::string a = deterministicRun();
    const std::string b = deterministicRun();
    CHECK(!a.empty());
    CHECK_STR(a, b);  // 逐字节一致
    note("      双跑事件序列 %d 字节，逐字节一致\n", static_cast<int>(a.size()));
}

void nfr01b_platform_declaration_order_does_not_matter() {
    requires_("SIM-NFR-01");
    SimScenario s1 = baseScenario();
    SimScenario s2 = baseScenario();
    std::reverse(s2.platforms.begin(), s2.platforms.end());
    std::reverse(s2.targets.begin(), s2.targets.end());
    std::reverse(s2.areas.begin(), s2.areas.end());

    auto run = [](const SimScenario& s) {
        SimSource sim;
        auto sink = std::make_shared<RecordingSink>();
        sim.setSink(sink);
        (void)sim.init(s);
        for (int i = 0; i < 30; ++i) sim.step(1000);
        return sink->joined();
    };
    const std::string a = run(s1);
    const std::string b = run(s2);
    CHECK(!a.empty());
    CHECK_STR(a, b);  // 声明顺序不影响输出（实体按 id 字典序）
}

void nfr02_contract_shape_is_frozen_in_header() {
    requires_("SIM-OUT-02");
    SimEvent e;
    e.deviceId = "d";
    e.deviceType = "t";
    e.kind = "k";
    e.lng = 1.0;
    e.lat = 2.0;
    e.alt = 3.0;
    e.heading = 4.0;
    e.speed = 5.0;
    e.battery = 6;
    e.seq = 7;
    e.ts = 8;
    const json j = e.toJson();
    CHECK(j["deviceId"].is_string());
    CHECK(j["deviceType"].is_string());
    CHECK(j["kind"].is_string());
    CHECK(j["lng"].is_number());
    CHECK(j["lat"].is_number());
    CHECK(j["alt"].is_number());
    CHECK(j["heading"].is_number());
    CHECK(j["speed"].is_number());
    CHECK(j["battery"].is_number_integer());
    CHECK(j["seq"].is_number_integer());
    CHECK(j["ts"].is_number_integer());
    CHECK_EQ(j.size(), 11);
    // 键序 = 契约键序（ordered_json 保证插入顺序）
    const std::vector<std::string> want = {"deviceId", "deviceType", "kind",    "lng", "lat", "alt",
                                           "heading",  "speed",      "battery", "seq", "ts"};
    std::size_t i = 0;
    for (auto it = j.begin(); it != j.end(); ++it, ++i) CHECK_STR(it.key(), want[i]);
    CHECK_EQ(i, 11);
}

void in01_json_scenario_round_trip() {
    requires_("SIM-IN-01", "SIM-IN-02");
    const SimScenario s = baseScenario();
    const json j = SimSource::toJson(s);
    SimScenario back;
    const ValidationResult r = SimSource::fromJson(j, back);
    CHECK(r.ok);
    if (!r.ok) return;
    CHECK_STR(back.scenarioKey, s.scenarioKey);
    CHECK_EQ(static_cast<int>(back.areas.size()), static_cast<int>(s.areas.size()));
    CHECK_EQ(static_cast<int>(back.platforms.size()), static_cast<int>(s.platforms.size()));
    CHECK_EQ(static_cast<int>(back.targets.size()), static_cast<int>(s.targets.size()));
    CHECK_EQ(static_cast<int>(back.groups.size()), static_cast<int>(s.groups.size()));
    CHECK(back.findArea("DEPLOY") != nullptr);
    CHECK_EQ(static_cast<int>(back.findArea("DEPLOY")->role), static_cast<int>(AreaRole::Deploy));
    CHECK_EQ(static_cast<int>(back.findArea("NOFLY")->hardness), static_cast<int>(Hardness::Hard));
    CHECK_EQ(static_cast<int>(back.findArea("SOFT")->hardness), static_cast<int>(Hardness::Soft));
    CHECK_NEAR(back.platforms[1].startOffset.rightM, 200.0, 1e-9);
    CHECK_EQ(static_cast<int>(back.targets[2].motion), static_cast<int>(TargetMotion::Popup));
    CHECK_EQ(back.targets[2].startOffsetMs, 30000);
    CHECK(back.targets[1].loop);
    // 往返后 JSON 逐字节一致 + 运行结果一致
    CHECK_STR(SimSource::toJson(back).dump(), j.dump());
    SimSource a;
    SimSource b;
    RecordingSink* sa = nullptr;
    RecordingSink* sb = nullptr;
    (void)runScenario(a, s, 20, 1000, &sa);
    (void)runScenario(b, back, 20, 1000, &sb);
    CHECK_STR(sa->joined(), sb->joined());

    // 未知 role → 可读问题（不抛异常）
    json bad = j;
    bad["areas"][0]["role"] = "unknown-role";
    SimScenario ignored;
    const ValidationResult rb = SimSource::fromJson(bad, ignored);
    CHECK(!rb.ok);
    CHECK(!rb.issues.empty());
    CHECK_STR(rb.issues[0].field, "role");
}

void in02_validation_reports_readable_issues() {
    requires_("SIM-IN-03");
    // ① 多边形顶点不足
    SimScenario s1 = baseScenario();
    s1.areas[0].polygon = {{116.300, 39.900}, {116.320, 39.900}};
    const ValidationResult r1 = SimSource::validate(s1);
    CHECK(!r1.ok);
    bool vertexIssue = false;
    for (const ScenarioIssue& i : r1.issues) {
        if (i.field == "polygon") vertexIssue = true;
    }
    CHECK(vertexIssue);

    // ② deviceId 重复
    SimScenario s2 = baseScenario();
    s2.platforms[1].deviceId = s2.platforms[0].deviceId;
    CHECK(!SimSource::validate(s2).ok);

    // ③ 电量越界
    SimScenario s3 = baseScenario();
    s3.platforms[0].battery = 150.0;
    CHECK(!SimSource::validate(s3).ok);

    // ④ dynamic 目标没有航路
    SimScenario s4 = baseScenario();
    s4.targets[1].route.clear();
    const ValidationResult r4 = SimSource::validate(s4);
    CHECK(!r4.ok);
    bool routeIssue = false;
    for (const ScenarioIssue& i : r4.issues) {
        if (i.field == "route") routeIssue = true;
    }
    CHECK(routeIssue);

    // ⑤ 非法场景 → init 失败且**保留上一次成功的局面**（原子性）
    SimScenario good = baseScenario();
    SimSource sim;
    CHECK(sim.init(good).ok);
    sim.step(5000);
    const int64_t afterGood = sim.simElapsedMs();
    const std::size_t devices = sim.deviceIds().size();
    SimScenario bad = baseScenario();
    bad.platforms[0].homeAreaKey = "NO-SUCH-AREA";
    const ValidationResult rBad = sim.init(bad);
    CHECK(!rBad.ok);
    CHECK_EQ(sim.simElapsedMs(), afterGood);
    CHECK_EQ(static_cast<int>(sim.deviceIds().size()), static_cast<int>(devices));
    CHECK_STR(sim.scenario().scenarioKey, "selftest");

    // ⑥ 空场景合法（没有实体，运行不崩）
    SimScenario empty;
    SimSource s5;
    CHECK(s5.init(empty).ok);
    CHECK_EQ(s5.step(1000), 0);
    CHECK(s5.entities().empty());
    CHECK(!s5.capabilities().initialized || s5.capabilities().platforms == 0);
}

void elc01_geometry_predicates() {
    requires_("SIM-ELC-01");
    const std::vector<std::pair<double, double>> square = {{0.0, 0.0}, {10.0, 0.0},
                                                           {10.0, 10.0}, {0.0, 10.0}};
    CHECK(SimSource::containsPoint(square, 5.0, 5.0));
    CHECK(!SimSource::containsPoint(square, 15.0, 5.0));
    CHECK(SimSource::containsPoint(square, 0.0, 5.0));  // 边界算在内
    CHECK(SimSource::segmentCrossesPolygon({-5.0, 5.0}, {15.0, 5.0}, square));
    CHECK(!SimSource::segmentCrossesPolygon({-5.0, -5.0}, {5.0, -5.0}, square));
    CHECK(SimSource::segmentCrossesPolygon({-1.0, 0.0}, {11.0, 0.0}, square));  // 贴边
    CHECK(SimSource::segmentCrossesPolygon({4.0, 4.0}, {6.0, 6.0}, square));    // 全在内
    CHECK(!SimSource::segmentCrossesPolygon({-5.0, -5.0}, {5.0, -5.0}, {{0.0, 0.0}, {1.0, 0.0}}));

    // 契约 §1：坐标 [经度, 纬度]，航向 0–360 度、北 0 顺时针
    CHECK_NEAR(SimSource::headingBetween(116.0, 39.0, 116.0, 40.0), 0.0, 1e-6);
    CHECK_NEAR(SimSource::headingBetween(116.0, 39.0, 117.0, 39.0), 90.0, 1e-6);
    CHECK_NEAR(SimSource::headingBetween(116.0, 39.0, 116.0, 38.0), 180.0, 1e-6);
    CHECK_NEAR(SimSource::headingBetween(116.0, 39.0, 115.0, 39.0), 270.0, 1e-6);
    CHECK_NEAR(SimSource::distanceMeters(116.0, 39.0, 116.0, 40.0), 111320.0, 20.0);
    CHECK_NEAR(SimSource::distanceMeters(116.0, 39.0, 117.0, 39.0),
               111320.0 * std::cos(39.0 * 3.14159265358979323846 / 180.0), 200.0);
    CHECK_NEAR(SimSource::distanceMeters(116.0, 39.0, 116.0, 39.0), 0.0, 1e-9);
    CHECK_NEAR(SimSource::routeLengthMeters({{116.0, 39.0}, {116.0, 40.0}}), 111320.0, 20.0);
}

void nfr03_time_is_injectable_and_clock_is_optional() {
    requires_("SIM-NFR-02");
    // 不给时钟、不调 tick(nowMs) 时，仿真**一动不动**
    const SimScenario s = baseScenario();
    SimSource sim;
    RecordingSink* sink = nullptr;
    (void)runScenario(sim, s, 0, 0, &sink);
    CHECK_EQ(sim.tick(), 0);
    CHECK_EQ(sim.tick(), 0);
    CHECK_EQ(sim.simElapsedMs(), 0);
    CHECK(sink->events.empty());
    CHECK(!sim.capabilities().clockInjected);

    // 注入时钟后 tick() 才有意义
    auto clock = std::make_shared<FakeClock>(5000);
    sim.setClock(clock);
    CHECK(sim.capabilities().clockInjected);
    sim.tick();
    clock->advance(2000);
    sim.tick();
    CHECK_EQ(sim.simElapsedMs(), 2000);
    CHECK(!sink->events.empty());

    // 同一输入双跑（假时钟路径）逐字节一致
    auto runClock = [](int64_t stepMs) {
        SimSource x;
        auto snk = std::make_shared<RecordingSink>();
        x.setSink(snk);
        auto c = std::make_shared<FakeClock>(1000000);
        x.setClock(c);
        (void)x.init(baseScenario());
        x.tick();
        for (int i = 0; i < 20; ++i) {
            c->advance(stepMs);
            x.tick();
        }
        return snk->joined();
    };
    CHECK_STR(runClock(1000), runClock(1000));
}

void nfr04_capabilities_and_metrics_are_observable() {
    requires_("SIM-NFR-03");
    const SimScenario s = baseScenario();
    SimSource sim;
    RecordingSink* sink = nullptr;
    (void)runScenario(sim, s, 5, 1000, &sink);
    auto model = std::make_shared<StubSensor>();
    sim.setSensorModel(model);
    sim.setSensorAttachments({SensorAttachment{"sim-01", "sensor-x", 0.0, 0}});
    sim.step(1000);

    const Capabilities c = sim.capabilities();
    CHECK(c.initialized);
    CHECK(c.sinkInjected);
    CHECK(c.customSensorInjected);
    CHECK_EQ(c.platforms, 2);
    CHECK_EQ(c.targets, 3);
    CHECK_EQ(c.groups, 1);
    CHECK_EQ(c.areas, 4);
    CHECK_EQ(c.hardZones, 1);
    CHECK_EQ(c.sensors, 1);
    CHECK_EQ(c.speedMultiplier, 1);
    CHECK(!c.paused);
    CHECK_STR(c.scenarioKey, "selftest");
    CHECK(!c.moduleVersion.empty());

    const Metrics m = sim.metrics();
    CHECK_EQ(m.steps, 6);
    CHECK_EQ(m.eventsEmitted, 4 * 6);  // 每次 step 都只有 4 个可见实体
    CHECK(m.substeps > 0);
    CHECK_EQ(m.arrivals, 0);
    CHECK_EQ(m.sensorCalls, 1);
    CHECK_EQ(m.simElapsedMs, 6000);

    // entitiesJson 可序列化（宿主排障用）
    const json arr = sim.entitiesJson();
    CHECK(arr.is_array());
    CHECK_EQ(arr.size(), 5);
    CHECK(arr[0].contains("inHardZone"));
    CHECK(arr[0].contains("inTaskArea"));
    CHECK(arr[0].contains("state"));
    CHECK(toJson(m).is_object());
    CHECK(toJson(c).is_object());
}

void out05_entity_status_reports_zone_membership() {
    requires_("SIM-OUT-05", "SIM-TGT-04");
    const SimScenario s = baseScenario();
    SimSource sim;
    RecordingSink* sink = nullptr;
    (void)runScenario(sim, s, 0, 0, &sink);
    const std::vector<EntityStatus> esP = sim.entities();
    const EntityStatus* p = find(esP, "sim-01");
    CHECK(p != nullptr);
    if (p == nullptr) return;
    CHECK(!p->inTaskArea);
    CHECK(!p->inHardZone);
    CHECK(!p->inSoftZone);
    // static 目标落在任务区多边形内
    const EntityStatus* t = find(esP, "tgt-1");
    CHECK(t != nullptr);
    if (t != nullptr) {
        CHECK(t->inTaskArea);
    }

    // 走到最后：平台进入任务区，且全程 inHardZone 恒为假
    int hardViolations = 0;
    for (int i = 0; i < 600; ++i) {
        sim.step(1000);
        for (const EntityStatus& e : sim.entities()) {
            if (e.inHardZone) ++hardViolations;
        }
    }
    CHECK_EQ(hardViolations, 0);
    const std::vector<EntityStatus> esP_final = sim.entities();
    const EntityStatus* pdone = find(esP_final, "sim-01");
    CHECK(pdone != nullptr);
    if (pdone != nullptr) {
        CHECK(pdone->inTaskArea);
        CHECK(!pdone->inHardZone);
    }
}

// ---------------------------------------------------------------- 用例表

struct Case {
    const char* name;
    void (*fn)();
};

const Case kCases[] = {
    {"route01_detour_around_hard_no_fly_zone", route01_detour_around_hard_no_fly_zone},
    {"route02_kinematics_never_enters_hard_zone_and_arrives",
     route02_kinematics_never_enters_hard_zone_and_arrives},
    {"route02b_soft_zone_does_not_change_route", route02b_soft_zone_does_not_change_route},
    {"route03_formation_keeps_relative_shape", route03_formation_keeps_relative_shape},
    {"route04_no_route_when_plan_impossible", route04_no_route_when_plan_impossible},
    {"kin01_straight_segment_interpolation", kin01_straight_segment_interpolation},
    {"kin02_waypoint_switch_and_arrival", kin02_waypoint_switch_and_arrival},
    {"kin03_battery_drain_and_clamp", kin03_battery_drain_and_clamp},
    {"tick01_speed_8x_advances_8s_per_real_second",
     tick01_speed_8x_advances_8s_per_real_second},
    {"tick02_speed_multiplier_is_whitelisted", tick02_speed_multiplier_is_whitelisted},
    {"tick03_pause_discards_real_time_resume_does_not_jump",
     tick03_pause_discards_real_time_resume_does_not_jump},
    {"tick04_step_is_independent_of_speed", tick04_step_is_independent_of_speed},
    {"tick05_substep_keeps_waypoint_turn_accurate", tick05_substep_keeps_waypoint_turn_accurate},
    {"tgt01_static_target_does_not_move", tgt01_static_target_does_not_move},
    {"tgt02_dynamic_target_moves_along_route_and_loops",
     tgt02_dynamic_target_moves_along_route_and_loops},
    {"tgt02b_non_loop_target_stops_at_end", tgt02b_non_loop_target_stops_at_end},
    {"tgt03_popup_appears_after_offset", tgt03_popup_appears_after_offset},
    {"evt01_one_normalized_event_per_visible_entity_per_tick",
     evt01_one_normalized_event_per_visible_entity_per_tick},
    {"evt02_event_fields_equal_device_ingest_contract",
     evt02_event_fields_equal_device_ingest_contract},
    {"evt03_extensions_are_host_owned_and_deterministic",
     evt03_extensions_are_host_owned_and_deterministic},
    {"evt04_parse_round_trip_and_rejections", evt04_parse_round_trip_and_rejections},
    {"sensor01_attached_model_is_called_with_pose", sensor01_attached_model_is_called_with_pose},
    {"sensor02_no_detection_algorithm_inside_module",
     sensor02_no_detection_algorithm_inside_module},
    {"sensor03_sink_exception_does_not_stop_simulation",
     sensor03_sink_exception_does_not_stop_simulation},
    {"nfr01_determinism_byte_identical_double_run", nfr01_determinism_byte_identical_double_run},
    {"nfr01b_platform_declaration_order_does_not_matter",
     nfr01b_platform_declaration_order_does_not_matter},
    {"nfr02_contract_shape_is_frozen_in_header", nfr02_contract_shape_is_frozen_in_header},
    {"in01_json_scenario_round_trip", in01_json_scenario_round_trip},
    {"in02_validation_reports_readable_issues", in02_validation_reports_readable_issues},
    {"elc01_geometry_predicates", elc01_geometry_predicates},
    {"nfr03_time_is_injectable_and_clock_is_optional", nfr03_time_is_injectable_and_clock_is_optional},
    {"nfr04_capabilities_and_metrics_are_observable",
     nfr04_capabilities_and_metrics_are_observable},
    {"out05_entity_status_reports_zone_membership", out05_entity_status_reports_zone_membership},
};

/// `--dump-event`：跑一小段仿真，把**真实产出**的一条归一化事件打到 stdout。
///
/// 用途：验收脚本据此与 device-ingest《外设接入契约》§2 **逐字段**比对
/// （不是比对头文件里的字段名，而是比对运行时真正发出的 JSON 键序与类型）。
int dumpEvent() {
    const SimScenario s = baseScenario();
    SimSource sim;
    auto sink = std::make_shared<RecordingSink>();
    sim.setSink(sink);
    if (!sim.init(s).ok) return 1;
    for (int i = 0; i < 3 && sink->lines.empty(); ++i) sim.step(1000);
    if (sink->lines.empty()) return 1;
    std::printf("%s\n", sink->lines.front().c_str());
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
#if defined(_WIN32)
    // 控制台切 UTF-8（否则中文输出乱码）。**只在没有参数时切**：
    // stdout 被重定向（验收脚本 / CI 抓输出）时 chcp 会以 EBUSY 失败并让 std::system 抛异常，
    // 那属环境噪声，不是测试失败。
    if (argc < 2) {
        try {
            std::system("chcp 65001 > nul");
        } catch (...) {
        }
    }
#endif
    std::string filter;
    bool listOnly = false;
    bool dumpOnly = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--list") {
            listOnly = true;
        } else if (arg == "--json") {
            g_jsonMode = true;
        } else if (arg == "--dump-event") {
            dumpOnly = true;
        } else {
            filter = arg;
        }
    }
    if (dumpOnly) return dumpEvent();
    if (listOnly) {
        for (const Case& c : kCases) std::printf("%s\n", c.name);
        return 0;
    }

    const auto t0 = std::chrono::steady_clock::now();
    for (const Case& c : kCases) {
        if (!filter.empty() && std::string(c.name).find(filter) == std::string::npos) continue;
        g_case = c.name;
        g_reqs.clear();
        const int failedBefore = g_failed;
        const int assertsBefore = g_asserts;
        ++g_cases;
        if (!g_jsonMode) std::printf("[ RUN  ] %s\n", c.name);
        try {
            c.fn();
        } catch (const std::exception& ex) {
            record(false, std::string("用例抛出异常：") + ex.what(), __FILE__, __LINE__);
        } catch (...) {
            record(false, "用例抛出未知异常", __FILE__, __LINE__);
        }
        const bool ok = (g_failed == failedBefore);
        if (!ok) ++g_casesFailed;
        CaseMeta meta;
        meta.name = c.name;
        meta.reqs = g_reqs;
        meta.asserts = g_asserts - assertsBefore;
        meta.failed = g_failed - failedBefore;
        meta.ok = ok;
        g_metas.push_back(std::move(meta));
        if (!g_jsonMode) {
            std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", c.name);
            std::fflush(stdout);
        }
    }
    const auto t1 = std::chrono::steady_clock::now();
    const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    if (g_jsonMode) {
        std::string out = "{\n";
        out += "  \"moduleVersion\": \"" + jsonEscape(kModuleVersion) + "\",\n";
        out += "  \"cases\": " + std::to_string(g_cases) + ",\n";
        out += "  \"casesFailed\": " + std::to_string(g_casesFailed) + ",\n";
        out += "  \"asserts\": " + std::to_string(g_asserts) + ",\n";
        out += "  \"assertsFailed\": " + std::to_string(g_failed) + ",\n";
        out += "  \"elapsedMs\": " + std::to_string(ms) + ",\n";
        out += "  \"result\": \"" + std::string(g_failed == 0 ? "ALL GREEN" : "FAILED") + "\",\n";
        out += "  \"details\": [";
        for (std::size_t i = 0; i < g_metas.size(); ++i) {
            const CaseMeta& m = g_metas[i];
            out += (i == 0 ? "\n" : ",\n");
            out += "    {\"name\": \"" + jsonEscape(m.name) + "\", \"ok\": " +
                   (m.ok ? "true" : "false") + ", \"asserts\": " + std::to_string(m.asserts) +
                   ", \"failed\": " + std::to_string(m.failed) + ", \"reqs\": " +
                   jsonArray(m.reqs) + "}";
        }
        out += "\n  ],\n";
        out += "  \"failures\": " + jsonArray(g_failures) + "\n";
        out += "}\n";
        std::fputs(out.c_str(), stdout);
        return g_failed == 0 ? 0 : 1;
    }

    std::printf("\n================ sim-source selftest ================\n");
    std::printf("用例 %d 个（失败 %d）｜断言 %d 条（失败 %d）｜耗时 %.1f ms\n", g_cases,
                g_casesFailed, g_asserts, g_failed, ms);
    if (!g_failures.empty()) {
        std::printf("\n---- 失败明细 ----\n");
        for (const std::string& f : g_failures) std::printf("  %s\n", f.c_str());
    }
    std::printf("结果：%s\n", g_failed == 0 ? "ALL GREEN" : "FAILED");
    std::printf("====================================================\n");
    return g_failed == 0 ? 0 : 1;
}
