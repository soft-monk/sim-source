// examples/full_flow/main.cc · 全流程演示
//
// 演示本模块**全部**功能面（README「做 / 不做」表中"做"的那一列）：
//   ① 编队航路规划：两机编队，部署区 → 任务区，绕开 hard 禁飞区，保持相对位形；
//   ② 实体运动学：位置 / 航向 / 速度 / 电量按注入时钟推进；
//   ③ 节拍控制：1× / 8×（真实时间驱动）与 pause / resume / step；
//   ④ 目标运动：static / dynamic(loop) / popup；
//   ⑤ 事件产出：每 tick 每个可见实体一条归一化事件 → ISimSink；
//   ⑥ 传感器挂接：ISensorModel 抽象接口 + 一个**最简宿主实现**（本模块不含探测算法）。
//
// 事件出口与探测模型都是**宿主侧**实现（下面的 HostSink / RangeOnlySensor）：
// 本模块自己不发 UDP、不广播、不做判定。
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "sim_source/sim_source.h"

using namespace sim_source;

namespace {

// ============================================================================
// 宿主侧：事件出口
// ============================================================================
class HostSink : public ISimSink {
public:
    void onEvent(const SimEvent& e) override {
        ++events;
        eventsByDevice[e.deviceId]++;
        if (sampleEvents < 2) {
            ++sampleEvents;
            std::printf("      [event ] %s\n", e.toJson().dump().c_str());
        }
    }
    void onObservation(const SimObservation& o) override {
        ++observations;
        if (sampleObservations < 2) {
            ++sampleObservations;
            std::printf("      [observ] %s\n", o.toJson().dump().c_str());
        }
    }
    long long events = 0;
    long long observations = 0;
    std::map<std::string, long long> eventsByDevice;

private:
    int sampleEvents = 0;
    int sampleObservations = 0;
};

// ============================================================================
// 宿主侧：探测模型（**演示用的最简策略**，绝不是本模块的一部分）
//
// 本模块只把"平台位姿 + 此刻可见的全部候选位姿"交出去，谁被发现由这里判定。
// 这里为了演示能跑出观测，用了一条极简规则：候选实体的**水平距离**在模型自报的
// 作用距离内则产出一条观测（置信度按距离线性衰减）。真实探测模型在 sensor-model 仓。
// ============================================================================
class RangeOnlySensor : public ISensorModel {
public:
    std::string id() const override { return "demo.range-only.v1"; }
    std::string deviceType() const override { return "radar"; }

    std::vector<SimObservation> sense(const SensorPose& pose) override {
        ++calls;
        std::vector<SimObservation> out;
        const double range = (pose.rangeM > 0.0) ? pose.rangeM : 20000.0;
        for (const EntityPose& c : pose.candidates) {
            const double d = distanceMeters(pose.self.lng, pose.self.lat, c.lng, c.lat);
            if (d > range) continue;
            SimObservation o;
            o.sensorId = pose.sensorId;
            o.deviceType = pose.deviceType;
            o.kind = "detect.pos";
            o.targetId = c.platformId;
            o.targetType = c.deviceType;
            o.lng = c.lng;
            o.lat = c.lat;
            o.altM = c.altM;
            o.heading = c.heading;
            o.speedMps = c.speedMps;
            o.confidence = std::max(0.05, 1.0 - d / range);
            o.ts = pose.ts;
            o.extensions["rangeM"] = range;
            out.push_back(o);
        }
        return out;
    }
    long long calls = 0;

private:
    static double distanceMeters(double lng1, double lat1, double lng2, double lat2) {
        const double mPerDegLat = 111320.0;
        const double mPerDegLng = 111320.0 * std::cos(lat1 * 3.14159265358979323846 / 180.0);
        const double dx = (lng2 - lng1) * mPerDegLng;
        const double dy = (lat2 - lat1) * mPerDegLat;
        return std::sqrt(dx * dx + dy * dy);
    }
};

// ============================================================================
// 演示场景（**演示数据**，合法住所就是 examples/）
// ============================================================================
SimScenario makeScenario() {
    SimScenario s;
    s.scenarioKey = "demo-full-flow";

    Area deploy;
    deploy.key = "DEPLOY-01";
    deploy.name = "部署区";
    deploy.role = AreaRole::Deploy;
    deploy.polygon = {{116.300, 39.900}, {116.320, 39.900}, {116.320, 39.920}, {116.300, 39.920}};
    s.areas.push_back(deploy);

    Area nofly;
    nofly.key = "NOFLY-HARD";
    nofly.name = "禁飞区（hard，挡住直连航线）";
    nofly.role = AreaRole::NoFly;
    nofly.hardness = Hardness::Hard;
    nofly.hasHardness = true;
    nofly.polygon = {{116.350, 39.895}, {116.365, 39.895}, {116.365, 39.915}, {116.350, 39.915}};
    s.areas.push_back(nofly);

    Area soft;
    soft.key = "NOFLY-SOFT";
    soft.name = "限制区（soft，只记录不绕行）";
    soft.role = AreaRole::NoFly;
    soft.hardness = Hardness::Soft;
    soft.hasHardness = true;
    soft.polygon = {{116.375, 39.905}, {116.385, 39.905}, {116.385, 39.912}, {116.375, 39.912}};
    s.areas.push_back(soft);

    Area task;
    task.key = "TASK-01";
    task.name = "任务区";
    task.role = AreaRole::Task;
    task.polygon = {{116.395, 39.900}, {116.415, 39.900}, {116.415, 39.920}, {116.395, 39.920}};
    s.areas.push_back(task);

    Group g;
    g.key = "G-A";
    g.name = "编队 A";
    g.role = "recon";
    s.groups.push_back(g);

    // 编队位形：0 号（领队，deviceId 字典序最小）居前，1 号右后 300 m
    Platform p1;
    p1.deviceId = "sim-a1";
    p1.deviceType = "uav";
    p1.kind = "uav.pos";
    p1.groupKey = "G-A";
    p1.homeAreaKey = "DEPLOY-01";
    p1.taskAreaKey = "TASK-01";
    p1.altM = 900.0;
    p1.speedMps = 60.0;
    s.platforms.push_back(p1);

    Platform p2;
    p2.deviceId = "sim-a2";
    p2.deviceType = "uav";
    p2.kind = "uav.pos";
    p2.groupKey = "G-A";
    p2.homeAreaKey = "DEPLOY-01";
    p2.taskAreaKey = "TASK-01";
    p2.altM = 950.0;
    p2.speedMps = 60.0;
    p2.startOffset.rightM = 300.0;
    p2.startOffset.fwdM = -200.0;
    s.platforms.push_back(p2);

    // 目标 ①：static（原地不动）
    Target t1;
    t1.no = 1;
    t1.id = "tgt-static";
    t1.typeKey = "fixed";
    t1.deviceType = "target";
    t1.kind = "target.pos";
    t1.name = "固定目标";
    t1.motion = TargetMotion::Static;
    t1.hasPosition = true;
    t1.position = {116.400, 39.916};
    s.targets.push_back(t1);

    // 目标 ②：dynamic + loop（按航线绕圈）
    Target t2;
    t2.no = 2;
    t2.id = "tgt-dynamic";
    t2.typeKey = "mobile";
    t2.deviceType = "target";
    t2.kind = "target.pos";
    t2.name = "机动目标";
    t2.motion = TargetMotion::Dynamic;
    t2.route = {{116.392, 39.902}, {116.400, 39.918}, {116.412, 39.904}, {116.400, 39.918}};
    t2.speedMps = 25.0;
    t2.loop = true;
    s.targets.push_back(t2);

    // 目标 ③：popup（30 s 后才出现）
    Target t3;
    t3.no = 3;
    t3.id = "tgt-popup";
    t3.typeKey = "popup";
    t3.deviceType = "target";
    t3.kind = "target.pos";
    t3.name = "临机目标";
    t3.motion = TargetMotion::Popup;
    t3.route = {{116.398, 39.906}, {116.410, 39.914}};
    t3.speedMps = 40.0;
    t3.startOffsetMs = 30000;
    s.targets.push_back(t3);

    return s;
}

void section(const char* title) { std::printf("\n---- %s ----\n", title); }

}  // namespace

int main() {
    int failures = 0;
    std::printf("== sim-source 全流程演示 ==\n");

    const SimScenario scenario = makeScenario();
    const ValidationResult vr = SimSource::validate(scenario);
    if (!vr.ok) {
        std::printf("场景校验未通过：\n");
        for (const ScenarioIssue& i : vr.issues) {
            std::printf("  - %s.%s：%s\n", i.path.c_str(), i.field.c_str(), i.reason.c_str());
        }
        return 1;
    }

    // ---------------------------------------------------------------- ① 规划
    section("① 编队航路规划（部署区 → 任务区，绕开 hard 禁飞区，保持位形）");
    const PlanResult plan = SimSource::planRoutes(scenario);
    std::printf("    hard 禁飞区 %d 个｜soft 限制区 %d 个｜需要绕行的编组 %d 个\n", plan.hardZones,
                plan.softZones, plan.detoured);
    for (std::size_t i = 0; i < plan.platformIds.size(); ++i) {
        const auto& r = plan.routes[i];
        const bool clear = SimSource::routeClearOfHardZones(r, scenario);
        std::printf("    %-7s 组=%-4s %d 个航路点  长 %8.1f m  从未穿入 hard = %s\n",
                    plan.platformIds[i].c_str(), plan.groupKeys[i].c_str(),
                    static_cast<int>(r.size()), SimSource::routeLengthMeters(r), clear ? "是" : "否");
        if (!clear) ++failures;
    }

    // ---------------------------------------------------------------- ② 装载
    SimSource sim;
    auto sink = std::make_shared<HostSink>();
    auto sensor = std::make_shared<RangeOnlySensor>();
    sim.setSink(sink);
    sim.setSensorModel(sensor);
    sim.setSensorAttachments({SensorAttachment{"sim-a1", "sensor-a1", 20000.0, 1000}});
    if (!sim.init(scenario).ok) {
        std::printf("init 失败\n");
        return 1;
    }

    // ---------------------------------------------------------------- ③ 1× 推进
    section("③ 节拍：真实时间驱动（1×）→ 10 s 仿真时间");
    std::int64_t now = 1789000000000LL;
    (void)sim.tick(now);  // 首次只记基线
    now += 10000;
    const int ev1 = sim.tick(now);
    std::printf("    1× 推进：仿真 %.1f s｜本节拍事件 %d 条｜累计事件 %lld 条\n",
                static_cast<double>(sim.simElapsedMs()) / 1000.0, ev1,
                static_cast<long long>(sim.metrics().eventsEmitted));
    if (sim.simElapsedMs() != 10000) {
        std::printf("    !! 期望 10 s，实际 %.3f s\n",
                    static_cast<double>(sim.simElapsedMs()) / 1000.0);
        ++failures;
    }

    // ---------------------------------------------------------------- ④ 8× 推进
    section("④ 节拍：倍速 8×（1 s 真实时间 → 8 s 仿真时间）");
    if (!sim.setSpeed(8)) {
        std::printf("    !! setSpeed(8) 被拒\n");
        ++failures;
    }
    if (sim.setSpeed(7)) {
        std::printf("    !! setSpeed(7) 本应被拒\n");
        ++failures;
    }
    const std::int64_t before = sim.simElapsedMs();
    now += 1000;
    const int ev8 = sim.tick(now);
    const std::int64_t delta = sim.simElapsedMs() - before;
    std::printf("    8× 推进 1.000 s 真实时间 → 仿真 +%.3f s｜本节拍事件 %d 条\n",
                static_cast<double>(delta) / 1000.0, ev8);
    if (delta != 8000) {
        std::printf("    !! 期望 8000 ms，实际 %lld ms\n", static_cast<long long>(delta));
        ++failures;
    }
    sim.setSpeed(1);

    // ---------------------------------------------------------------- ⑤ 暂停 / 单步
    section("⑤ 节拍：pause / resume / step(dtMs)");
    sim.pause();
    const std::int64_t pausedAt = sim.simElapsedMs();
    now += 60000;  // 暂停期间真实时间过了 60 s
    sim.tick(now);
    std::printf("    pause 后真实时间 +60 s → 仿真时间 +%lld ms（暂停期间 MUST NOT 推进）\n",
                static_cast<long long>(sim.simElapsedMs() - pausedAt));
    if (sim.simElapsedMs() != pausedAt) ++failures;
    sim.resume();
    const int evStep = sim.step(5000);  // 定步长：与倍速无关
    std::printf("    resume 后 step(5000) → 仿真 +%lld ms｜事件 %d 条\n",
                static_cast<long long>(sim.simElapsedMs() - pausedAt), evStep);
    if (sim.simElapsedMs() - pausedAt != 5000) ++failures;

    // ---------------------------------------------------------------- ⑥ 目标三态
    section("⑥ 目标运动：static / dynamic(loop) / popup");
    auto show = [&](const char* when) {
        std::printf("    [%s] 仿真时刻 %.1f s\n", when,
                    static_cast<double>(sim.simElapsedMs()) / 1000.0);
        for (const EntityStatus& e : sim.entities()) {
            if (!e.isTarget) continue;
            std::printf("      %-12s 出现=%-3s 状态=%-8s lng=%.6f lat=%.6f 航段=%d\n", e.id.c_str(),
                        e.visible ? "是" : "否", toString(e.state), e.lng, e.lat, e.waypointIndex);
        }
    };
    show("当前");
    for (int i = 0; i < 30; ++i) sim.step(1000);  // 推进到 45 s：popup(30 s) 应已出现
    show("再推进 30 s");

    // ---------------------------------------------------------------- ⑦ 到达与传感器
    section("⑦ 到达任务区 + 传感器挂接（ISensorModel 由宿主实现）");
    for (int i = 0; i < 600; ++i) sim.step(1000);  // 最多再推进 600 s
    for (const EntityStatus& e : sim.entities()) {
        std::printf("    %-12s 目标=%-3s 状态=%-8s 在任务区=%-3s 在 hard 禁飞区=%-3s 电量=%.1f\n",
                    e.id.c_str(), e.isTarget ? "是" : "否", toString(e.state),
                    e.inTaskArea ? "是" : "否", e.inHardZone ? "是" : "否", e.battery);
        if (e.inHardZone) ++failures;
    }

    const Metrics m = sim.metrics();
    const Capabilities caps = sim.capabilities();
    std::printf("\n    事件 %lld 条｜观测 %lld 条｜探测调用 %lld 次（宿主模型自报）\n",
                static_cast<long long>(m.eventsEmitted),
                static_cast<long long>(m.observationsEmitted), static_cast<long long>(sensor->calls));
    std::printf("    Sink 实收：事件 %lld 条｜观测 %lld 条\n", sink->events, sink->observations);
    std::printf("    capabilities：平台 %d｜目标 %d｜编组 %d｜挂接传感器 %d｜hard 禁飞区 %d\n",
                caps.platforms, caps.targets, caps.groups, caps.sensors, caps.hardZones);

    if (sink->events != m.eventsEmitted) {
        std::printf("    !! Sink 实收事件数与 metrics 不一致\n");
        ++failures;
    }
    if (m.observationsEmitted == 0) {
        std::printf("    !! 没有产出任何观测（检查传感器挂接）\n");
        ++failures;
    }
    if (m.arrivals == 0) {
        std::printf("    !! 没有任何实体到达任务区\n");
        ++failures;
    }

    section("结论");
    if (failures == 0) {
        std::printf("全流程演示通过（退出码 0）\n");
        return 0;
    }
    std::printf("全流程演示失败：%d 项不符（退出码 1）\n", failures);
    return 1;
}
