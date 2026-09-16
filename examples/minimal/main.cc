// examples/minimal/main.cc · 最小示例：一个部署区 → 一个任务区 → 一条硬禁飞区挡路
//
// 演示最小闭环：中立场景进 → 航路规划（绕行）→ 定步长推进 → 归一化事件出。
// 事件出口是**宿主实现**的 ISimSink（这里是最简单的收集器），本模块自己不发任何东西。
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "sim_source/sim_source.h"

using namespace sim_source;

namespace {

/// 示例用的事件收集器（宿主侧最简实现：只打印前几条，其余只计数）
class CollectingSink : public ISimSink {
public:
    void onEvent(const SimEvent& e) override {
        ++count;
        if (printed < 4) {
            ++printed;
            std::printf("    %s\n", e.toJson().dump().c_str());
        }
    }
    void onObservation(const SimObservation& o) override {
        (void)o;
        ++observations;
    }
    int count = 0;
    int observations = 0;
    int printed = 0;
};

/// 一条直穿禁飞区的路线：部署区在西，任务区在东，禁飞区正好在中间
SimScenario makeScenario() {
    SimScenario s;
    s.scenarioKey = "example-minimal";

    Area deploy;
    deploy.key = "A-deploy";
    deploy.name = "部署区";
    deploy.role = AreaRole::Deploy;
    deploy.polygon = {{116.300, 39.900}, {116.320, 39.900}, {116.320, 39.920}, {116.300, 39.920}};
    s.areas.push_back(deploy);

    Area nofly;
    nofly.key = "Z-nofly";
    nofly.name = "禁飞区（hard）";
    nofly.role = AreaRole::NoFly;
    nofly.hardness = Hardness::Hard;
    nofly.hasHardness = true;
    nofly.polygon = {{116.350, 39.895}, {116.365, 39.895}, {116.365, 39.915}, {116.350, 39.915}};
    s.areas.push_back(nofly);

    Area task;
    task.key = "A-task";
    task.name = "任务区";
    task.role = AreaRole::Task;
    task.polygon = {{116.395, 39.900}, {116.415, 39.900}, {116.415, 39.920}, {116.395, 39.920}};
    s.areas.push_back(task);

    Group g;
    g.key = "G-1";
    g.name = "演示编队";
    s.groups.push_back(g);

    Platform p;
    p.deviceId = "sim-001";
    p.deviceType = "uav";
    p.kind = "uav.pos";  // kind 由宿主注入（模块只当字符串）
    p.groupKey = "G-1";
    p.homeAreaKey = "A-deploy";
    p.taskAreaKey = "A-task";
    p.altM = 900.0;
    p.speedMps = 50.0;
    s.platforms.push_back(p);
    return s;
}

}  // namespace

int main() {
    std::printf("== sim-source 最小示例：编队航路规划 + 时间推进 ==\n");

    const SimScenario scenario = makeScenario();
    const ValidationResult vr = SimSource::validate(scenario);
    if (!vr.ok) {
        std::printf("场景校验未通过：\n");
        for (const ScenarioIssue& i : vr.issues) {
            std::printf("  - %s.%s：%s\n", i.path.c_str(), i.field.c_str(), i.reason.c_str());
        }
        return 1;
    }

    // ① 静态规划（与 init 走同一条实现路径）
    const PlanResult plan = SimSource::planRoutes(scenario);
    const auto* route = plan.routeOf("sim-001");
    if (route == nullptr) {
        std::printf("没有规划出航路\n");
        return 1;
    }
    // 直连长度 vs 实际航路长度：绕行的代价一眼可见
    const double directLen =
        SimSource::distanceMeters(scenario.areas[0].centroidLng(), scenario.areas[0].centroidLat(),
                                  scenario.areas[2].centroidLng(), scenario.areas[2].centroidLat());
    const double planLen = SimSource::routeLengthMeters(*route);
    const bool detoured = planLen > directLen * 1.01;
    std::printf("\n规划结果：hard 禁飞区 %d 个；本平台绕行 = %s；航路长 %.1f m（%d 个航路点）\n",
                plan.hardZones, detoured ? "是" : "否", SimSource::routeLengthMeters(*route),
                static_cast<int>(route->size()));
    for (std::size_t i = 0; i < route->size(); ++i) {
        std::printf("    wp%d  lng=%.6f lat=%.6f\n", static_cast<int>(i), (*route)[i].first,
                    (*route)[i].second);
    }
    const bool clear = SimSource::routeClearOfHardZones(*route, scenario);
    std::printf("逐段复核：航路从未穿入 hard 禁飞区 = %s\n", clear ? "是" : "否");

    // ② 装载并推进
    SimSource sim;
    auto sink = std::make_shared<CollectingSink>();
    sim.setSink(sink);
    if (!sim.init(scenario).ok) {
        std::printf("init 失败\n");
        return 1;
    }


    // 定步长推进 60 秒（与倍速无关；倍速只影响 tick 把真实时间换算成仿真时间的口径）
    for (int i = 0; i < 60; ++i) sim.step(1000);
    std::printf("\n推进 %.1f s 仿真时间后：\n", static_cast<double>(sim.simElapsedMs()) / 1000.0);
    for (const EntityStatus& e : sim.entities()) {
        std::printf("    %-8s state=%-8s lng=%.6f lat=%.6f battery=%.1f seq=%lld 在禁飞区=%s\n",
                    e.id.c_str(), toString(e.state), e.lng, e.lat, e.battery,
                    static_cast<long long>(e.seq), e.inHardZone ? "是" : "否");
    }

    const Metrics m = sim.metrics();
    std::printf("\n事件 %lld 条｜到达 %lld 次｜Sink 实收 %d 条\n",
                static_cast<long long>(m.eventsEmitted), static_cast<long long>(m.arrivals),
                sink->count);
    if (!clear || sink->count == 0) {
        std::printf("示例失败：航路或事件不符合预期\n");
        return 1;
    }
    std::printf("示例通过（退出码 0）\n");
    return 0;
}
