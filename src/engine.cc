// src/engine.cc · 仿真源主实现（时间推进 / 运动学 / 事件产出 / 传感器挂接）
//
// 时间口径（全部由这里定义，别处不取挂钟）：
//   · **仿真时间是唯一真相**：simElapsedMs 只被 step()/tick() 推进；事件 ts = epoch0 + 仿真时间。
//   · step(dtMs)  —— 推进固定仿真毫秒，与倍速无关（离线复算 / 自测入口）。
//   · tick(nowMs) —— (nowMs - 上次) × 倍速 → 仿真毫秒，再按 substepMs 切子步（拐点不失真）。
//   · pause() 期间 tick 丢弃真实时间、step 不推进；resume() 从此刻重新计时（不"跳一下"）。
#include "internal.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sim_source {
namespace detail {

namespace {
/// 某段航路的方向角（度）；退化段返回 -1（表示"这一段没有方向"）
double segmentHeading(const Vec2& a, const Vec2& b) {
    const Vec2 d = b - a;
    const double l = d.len();
    if (!(l > 1e-9)) return -1.0;
    return normalizeHeading(std::atan2(d.x, d.y) * 180.0 / kPi);
}
}  // namespace

void buildArc(const std::vector<Vec2>& route, std::vector<double>& cum, double& total) {
    cum.clear();
    cum.reserve(route.size());
    if (route.empty()) {
        total = 0.0;
        return;
    }
    cum.push_back(0.0);
    for (std::size_t i = 0; i + 1 < route.size(); ++i) {
        cum.push_back(cum.back() + (route[i + 1] - route[i]).len());
    }
    total = cum.back();
}

PosAt locate(const std::vector<Vec2>& route, const std::vector<double>& cum, double total,
             double arc, bool loop, std::size_t& segIdx) {
    PosAt out;
    if (route.empty()) {
        segIdx = 0;
        return out;
    }
    if (route.size() == 1 || !(total > 0.0)) {
        out.p = route.front();
        out.segPos = 0.0;
        out.segLen = 0.0;
        segIdx = 0;
        return out;
    }
    double s = arc;
    if (loop) {
        s = std::fmod(s, total);
        if (s < 0.0) s += total;
        // 环线闭环：从最后一点回到第一点（仅当最后一点不是第一点时才有这段）
        const double closeLen = (cum.back() < total) ? (total - cum.back()) : 0.0;
        if (closeLen > 0.0 && s >= cum.back()) {
            const double t = (s - cum.back()) / closeLen;
            out.p = route.back() + (route.front() - route.back()) * t;
            segIdx = route.size() - 1;
            out.segPos = s - cum.back();
            out.segLen = closeLen;
            return out;
        }
    } else if (s >= total) {
        s = total;
    }
    // 找到 s 所在段（段数 = 点数 - 1）
    std::size_t i = 0;
    while (i + 2 < cum.size() && cum[i + 1] <= s) ++i;
    const double segLen = cum[i + 1] - cum[i];
    const double local = s - cum[i];
    if (!(segLen > 0.0)) {  // 退化段：位置取段首，绝不产生 NaN
        out.p = route[i];
        segIdx = i;
        out.segPos = 0.0;
        out.segLen = 0.0;
        return out;
    }
    const double t = local / segLen;
    out.p = route[i] + (route[i + 1] - route[i]) * t;
    segIdx = i;
    out.segPos = local;
    out.segLen = segLen;
    return out;
}

AdvanceOutcome advanceEntity(Entity& e, double dtSec, double drainPctPerSec) {
    AdvanceOutcome out;
    if (dtSec > 0.0 && drainPctPerSec > 0.0) {
        e.battery -= drainPctPerSec * dtSec;
        if (e.battery < 0.0) e.battery = 0.0;
    }
    if (e.arrived || !(dtSec > 0.0) || !(e.speedNominal > 0.0) || e.route.size() < 2) return out;

    // **里程参数化**：位置只由"已走里程 arc"派生。
    // 这样"到点 / 拐点 / 回环"全都退化成一次标量比较或一次取模，
    // 不存在"位置与航点判定各算一遍、浮点差异导致活锁"的空间
    // （早期用"逐段推进 + 容差判定"时，位置会卡死在航点上，见 docs/实现报告.md §4）。
    double arc = e.arc + e.speedNominal * dtSec;
    if (!e.loop && arc > e.routeLenM) arc = e.routeLenM;

    std::size_t segIdx = 0;
    const PosAt at = locate(e.route, e.arcCum, e.routeLenM, arc, e.loop, segIdx);
    e.pos = at.p;
    e.arc = arc;
    e.wp = segIdx;
    e.traveledM += e.speedNominal * dtSec;

    const std::size_t segCount = (e.route.size() >= 2) ? (e.route.size() - 1) : 0;
    const bool atRouteEnd = e.loop ? false : (e.arc >= e.routeLenM);
    if (atRouteEnd) {
        e.pos = e.route.back();
        e.arrived = true;
        out.arrivedNow = true;
    }
    // 航向 = 当前正在走的那一段（含环线闭环段）
    if (segCount > 0) {
        std::size_t k = segIdx;
        Vec2 a0 = e.route[k];
        Vec2 b0 = (k + 1 < e.route.size()) ? e.route[k + 1] : e.route.front();
        if (k + 1 >= e.route.size()) {
            a0 = e.route.back();
            b0 = e.route.front();
        }
        const double h = segmentHeading(a0, b0);
        if (h >= 0.0) e.heading = h;
    }
    (void)segCount;
    return out;
}

/// 环线闭环：`loop=true` 的航路补一条"从终点回到起点"的边。
///
/// 为什么必须补：里程参数化下 `arc` 对总长取模，若不补，"终点 → 起点"这一步就是
/// **瞬移**（上一刻在终点、下一刻回到起点，位置不连续、航向也不对）。
/// 补上之后环线是真正的闭环（A→B→A→B…），位置连续、折返航向正确。
/// 注意：这会让一圈的里程变成 `2 × |AB|`（对 [A,B] 这种"往返"航路而言）；
/// 宿主若想单向循环（如 [A,B,C] 三角形），请把 route 直接写成闭合环
/// （首尾点相同，此时不会重复补点）。
void loopClose(std::vector<Vec2>& route) {
    if (route.size() < 2) return;
    const Vec2 d = route.front() - route.back();
    if (d.len() <= 1e-9) return;  // 已经是闭合环
    route.push_back(route.front());
}

/// 去掉连续重复点（零长度段会让"里程 → 位置"的插值除零；宿主配置里很容易手滑写重）。
/// **容差按米**（1e-6 m）：这里拿到的是经纬度，用 1e-9 这种"度"级阈值等于没判。
void dropDuplicatePoints(std::vector<Vec2>& route) {
    std::vector<Vec2> out;
    out.reserve(route.size());
    for (const Vec2& p : route) {
        if (!out.empty() && (p - out.back()).len() <= 1e-6) continue;
        out.push_back(p);
    }
    route.swap(out);
}

EntityPose poseOf(const Entity& e, const LocalFrame& frame) {
    EntityPose p;
    p.platformId = e.id;
    p.deviceType = e.deviceType;
    const auto wgs = frame.toWgs(e.pos);
    p.lng = wgs.first;
    p.lat = wgs.second;
    p.altM = e.altM;
    p.heading = e.heading;
    p.speedMps = e.arrived ? 0.0 : e.speedNominal;
    p.isTarget = e.isTarget;
    return p;
}

Entity makeEntity(const std::string& id, const std::string& deviceType, const std::string& kind,
                  bool isTarget, const std::string& groupKey, const std::vector<Vec2>& routeIn,
                  double altM, double speedMps, double battery, bool loop,
                  const LocalFrame& frame) {
    (void)frame;
    std::vector<Vec2> route = routeIn;
    dropDuplicatePoints(route);
    if (loop) loopClose(route);
    Entity e;
    e.id = id;
    e.deviceType = deviceType;
    e.kind = kind;
    e.isTarget = isTarget;
    e.groupKey = groupKey;
    e.route = route;
    e.wp = 0;
    e.pos = route.empty() ? Vec2() : route.front();
    e.altM = altM;
    e.speedNominal = speedMps;
    e.battery = battery;
    detail::buildArc(route, e.arcCum, e.routeLenM);
    e.arc = 0.0;
    e.seq = 0;
    e.spawned = true;
    e.visible = true;
    e.state = MotionState::Enroute;
    if (route.size() >= 2) {
        const Vec2 d = route[1] - route[0];
        const double l = d.len();
        if (l > 1e-9) e.heading = normalizeHeading(std::atan2(d.x, d.y) * 180.0 / kPi);
    }
    return e;
}

double normalizeHeading(double deg) {
    if (!std::isfinite(deg)) return 0.0;
    deg = std::fmod(deg, 360.0);
    if (deg < 0.0) deg += 360.0;
    if (deg >= 360.0) deg -= 360.0;
    return deg;
}

}  // namespace detail

std::vector<int> allowedSpeedMultipliers() { return {1, 8, 60}; }

// ============================================================================
// SimSource::Impl
// ============================================================================

struct SimSource::Impl {
    SimOptions opts;
    SimScenario scenario;
    bool initialized = false;

    detail::LocalFrame frame;
    std::vector<detail::Obstacle> hardZones;  // 原环（复核与"是否在禁飞区内"判定用）
    std::vector<detail::Poly> softZones;      // 原环
    detail::Poly taskZoneRing;                // 场景里第一个任务区（只用于目标的归属报告）
    std::vector<detail::Entity> entities;
    std::unordered_map<std::string, std::size_t> index;
    std::vector<SensorAttachment> attachments;
    PlanResult plan;

    std::shared_ptr<IClock> clock;
    std::shared_ptr<ISimSink> sink;
    std::shared_ptr<ISensorModel> sensor;

    int64_t epoch0 = 0;
    int64_t simElapsedMs = 0;
    int64_t lastTickMs = 0;
    bool hasTickBaseline = false;
    bool paused = false;
    int64_t pausedRealMs = 0;
    Metrics metrics;

    // ---------------------------------------------------------------- 查询

    detail::Entity* find(const std::string& id) {
        auto it = index.find(id);
        if (it == index.end()) return nullptr;
        return &entities[it->second];
    }

    void markZoneMembership(detail::Entity& e) const {
        e.visible = e.spawned && e.state != MotionState::Pending;
        e.inHardZone = false;
        e.inSoftZone = false;
        for (const detail::Obstacle& o : hardZones) {
            if (detail::pointInPolygon(o.ring, e.pos)) {
                e.inHardZone = true;
                break;
            }
        }
        for (const detail::Poly& p : softZones) {
            if (detail::pointInPolygon(p, e.pos)) {
                e.inSoftZone = true;
                break;
            }
        }
        e.inTaskArea = false;
        if (!e.taskRing.empty()) {
            e.inTaskArea = detail::pointInPolygon(e.taskRing, e.pos);
        } else if (e.isTarget && !taskZoneRing.empty()) {
            // 目标是"外部实体"，没有 taskAreaKey；用**场景里第一个任务区**报告它的归属。
            // 这只是给宿主的可读事实（"这个目标此刻在任务区里"），不参与任何判定。
            e.inTaskArea = detail::pointInPolygon(taskZoneRing, e.pos);
        }
    }

    // ---------------------------------------------------------------- 事件

    SimEvent buildEvent(const detail::Entity& e, int64_t ts) const {
        SimEvent ev;
        ev.deviceId = e.id;
        ev.deviceType = e.deviceType;
        ev.kind = e.kind;
        ev.ts = ts;
        const auto wgs = frame.toWgs(e.pos);
        ev.lng = wgs.first;
        ev.lat = wgs.second;
        ev.alt = e.altM;
        ev.heading = e.heading;
        ev.speed = e.arrived ? 0.0 : e.speedNominal;
        const long rounded = std::lround(e.battery);
        ev.battery = static_cast<int>(std::max<long>(0, std::min<long>(100, rounded)));
        ev.seq = e.seq;
        ev.presence = SimEvent::PresAll;
        return ev;
    }

    void emit(detail::Entity& e, int64_t ts) {
        if (sink) {
            const SimEvent ev = buildEvent(e, ts);
            try {
                sink->onEvent(ev);
            } catch (...) {
                ++metrics.sinkErrors;  // 宿主回调出问题 MUST NOT 影响仿真推进
            }
        }
        ++metrics.eventsEmitted;
        ++e.seq;
    }

    // ---------------------------------------------------------------- 传感器挂接

    void runSensors(int64_t ts) {
        if (!sensor) return;
        for (detail::Entity& e : entities) {
            if (e.isTarget || !e.hasSensor || !e.spawned) continue;
            if (e.sensorIntervalMs > 0 && e.lastSenseMs > 0 &&
                (ts - e.lastSenseMs) < e.sensorIntervalMs) {
                continue;
            }
            e.lastSenseMs = ts;

            SensorPose pose;
            pose.sensorId = e.sensorId;
            pose.deviceId = e.id;
            pose.deviceType = e.deviceType;
            pose.rangeM = e.sensorRangeM;
            pose.ts = ts;
            pose.self = detail::poseOf(e, frame);
            for (const detail::Entity& c : entities) {
                if (c.id == e.id) continue;  // 不自我探测
                if (!c.spawned) continue;    // 未出现的 popup 不交出去
                pose.candidates.push_back(detail::poseOf(c, frame));
            }

            std::vector<SimObservation> obs;
            ++metrics.sensorCalls;
            try {
                obs = sensor->sense(pose);
            } catch (...) {
                ++metrics.sensorErrors;
                continue;
            }
            for (const SimObservation& o : obs) {
                if (sink) {
                    try {
                        sink->onObservation(o);
                    } catch (...) {
                        ++metrics.sinkErrors;
                    }
                }
                ++metrics.observationsEmitted;
            }
        }
    }

    // ---------------------------------------------------------------- 推进

    int64_t simNow() const { return epoch0 + simElapsedMs; }

    /// 推进一个子步（**不产出事件**）。`simElapsedMs` 的累加在这里发生，
    /// 这样 popup 的"到点出现"与事件 ts 都由**同一个**仿真时间驱动。
    void advanceSubstep(int64_t dtMs) {
        if (!(dtMs > 0)) return;
        simElapsedMs += dtMs;
        const double dtSec = static_cast<double>(dtMs) / 1000.0;
        for (detail::Entity& e : entities) {
            if (!e.spawned) {
                if (simElapsedMs >= e.popupOffsetMs) e.spawned = true;
                if (!e.spawned) {
                    e.state = MotionState::Pending;
                    continue;
                }
                ++metrics.popupSpawned;
            }
            if (e.arrived) {
                e.state = MotionState::Arrived;
                continue;
            }
            e.state = MotionState::Enroute;
            const detail::AdvanceOutcome oc =
                detail::advanceEntity(e, dtSec, opts.batteryDrainPctPerSec);
            if (oc.arrivedNow) {
                ++metrics.arrivals;
                e.state = MotionState::Arrived;
            }
        }
        ++metrics.substeps;
    }

    /// 按仿真毫秒推进（自动切子步），末尾产出本节拍的事件
    int advanceSimMs(int64_t simMs) {
        if (simMs <= 0) return 0;
        const int64_t sub = (opts.substepMs > 0) ? opts.substepMs : 100;
        int64_t left = simMs;
        while (left > 0) {
            const int64_t d = (left < sub) ? left : sub;
            advanceSubstep(d);
            left -= d;
        }
        if (paused) {
            for (detail::Entity& e : entities) {
                if (e.spawned && e.state != MotionState::Arrived) e.state = MotionState::Holding;
            }
        }
        int emitted = 0;
        const int64_t ts = simNow();
        metrics.simElapsedMs = simElapsedMs;  // 观测口径：仿真已推进的总时长
        for (detail::Entity& e : entities) {
            markZoneMembership(e);
            if (!e.visible) continue;  // popup 偏移未到 → 不出现、不发事件
            emit(e, ts);
            ++emitted;
        }
        runSensors(ts);
        return emitted;
    }

    void seedEntities() {
        entities.clear();
        index.clear();
        metrics.arrivals = 0;
        metrics.popupSpawned = 0;

        // 场景里第一个任务区（给目标做归属报告；平台用各自的 taskAreaKey）
        taskZoneRing.clear();
        for (const Area& a : scenario.areas) {
            if (a.role != AreaRole::Task) continue;
            for (const auto& pt : a.polygon) taskZoneRing.push_back(frame.toLocal(pt.first, pt.second));
            break;
        }

        // 平台：按 deviceId 字典序（与规划同序 → 事件顺序可复现）
        std::vector<const Platform*> plats;
        plats.reserve(scenario.platforms.size());
        for (const Platform& p : scenario.platforms) plats.push_back(&p);
        std::sort(plats.begin(), plats.end(),
                  [](const Platform* a, const Platform* b) { return a->deviceId < b->deviceId; });

        for (const Platform* p : plats) {
            std::vector<detail::Vec2> route;
            const std::vector<std::pair<double, double>>* wgs = plan.routeOf(p->deviceId);
            if (wgs != nullptr) {
                for (const auto& pt : *wgs) route.push_back(frame.toLocal(pt.first, pt.second));
            }
            if (route.empty()) {
                detail::Vec2 c;
                if (detail::areaCentroidLocal(scenario, frame, p->homeAreaKey, c)) {
                    route.push_back(c);
                } else {
                    route.push_back(detail::Vec2());
                }
            }
            const double spd = (p->speedMps > 0.0) ? p->speedMps : opts.defaultSpeedMps;
            const std::string kind = p->kind.empty() ? opts.defaultKind : p->kind;
            detail::Entity e = detail::makeEntity(p->deviceId, p->deviceType, kind, false,
                                                  p->groupKey, route, p->altM, spd, p->battery,
                                                  false, frame);
            // 任务区环（用于"到了没到"的判定）
            const Area* task = scenario.findArea(p->taskAreaKey);
            if (task != nullptr) {
                for (const auto& pt : task->polygon) {
                    e.taskRing.push_back(frame.toLocal(pt.first, pt.second));
                }
            }
            index[e.id] = entities.size();
            entities.push_back(e);
        }

        // 目标：按 id 字典序
        std::vector<const Target*> targets;
        targets.reserve(scenario.targets.size());
        for (const Target& t : scenario.targets) targets.push_back(&t);
        std::sort(targets.begin(), targets.end(),
                  [](const Target* a, const Target* b) { return a->id < b->id; });

        for (const Target* t : targets) {
            std::vector<detail::Vec2> route;
            for (const auto& pt : t->route) route.push_back(frame.toLocal(pt.first, pt.second));
            if (route.empty()) {
                route.push_back(t->hasPosition ? frame.toLocal(t->position.first, t->position.second)
                                               : detail::Vec2());
            }
            const std::string dt =
                t->deviceType.empty() ? opts.defaultTargetDeviceType : t->deviceType;
            const std::string kind = t->kind.empty() ? opts.defaultKind : t->kind;
            const bool isPopup = (t->motion == TargetMotion::Popup);
            detail::Entity e = detail::makeEntity(t->id, dt, kind, true, std::string(), route, 0.0,
                                                  t->speedMps, 100.0, t->loop, frame);
            e.loop = t->loop;
            e.confidence = t->confidence;
            e.targetNo = t->no;
            e.typeKey = t->typeKey;
            e.isPopup = isPopup;
            e.popupOffsetMs = t->startOffsetMs;
            e.spawnAtMs = epoch0 + t->startOffsetMs;
            const bool moves = (t->motion == TargetMotion::Dynamic || isPopup) &&
                               (t->speedMps > 0.0) && (route.size() >= 2);
            e.arrived = !moves;  // static（或给了 speed 却没有航路）→ 就地不动
            if (e.isPopup) {
                e.spawned = (simElapsedMs >= e.popupOffsetMs);
                e.visible = e.spawned;
                e.state = e.spawned ? MotionState::Enroute : MotionState::Pending;
            } else {
                e.spawned = true;
                e.visible = true;
                e.state = MotionState::Enroute;
            }
            index[e.id] = entities.size();
            entities.push_back(e);
        }

        // 出生即算一次区域归属：宿主 init 之后立刻问实体状态，也要拿到正确的
        // inTaskArea / inHardZone / visible（否则要等到第一次 step 才有值）
        for (detail::Entity& e : entities) markZoneMembership(e);
    }

    void applyAttachments() {
        for (detail::Entity& e : entities) {
            e.hasSensor = false;
            e.sensorId.clear();
            e.sensorRangeM = 0.0;
            e.sensorIntervalMs = 0;
            e.lastSenseMs = 0;
        }
        for (const SensorAttachment& a : attachments) {
            detail::Entity* e = find(a.platformId);
            if (e == nullptr) continue;
            e->hasSensor = true;
            e->sensorId = a.sensorId.empty() ? a.platformId : a.sensorId;
            e->sensorRangeM = a.rangeM;
            e->sensorIntervalMs = a.intervalMs;
        }
    }

    void resetClock() {
        // 仿真只由场景 + 调用序列决定（SIM-NFR-01）：
        //   `step()` 驱动时 epoch0 = 0（事件 ts = 相对毫秒，绝对可复现）；
        //   首次 `tick(nowMs)` 驱动时 epoch0 取那次 nowMs（事件 ts = 真实 epoch 毫秒）。
        epoch0 = 0;
        simElapsedMs = 0;
        lastTickMs = 0;
        hasTickBaseline = false;
        paused = false;
        pausedRealMs = 0;
        metrics = Metrics();
    }

    // ---------------------------------------------------------------- 校验

    /// 场景校验（**纯校验**：只读，不修改任何状态）
    ValidationResult validateScenario() const {
        ValidationResult r;
        auto issue = [&r](const std::string& path, const std::string& field,
                          const std::string& reason) {
            ScenarioIssue i;
            i.path = path;
            i.field = field;
            i.reason = reason;
            r.issues.push_back(i);
            r.ok = false;
        };
        auto coordOk = [](double lng, double lat) {
            return std::isfinite(lng) && std::isfinite(lat) && lng >= -180.0 && lng <= 180.0 &&
                   lat >= -90.0 && lat <= 90.0;
        };

        std::vector<std::string> areaKeys;
        for (std::size_t i = 0; i < scenario.areas.size(); ++i) {
            const Area& a = scenario.areas[i];
            const std::string path = "areas[" + std::to_string(i) + "]";
            if (a.key.empty()) issue(path, "key", "区域 key 为空");
            if (std::find(areaKeys.begin(), areaKeys.end(), a.key) != areaKeys.end()) {
                issue(path, "key", "区域 key 重复：" + a.key);
            }
            areaKeys.push_back(a.key);
            if (a.polygon.size() < 3) {
                issue(path, "polygon",
                      "多边形至少需要 3 个顶点（实际 " + std::to_string(a.polygon.size()) + "）");
                continue;
            }
            for (std::size_t k = 0; k < a.polygon.size(); ++k) {
                if (!coordOk(a.polygon[k].first, a.polygon[k].second)) {
                    issue(path + ".polygon[" + std::to_string(k) + "]", "lng/lat",
                          "坐标非有限值或越界");
                }
            }
            detail::Poly ring;
            for (const auto& pt : a.polygon) ring.push_back(detail::Vec2(pt.first, pt.second));
            if (!detail::validRing(ring)) issue(path, "polygon", "多边形退化（面积为 0）");
        }

        std::vector<std::string> groupKeys;
        for (std::size_t i = 0; i < scenario.groups.size(); ++i) {
            const Group& g = scenario.groups[i];
            const std::string path = "groups[" + std::to_string(i) + "]";
            if (g.key.empty()) issue(path, "key", "编组 key 为空");
            groupKeys.push_back(g.key);
        }

        auto areaWithRole = [&](const std::string& key, AreaRole role) -> const Area* {
            const Area* a = scenario.findArea(key);
            if (a == nullptr) return nullptr;
            return (a->role == role) ? a : nullptr;
        };

        std::vector<std::string> ids;
        for (std::size_t i = 0; i < scenario.platforms.size(); ++i) {
            const Platform& p = scenario.platforms[i];
            const std::string path = "platforms[" + std::to_string(i) + "]";
            if (p.deviceId.empty()) {
                issue(path, "deviceId", "deviceId 为空");
            } else if (std::find(ids.begin(), ids.end(), p.deviceId) != ids.end()) {
                issue(path, "deviceId", "deviceId 重复：" + p.deviceId);
            }
            ids.push_back(p.deviceId);
            if (p.deviceType.empty()) issue(path, "deviceType", "deviceType 为空");
            if (areaWithRole(p.homeAreaKey, AreaRole::Deploy) == nullptr) {
                issue(path, "homeAreaKey", "不是有效的部署区（role=deploy）：" + p.homeAreaKey);
            }
            if (areaWithRole(p.taskAreaKey, AreaRole::Task) == nullptr) {
                issue(path, "taskAreaKey", "不是有效的任务区（role=task）：" + p.taskAreaKey);
            }
            if (!p.groupKey.empty() &&
                std::find(groupKeys.begin(), groupKeys.end(), p.groupKey) == groupKeys.end()) {
                issue(path, "groupKey", "引用了未声明的编组：" + p.groupKey);
            }
            if (!std::isfinite(p.altM)) issue(path, "altM", "高度非有限值");
            if (!std::isfinite(p.speedMps) || p.speedMps < 0.0) {
                issue(path, "speedMps", "速度必须 >= 0");
            }
            if (!std::isfinite(p.battery) || p.battery < 0.0 || p.battery > 100.0) {
                issue(path, "battery", "电量必须在 [0, 100]");
            }
        }

        std::vector<std::string> tids;
        for (std::size_t i = 0; i < scenario.targets.size(); ++i) {
            const Target& t = scenario.targets[i];
            const std::string path = "targets[" + std::to_string(i) + "]";
            if (t.id.empty()) issue(path, "id", "目标 id 为空（请显式给 id 或 no）");
            if (std::find(tids.begin(), tids.end(), t.id) != tids.end()) {
                issue(path, "id", "目标 id 重复：" + t.id);
            }
            tids.push_back(t.id);
            if (t.motion == TargetMotion::Dynamic || t.motion == TargetMotion::Popup) {
                if (t.route.size() < 2) {
                    issue(path, "route", "dynamic/popup 目标至少需要 2 个航路点（实际 " +
                                             std::to_string(t.route.size()) + "）");
                }
                if (!(t.speedMps > 0.0)) issue(path, "speedMps", "dynamic/popup 目标速度必须 > 0");
            }
            for (std::size_t k = 0; k < t.route.size(); ++k) {
                if (!coordOk(t.route[k].first, t.route[k].second)) {
                    issue(path + ".route[" + std::to_string(k) + "]", "lng/lat",
                          "坐标非有限值或越界");
                }
            }
            if (t.hasPosition && !coordOk(t.position.first, t.position.second)) {
                issue(path, "position", "坐标非有限值或越界");
            }
            if (t.startOffsetMs < 0) issue(path, "startOffsetMs", "偏移必须 >= 0");
            if (!std::isfinite(t.confidence) || t.confidence < 0.0 || t.confidence > 1.0) {
                issue(path, "confidence", "置信度必须在 [0, 1]");
            }
        }

        // 部署区质心落在 hard 禁飞区内 → 规划必然无解，属输入问题（提前报，别等运行期）
        std::vector<detail::Obstacle> hard;
        std::vector<detail::Poly> soft;
        detail::collectZones(scenario, frame, hard, soft);
        for (std::size_t i = 0; i < scenario.platforms.size(); ++i) {
            const Platform& p = scenario.platforms[i];
            detail::Vec2 c;
            if (!detail::areaCentroidLocal(scenario, frame, p.homeAreaKey, c)) continue;
            for (const detail::Obstacle& o : hard) {
                if (detail::pointInPolygon(o.ring, c)) {
                    issue("platforms[" + std::to_string(i) + "]", "homeAreaKey",
                          "部署区质心落在 hard 禁飞区 " + o.key + " 内，规划无解");
                    break;
                }
            }
        }
        return r;
    }
};

// ============================================================================
// SimSource 生命周期与公开面
// ============================================================================

SimSource::SimSource() : impl_(new Impl()) {}
SimSource::SimSource(const SimOptions& opts) : impl_(new Impl()) { impl_->opts = opts; }
SimSource::~SimSource() = default;
SimSource::SimSource(SimSource&&) noexcept = default;
SimSource& SimSource::operator=(SimSource&&) noexcept = default;

ValidationResult SimSource::validate(const SimScenario& scenario, const SimOptions& opts) {
    Impl tmp;
    tmp.opts = opts;
    tmp.scenario = scenario;
    tmp.frame = detail::frameFor(scenario);
    return tmp.validateScenario();
}

ValidationResult SimSource::init(const SimScenario& scenario) {
    // 全部在**暂存区**完成：失败时实例保持上一次成功的局面（与 resource-alloc 的原子替换同口径）
    Impl staging;
    staging.opts = impl_->opts;
    staging.scenario = scenario;
    staging.frame = detail::frameFor(scenario);
    const ValidationResult vr = staging.validateScenario();
    if (!vr.ok) return vr;

    // 规划与运行期实体**必须**共用同一个局部平面（frameFor 的参考点只依赖场景内容）。
    // stand-alone 的 planRoutes() 内部用同一个 frameFor(scenario)，因此两者逐字节可比对。
    staging.plan = SimSource::planRoutesInFrame(scenario, impl_->opts, staging.frame);
    if (!staging.plan.ok) {
        ValidationResult bad;
        bad.ok = false;
        ScenarioIssue i;
        i.path = "plan";
        i.field = "routes";
        i.reason = staging.plan.message;
        bad.issues.push_back(i);
        return bad;
    }

    detail::collectZones(scenario, staging.frame, staging.hardZones, staging.softZones);
    staging.seedEntities();

    impl_->scenario = scenario;
    impl_->frame = staging.frame;
    impl_->plan = staging.plan;
    impl_->hardZones = staging.hardZones;
    impl_->softZones = staging.softZones;
    impl_->taskZoneRing = staging.taskZoneRing;  // 暂存区算出来的任务区环必须一并接管
    impl_->entities = staging.entities;
    impl_->index = staging.index;
    impl_->resetClock();
    impl_->initialized = true;
    impl_->applyAttachments();
    return vr;
}

SimScenario SimSource::scenario() const { return impl_->scenario; }
const PlanResult& SimSource::plan() const { return impl_->plan; }

// ---------------------------------------------------------------- 节拍

bool SimSource::setSpeed(int multiplier) {
    const std::vector<int> allowed = allowedSpeedMultipliers();
    if (std::find(allowed.begin(), allowed.end(), multiplier) == allowed.end()) {
        return false;  // 只接受 1 / 8 / 60（README「节拍控制：实时 / 倍速（1× 8× 60×）」）
    }
    impl_->opts.speedMultiplier = multiplier;
    return true;
}

int SimSource::speedMultiplier() const { return impl_->opts.speedMultiplier; }

void SimSource::pause() {
    if (impl_->paused) return;
    impl_->paused = true;
    for (detail::Entity& e : impl_->entities) {
        if (e.spawned && e.state != MotionState::Arrived) e.state = MotionState::Holding;
    }
}

void SimSource::resume() {
    impl_->paused = false;
    // 从此刻重新计时：暂停期间的真实时间 MUST NOT 变成一次仿真跳跃
    impl_->hasTickBaseline = false;
    for (detail::Entity& e : impl_->entities) {
        if (e.spawned && e.state == MotionState::Holding) e.state = MotionState::Enroute;
    }
}

bool SimSource::paused() const { return impl_->paused; }

int SimSource::step(int64_t dtMs) {
    ++impl_->metrics.steps;
    if (impl_->paused) return 0;  // 暂停 = 单步也不推进（要单步请先 resume）
    if (dtMs <= 0) return 0;
    return impl_->advanceSimMs(dtMs);
}

int SimSource::tick(int64_t nowMs) {
    ++impl_->metrics.ticks;
    if (impl_->paused) {
        if (impl_->hasTickBaseline) {
            const int64_t d = nowMs - impl_->lastTickMs;
            if (d > 0) {
                impl_->pausedRealMs += d;     // 暂停期间的真实时间被**丢弃**（MUST NOT 补偿）
                impl_->metrics.pausedMs += d; // 观测口径：这段时间被丢了多少
            }
        }
        impl_->lastTickMs = nowMs;
        impl_->hasTickBaseline = true;
        return 0;
    }
    if (!impl_->hasTickBaseline) {
        // 首次 tick：把**真实时间**定为仿真纪元起点。这样事件 ts 是真实 epoch 毫秒
        // （对齐 device-ingest 契约的"epoch 毫秒"口径），而不是"从零开始的偏移"。
        impl_->epoch0 = nowMs;
        impl_->lastTickMs = nowMs;
        impl_->hasTickBaseline = true;
        return 0;  // 首次只记基线（没有"上一次"，无法算差值）
    }
    const int64_t realMs = nowMs - impl_->lastTickMs;
    impl_->lastTickMs = nowMs;
    if (realMs <= 0) return 0;

    // 倍速在这里生效：仿真时间(ms) = 真实时间(ms) × 倍率。单位自始至终是**毫秒**：
    // nowMs 是 epoch 毫秒、倍率是纯倍数，因此不存在任何"秒 → 毫秒"的换算
    // （早期版本多写了一次 /1000，表现为"8× 推进 1 s 只走了 8 ms"，见 docs/实现报告.md §4）。
    const int64_t simMs = realMs * impl_->opts.speedMultiplier;
    if (simMs <= 0) return 0;
    return impl_->advanceSimMs(simMs);
}

int SimSource::tick() {
    // 缺时间参数：注入时钟存在 → 用它；否则按 SimOptions 口径（默认不推进，保可复现）
    if (impl_->clock) return tick(impl_->clock->nowMs());
    if (impl_->opts.useClockWhenTickArgMissing) return 0;
    ++impl_->metrics.ticks;
    return 0;
}

// ---------------------------------------------------------------- 注入

void SimSource::setClock(std::shared_ptr<IClock> clock) { impl_->clock = std::move(clock); }

void SimSource::setSink(std::shared_ptr<ISimSink> sink) { impl_->sink = std::move(sink); }

void SimSource::setSensorModel(std::shared_ptr<ISensorModel> model) {
    impl_->sensor = std::move(model);
    impl_->applyAttachments();
}

void SimSource::setSensorAttachments(const std::vector<SensorAttachment>& attachments) {
    impl_->attachments = attachments;
    impl_->applyAttachments();
}

// ---------------------------------------------------------------- 观测

int64_t SimSource::simElapsedMs() const { return impl_->simElapsedMs; }
int64_t SimSource::simNowMs() const { return impl_->simNow(); }

std::vector<EntityStatus> SimSource::entities() const {
    std::vector<EntityStatus> out;
    out.reserve(impl_->entities.size());
    for (const detail::Entity& e : impl_->entities) {
        detail::Entity tmp = e;
        impl_->markZoneMembership(tmp);
        EntityStatus s;
        s.id = tmp.id;
        s.deviceType = tmp.deviceType;
        s.kind = tmp.kind;
        s.isTarget = tmp.isTarget;
        s.groupKey = tmp.groupKey;
        s.state = tmp.arrived ? MotionState::Arrived : tmp.state;
        const auto wgs = impl_->frame.toWgs(tmp.pos);
        s.lng = wgs.first;
        s.lat = wgs.second;
        s.altM = tmp.altM;
        s.heading = tmp.heading;
        s.speedMps = tmp.arrived ? 0.0 : tmp.speedNominal;
        s.battery = tmp.battery;
        s.seq = tmp.seq;
        s.waypointIndex = static_cast<int>(tmp.wp);
        s.visible = tmp.visible;
        s.inTaskArea = tmp.inTaskArea;
        s.inHardZone = tmp.inHardZone;
        s.inSoftZone = tmp.inSoftZone;
        out.push_back(s);
    }
    return out;
}

bool SimSource::entity(const std::string& id, EntityStatus& out) const {
    for (const EntityStatus& s : entities()) {
        if (s.id == id) {
            out = s;
            return true;
        }
    }
    return false;
}

std::vector<std::string> SimSource::deviceIds() const {
    std::vector<std::string> out;
    out.reserve(impl_->entities.size());
    for (const detail::Entity& e : impl_->entities) out.push_back(e.id);
    return out;
}

Metrics SimSource::metrics() const { return impl_->metrics; }

Capabilities SimSource::capabilities() const {
    Capabilities c;
    c.initialized = impl_->initialized;
    c.clockInjected = (impl_->clock != nullptr);
    c.sinkInjected = (impl_->sink != nullptr);
    c.customSensorInjected = (impl_->sensor != nullptr);
    c.platforms = static_cast<int>(impl_->scenario.platforms.size());
    c.targets = static_cast<int>(impl_->scenario.targets.size());
    c.groups = static_cast<int>(impl_->scenario.groups.size());
    c.areas = static_cast<int>(impl_->scenario.areas.size());
    for (const Area& a : impl_->scenario.areas) {
        if (a.role == AreaRole::NoFly && !(a.hasHardness && a.hardness == Hardness::Soft)) {
            ++c.hardZones;
        }
    }
    for (const detail::Entity& e : impl_->entities) {
        if (e.hasSensor) ++c.sensors;
    }
    c.speedMultiplier = impl_->opts.speedMultiplier;
    c.paused = impl_->paused;
    c.simElapsedMs = impl_->simElapsedMs;
    c.scenarioKey = impl_->scenario.scenarioKey;
    return c;
}

json SimSource::entitiesJson() const {
    json arr = json::array();
    for (const EntityStatus& s : entities()) arr.push_back(sim_source::toJson(s));
    return arr;
}

}  // namespace sim_source
