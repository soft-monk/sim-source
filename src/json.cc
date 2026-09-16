// src/json.cc · JSON 边界（归一化对象 / 中立结构 / 规划与校验结果）
//
// 两条纪律：
//   ① **主键序 = 《外设接入契约》§2 的键序**（deviceId/deviceType/kind/lng/lat/alt/heading/
//      speed/battery/seq/ts）。这是可机检的契约，验收脚本会逐字段比对；用 ordered_json
//      就是为了让插入顺序 = 输出顺序。
//   ② **解析不抛异常**：结构/取值问题一律变成 ok=false + 可读原因（对齐 device-ingest 的
//      降级约定与 resource-alloc 的 P10），宿主拿到的是结论而不是异常。
#include "internal.h"

#include <algorithm>
#include <cmath>

namespace sim_source {

// ============================================================================
// §1 归一化对象（出口形状）
// ============================================================================

const std::vector<std::string>& normalizedFieldOrder() {
    static const std::vector<std::string> kOrder = {
        "deviceId", "deviceType", "kind", "lng",  "lat",     "alt",
        "heading",  "speed",      "battery", "seq", "ts"};
    return kOrder;
}

json SimEvent::toJson() const {
    json j = json::object();
    j["deviceId"] = deviceId;
    j["deviceType"] = deviceType;
    j["kind"] = kind;
    j["lng"] = detail::numOrNull(lng);
    j["lat"] = detail::numOrNull(lat);
    if (hasAlt()) j["alt"] = detail::numOrNull(alt);
    if (hasHeading()) j["heading"] = detail::numOrNull(heading);
    if (hasSpeed()) j["speed"] = detail::numOrNull(speed);
    if (hasBattery()) j["battery"] = battery;
    if (hasSeq()) j["seq"] = seq;
    j["ts"] = ts;
    // 宿主扩展位：**按 key 排序**后追加。ordered_json 保留插入顺序，
    // 若直接遍历源对象，输出顺序会依赖宿主构造顺序 → 双跑可能不一致。
    if (extensions.is_object() && !extensions.empty()) {
        std::vector<std::string> keys;
        keys.reserve(extensions.size());
        for (auto it = extensions.begin(); it != extensions.end(); ++it) keys.push_back(it.key());
        std::sort(keys.begin(), keys.end());
        for (const std::string& k : keys) j[k] = extensions[k];
    }
    return j;
}

json toJson(const SimEvent& e) { return e.toJson(); }

EventParseResult parseSimEvent(const json& j) {
    EventParseResult r;
    if (!j.is_object()) {
        r.reason = "不是 JSON 对象";
        return r;
    }
    auto needStr = [&](const char* key, std::string& out) {
        auto it = j.find(key);
        if (it == j.end()) {
            r.reason = std::string("缺字段 ") + key;
            return false;
        }
        if (it->is_string()) {
            out = it->get<std::string>();
            return true;
        }
        if (it->is_number_integer()) {
            out = std::to_string(it->get<long long>());
            return true;
        }
        r.reason = std::string("字段 ") + key + " 类型不是字符串";
        return false;
    };
    if (!needStr("deviceId", r.event.deviceId)) return r;
    if (!needStr("deviceType", r.event.deviceType)) return r;
    if (!needStr("kind", r.event.kind)) return r;
    if (r.event.deviceId.empty()) {
        r.reason = "deviceId 为空";
        return r;
    }
    auto itTs = j.find("ts");
    if (itTs == j.end() || !itTs->is_number()) {
        r.reason = "缺字段 ts（或类型不是数字）";
        return r;
    }
    r.event.ts = itTs->get<int64_t>();

    unsigned presence = 0;
    // 可选数值字段：缺失/为 null → 视为"该字段不存在"（契约允许整字段缺失）
    auto optNum = [&](const char* key, double& out, unsigned bit) {
        auto it = j.find(key);
        if (it == j.end() || it->is_null()) return;
        if (!it->is_number()) {
            r.reason = std::string("字段 ") + key + " 类型不是数字";
            return;
        }
        out = it->get<double>();
        presence |= bit;
    };
    // 坐标是 MUST 字段，但契约允许"非法/越界则整字段缺失"
    if (j.find("lng") == j.end() || j.find("lat") == j.end() || j["lng"].is_null() ||
        j["lat"].is_null()) {
        r.reason = "缺坐标字段 lng/lat（契约：坐标越界或非法时整字段缺失）";
        return r;
    }
    if (!j["lng"].is_number() || !j["lat"].is_number()) {
        r.reason = "坐标字段 lng/lat 类型不是数字";
        return r;
    }
    r.event.lng = j["lng"].get<double>();
    r.event.lat = j["lat"].get<double>();
    if (!std::isfinite(r.event.lng) || !std::isfinite(r.event.lat) ||
        r.event.lng < -180.0 || r.event.lng > 180.0 || r.event.lat < -90.0 ||
        r.event.lat > 90.0) {
        r.reason = "坐标越界（lng ∈ [-180,180]，lat ∈ [-90,90]）";
        return r;
    }
    optNum("alt", r.event.alt, SimEvent::PresAlt);
    if (!r.reason.empty()) return r;
    auto itHeading = j.find("heading");
    if (itHeading != j.end() && !itHeading->is_null()) {
        if (!itHeading->is_number()) {
            r.reason = "字段 heading 类型不是数字";
            return r;
        }
        r.event.heading = detail::normalizeHeading(itHeading->get<double>());
        presence |= SimEvent::PresHeading;
    }
    optNum("speed", r.event.speed, SimEvent::PresSpeed);
    if (!r.reason.empty()) return r;
    auto itBattery = j.find("battery");
    if (itBattery != j.end() && !itBattery->is_null()) {
        if (!itBattery->is_number_integer()) {
            r.reason = "字段 battery 类型不是整数";
            return r;
        }
        const long long b = itBattery->get<long long>();
        if (b < 0 || b > 100) {
            r.reason = "字段 battery 越界（必须在 0–100）";
            return r;
        }
        r.event.battery = static_cast<int>(b);
        presence |= SimEvent::PresBattery;
    }
    auto itSeq = j.find("seq");
    if (itSeq != j.end() && !itSeq->is_null()) {
        if (!itSeq->is_number_integer()) {
            r.reason = "字段 seq 类型不是整数";
            return r;
        }
        r.event.seq = itSeq->get<int64_t>();
        presence |= SimEvent::PresSeq;
    }
    r.event.presence = presence;

    // 契约之外的键（tsSource/recvAt/source/...）原样收进扩展位，不报错（未知字段忽略）
    for (auto it = j.begin(); it != j.end(); ++it) {
        const std::string& k = it.key();
        if (std::find(normalizedFieldOrder().begin(), normalizedFieldOrder().end(), k) !=
            normalizedFieldOrder().end()) {
            continue;
        }
        r.event.extensions[k] = it.value();
    }
    r.ok = true;
    r.reason.clear();
    return r;
}

// ============================================================================
// §2 枚举字符串（取值冻结）
// ============================================================================

const char* toString(AreaRole r) {
    switch (r) {
        case AreaRole::Deploy: return "deploy";
        case AreaRole::Task: return "task";
        case AreaRole::NoFly: return "nofly";
    }
    return "";
}

const char* toString(Hardness h) {
    switch (h) {
        case Hardness::Hard: return "hard";
        case Hardness::Soft: return "soft";
    }
    return "";
}

const char* toString(TargetMotion m) {
    switch (m) {
        case TargetMotion::Static: return "static";
        case TargetMotion::Dynamic: return "dynamic";
        case TargetMotion::Popup: return "popup";
    }
    return "";
}

const char* toString(MotionState s) {
    switch (s) {
        case MotionState::Enroute: return "enroute";
        case MotionState::Arrived: return "arrived";
        case MotionState::Holding: return "holding";
        case MotionState::Pending: return "pending";
    }
    return "";
}

std::optional<AreaRole> areaRoleFromString(const std::string& s) {
    if (s == "deploy") return AreaRole::Deploy;
    if (s == "task") return AreaRole::Task;
    if (s == "nofly" || s == "no-fly" || s == "nfz") return AreaRole::NoFly;
    return std::nullopt;
}

std::optional<Hardness> hardnessFromString(const std::string& s) {
    if (s == "hard") return Hardness::Hard;
    if (s == "soft") return Hardness::Soft;
    return std::nullopt;
}

std::optional<TargetMotion> targetMotionFromString(const std::string& s) {
    if (s == "static") return TargetMotion::Static;
    if (s == "dynamic") return TargetMotion::Dynamic;
    if (s == "popup") return TargetMotion::Popup;
    return std::nullopt;
}

// ============================================================================
// §3 中立结构
// ============================================================================

namespace {

json polygonJson(const std::vector<std::pair<double, double>>& poly) {
    json arr = json::array();
    for (const auto& pt : poly) {
        json pair = json::array();
        pair.push_back(detail::numOrNull(pt.first));
        pair.push_back(detail::numOrNull(pt.second));
        arr.push_back(pair);
    }
    return arr;
}

std::vector<std::pair<double, double>> polygonFromJson(const json& j) {
    std::vector<std::pair<double, double>> out;
    if (!j.is_array()) return out;
    for (const json& item : j) {
        if (item.is_array() && item.size() >= 2 && item[0].is_number() && item[1].is_number()) {
            out.emplace_back(item[0].get<double>(), item[1].get<double>());
        }
    }
    return out;
}

}  // namespace

double Area::centroidLng() const {
    if (polygon.empty()) return 0.0;
    double s = 0.0;
    for (const auto& pt : polygon) s += pt.first;
    return s / static_cast<double>(polygon.size());
}

double Area::centroidLat() const {
    if (polygon.empty()) return 0.0;
    double s = 0.0;
    for (const auto& pt : polygon) s += pt.second;
    return s / static_cast<double>(polygon.size());
}

const Area* SimScenario::findArea(const std::string& key) const {
    for (const Area& a : areas) {
        if (a.key == key) return &a;
    }
    return nullptr;
}

json toJson(const Area& v) {
    json j = json::object();
    j["key"] = v.key;
    j["name"] = v.name;
    j["role"] = toString(v.role);
    j["polygon"] = polygonJson(v.polygon);
    if (v.hasHardness) j["hardness"] = toString(v.hardness);
    return j;
}

json toJson(const Group& v) {
    json j = json::object();
    j["key"] = v.key;
    j["name"] = v.name;
    j["role"] = v.role;
    return j;
}

json toJson(const FormationSlot& v) {
    json j = json::object();
    j["rightM"] = v.rightM;
    j["fwdM"] = v.fwdM;
    return j;
}

json toJson(const Platform& v) {
    json j = json::object();
    j["deviceId"] = v.deviceId;
    j["deviceType"] = v.deviceType;
    j["kind"] = v.kind;  // 空串也输出：往返形状稳定（fromJson 对空串回落到 defaultKind）
    j["groupKey"] = v.groupKey;
    j["homeAreaKey"] = v.homeAreaKey;
    j["taskAreaKey"] = v.taskAreaKey;
    j["altM"] = v.altM;
    j["speedMps"] = v.speedMps;
    j["battery"] = v.battery;
    j["startOffset"] = toJson(v.startOffset);
    return j;
}

json toJson(const Target& v) {
    json j = json::object();
    j["no"] = v.no;
    j["id"] = v.id;
    j["typeKey"] = v.typeKey;
    j["deviceType"] = v.deviceType;  // 往返必须带上：缺了它 fromJson 会回落默认值
    j["kind"] = v.kind;
    j["name"] = v.name;
    j["motion"] = toString(v.motion);
    if (v.hasPosition) {
        json p = json::array();
        p.push_back(v.position.first);
        p.push_back(v.position.second);
        j["position"] = p;
    }
    j["route"] = polygonJson(v.route);  // 与 polygon 同形状（[lng,lat] 数组）
    j["speedMps"] = v.speedMps;
    j["loop"] = v.loop;
    j["startOffsetMs"] = v.startOffsetMs;
    j["confidence"] = v.confidence;
    return j;
}

json toJson(const ScenarioIssue& v) {
    json j = json::object();
    j["path"] = v.path;
    j["field"] = v.field;
    j["reason"] = v.reason;
    return j;
}

json ValidationResult::toJson() const {
    json j = json::object();
    j["ok"] = ok;
    json arr = json::array();
    for (const ScenarioIssue& i : issues) arr.push_back(sim_source::toJson(i));
    j["issues"] = arr;
    return j;
}

json PlanResult::toJson() const {
    json j = json::object();
    j["ok"] = ok;
    j["message"] = message;
    json ids = json::array();
    for (const std::string& id : platformIds) ids.push_back(id);
    j["platformIds"] = ids;
    json rs = json::array();
    for (const auto& r : routes) rs.push_back(polygonJson(r));
    j["routes"] = rs;
    json gs = json::array();
    for (const std::string& g : groupKeys) gs.push_back(g);
    j["groupKeys"] = gs;
    j["hardZones"] = hardZones;
    j["softZones"] = softZones;
    j["detoured"] = detoured;
    json ws = json::array();
    for (const std::string& w : warnings) ws.push_back(w);
    j["warnings"] = ws;
    return j;
}

json toJson(const SimScenario& s) {
    json j = json::object();
    j["scenarioKey"] = s.scenarioKey;
    json as = json::array();
    for (const Area& a : s.areas) as.push_back(toJson(a));
    j["areas"] = as;
    json gs = json::array();
    for (const Group& g : s.groups) gs.push_back(toJson(g));
    j["groups"] = gs;
    json ps = json::array();
    for (const Platform& p : s.platforms) ps.push_back(toJson(p));
    j["platforms"] = ps;
    json ts = json::array();
    for (const Target& t : s.targets) ts.push_back(toJson(t));
    j["targets"] = ts;
    return j;
}

json SimSource::toJson(const SimScenario& scenario) { return sim_source::toJson(scenario); }

ValidationResult SimSource::fromJson(const json& j, SimScenario& out) {
    ValidationResult r;
    auto issue = [&r](const std::string& path, const std::string& field, const std::string& why) {
        ScenarioIssue i;
        i.path = path;
        i.field = field;
        i.reason = why;
        r.issues.push_back(i);
        r.ok = false;
    };
    if (!j.is_object()) {
        issue("$", "", "不是 JSON 对象");
        return r;
    }
    SimScenario s;
    auto str = [](const json& o, const char* key) -> std::string {
        auto it = o.find(key);
        if (it == o.end() || !it->is_string()) return std::string();
        return it->get<std::string>();
    };
    auto num = [](const json& o, const char* key, double dflt) -> double {
        auto it = o.find(key);
        if (it == o.end() || !it->is_number()) return dflt;
        return it->get<double>();
    };
    auto bnum = [](const json& o, const char* key, bool dflt) -> bool {
        auto it = o.find(key);
        if (it == o.end() || !it->is_boolean()) return dflt;
        return it->get<bool>();
    };
    s.scenarioKey = str(j, "scenarioKey");

    if (j.contains("areas")) {
        if (!j["areas"].is_array()) {
            issue("areas", "areas", "不是数组");
        } else {
            for (std::size_t i = 0; i < j["areas"].size(); ++i) {
                const json& a = j["areas"][i];
                Area area;
                area.key = str(a, "key");
                area.name = str(a, "name");
                const std::string role = str(a, "role");
                const auto pr = areaRoleFromString(role);
                if (!pr.has_value()) {
                    issue("areas[" + std::to_string(i) + "]", "role",
                          "未知 role：" + role + "（取值 deploy|task|nofly）");
                } else {
                    area.role = *pr;
                }
                area.polygon = a.contains("polygon") ? polygonFromJson(a["polygon"])
                                                     : std::vector<std::pair<double, double>>{};
                if (a.contains("hardness") && a["hardness"].is_string()) {
                    const std::string h = a["hardness"].get<std::string>();
                    const auto ph = hardnessFromString(h);
                    if (!ph.has_value()) {
                        issue("areas[" + std::to_string(i) + "]", "hardness",
                              "未知 hardness：" + h + "（取值 hard|soft）");
                    } else {
                        area.hardness = *ph;
                        area.hasHardness = true;
                    }
                }
                s.areas.push_back(area);
            }
        }
    }
    if (j.contains("groups") && j["groups"].is_array()) {
        for (const json& g : j["groups"]) {
            Group grp;
            grp.key = str(g, "key");
            grp.name = str(g, "name");
            grp.role = str(g, "role");
            s.groups.push_back(grp);
        }
    }
    if (j.contains("platforms")) {
        if (!j["platforms"].is_array()) {
            issue("platforms", "platforms", "不是数组");
        } else {
            for (std::size_t i = 0; i < j["platforms"].size(); ++i) {
                const json& p = j["platforms"][i];
                Platform pl;
                pl.deviceId = str(p, "deviceId");
                pl.deviceType = str(p, "deviceType");
                pl.kind = str(p, "kind");
                pl.groupKey = str(p, "groupKey");
                pl.homeAreaKey = str(p, "homeAreaKey");
                pl.taskAreaKey = str(p, "taskAreaKey");
                pl.altM = num(p, "altM", 0.0);
                pl.speedMps = num(p, "speedMps", 0.0);
                pl.battery = num(p, "battery", 100.0);
                if (p.contains("startOffset") && p["startOffset"].is_object()) {
                    pl.startOffset.rightM = num(p["startOffset"], "rightM", 0.0);
                    pl.startOffset.fwdM = num(p["startOffset"], "fwdM", 0.0);
                }
                s.platforms.push_back(pl);
            }
        }
    }
    if (j.contains("targets")) {
        if (!j["targets"].is_array()) {
            issue("targets", "targets", "不是数组");
        } else {
            for (std::size_t i = 0; i < j["targets"].size(); ++i) {
                const json& t = j["targets"][i];
                Target tg;
                if (t.contains("no") && t["no"].is_number()) tg.no = t["no"].get<int>();
                tg.id = str(t, "id");
                if (tg.id.empty()) tg.id = "target-" + std::to_string(tg.no);
                tg.typeKey = str(t, "typeKey");
                tg.deviceType = str(t, "deviceType");
                tg.kind = str(t, "kind");
                tg.name = str(t, "name");
                const std::string motion = str(t, "motion");
                const auto pm = targetMotionFromString(motion);
                if (!pm.has_value()) {
                    issue("targets[" + std::to_string(i) + "]", "motion",
                          "未知 motion：" + motion + "（取值 static|dynamic|popup）");
                } else {
                    tg.motion = *pm;
                }
                if (t.contains("position") && t["position"].is_array() &&
                    t["position"].size() >= 2 && t["position"][0].is_number() &&
                    t["position"][1].is_number()) {
                    // `position` 是**一个**点（[lng, lat]），不是点集 —— 与 `polygon`/`route`
                    // 的"数组的数组"形状不同，早期版本误用了点集解析器 → 位置被静默丢弃
                    // （见 docs/实现报告.md §4）。
                    tg.hasPosition = true;
                    tg.position = {t["position"][0].get<double>(), t["position"][1].get<double>()};
                }
                if (t.contains("route")) tg.route = polygonFromJson(t["route"]);
                tg.speedMps = num(t, "speedMps", 0.0);
                tg.loop = bnum(t, "loop", false);
                if (t.contains("startOffsetMs") && t["startOffsetMs"].is_number()) {
                    tg.startOffsetMs = t["startOffsetMs"].get<int64_t>();
                }
                tg.confidence = num(t, "confidence", 1.0);
                s.targets.push_back(tg);
            }
        }
    }
    out = s;
    return r;
}

// ============================================================================
// §4 位姿 / 观测 / 状态 / 自述
// ============================================================================

json toJson(const EntityPose& v) {
    json j = json::object();
    j["platformId"] = v.platformId;
    j["deviceType"] = v.deviceType;
    j["lng"] = detail::numOrNull(v.lng);
    j["lat"] = detail::numOrNull(v.lat);
    j["altM"] = detail::numOrNull(v.altM);
    j["heading"] = detail::numOrNull(v.heading);
    j["speedMps"] = detail::numOrNull(v.speedMps);
    j["isTarget"] = v.isTarget;
    return j;
}

json toJson(const SensorPose& v) {
    json j = json::object();
    j["sensorId"] = v.sensorId;
    j["deviceId"] = v.deviceId;
    j["deviceType"] = v.deviceType;
    j["rangeM"] = v.rangeM;
    j["ts"] = v.ts;
    j["self"] = toJson(v.self);
    json arr = json::array();
    for (const EntityPose& c : v.candidates) arr.push_back(toJson(c));
    j["candidates"] = arr;
    return j;
}

json SimObservation::toJson() const {
    json j = json::object();
    j["sensorId"] = sensorId;
    j["deviceType"] = deviceType;
    j["kind"] = kind;
    j["targetId"] = targetId;
    j["targetType"] = targetType;
    j["lng"] = detail::numOrNull(lng);
    j["lat"] = detail::numOrNull(lat);
    j["altM"] = detail::numOrNull(altM);
    j["heading"] = detail::numOrNull(heading);
    j["speedMps"] = detail::numOrNull(speedMps);
    j["confidence"] = detail::numOrNull(confidence);
    j["ts"] = ts;
    if (extensions.is_object() && !extensions.empty()) {
        std::vector<std::string> keys;
        keys.reserve(extensions.size());
        for (auto it = extensions.begin(); it != extensions.end(); ++it) keys.push_back(it.key());
        std::sort(keys.begin(), keys.end());
        for (const std::string& k : keys) j[k] = extensions[k];
    }
    return j;
}

json toJson(const SimObservation& v) { return v.toJson(); }

json toJson(const EntityStatus& v) {
    json j = json::object();
    j["id"] = v.id;
    j["deviceType"] = v.deviceType;
    j["kind"] = v.kind;
    j["isTarget"] = v.isTarget;
    j["groupKey"] = v.groupKey;
    j["state"] = toString(v.state);
    j["lng"] = detail::numOrNull(v.lng);
    j["lat"] = detail::numOrNull(v.lat);
    j["altM"] = detail::numOrNull(v.altM);
    j["heading"] = detail::numOrNull(v.heading);
    j["speedMps"] = detail::numOrNull(v.speedMps);
    j["battery"] = v.battery;
    j["seq"] = v.seq;
    j["waypointIndex"] = v.waypointIndex;
    j["visible"] = v.visible;
    j["inTaskArea"] = v.inTaskArea;
    j["inHardZone"] = v.inHardZone;
    j["inSoftZone"] = v.inSoftZone;
    return j;
}

json toJson(const Metrics& v) {
    json j = json::object();
    j["ticks"] = v.ticks;
    j["steps"] = v.steps;
    j["substeps"] = v.substeps;
    j["eventsEmitted"] = v.eventsEmitted;
    j["observationsEmitted"] = v.observationsEmitted;
    j["sensorCalls"] = v.sensorCalls;
    j["sensorErrors"] = v.sensorErrors;
    j["sinkErrors"] = v.sinkErrors;
    j["pausedMs"] = v.pausedMs;
    j["simElapsedMs"] = v.simElapsedMs;
    j["popupSpawned"] = v.popupSpawned;
    j["arrivals"] = v.arrivals;
    return j;
}

json toJson(const Capabilities& v) {
    json j = json::object();
    j["initialized"] = v.initialized;
    j["clockInjected"] = v.clockInjected;
    j["sinkInjected"] = v.sinkInjected;
    j["customSensorInjected"] = v.customSensorInjected;
    j["platforms"] = v.platforms;
    j["targets"] = v.targets;
    j["groups"] = v.groups;
    j["areas"] = v.areas;
    j["hardZones"] = v.hardZones;
    j["sensors"] = v.sensors;
    j["speedMultiplier"] = v.speedMultiplier;
    j["paused"] = v.paused;
    j["simElapsedMs"] = v.simElapsedMs;
    j["moduleVersion"] = v.moduleVersion;
    j["scenarioKey"] = v.scenarioKey;
    return j;
}

}  // namespace sim_source
