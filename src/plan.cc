// src/plan.cc · 编队航路规划（部署区 → 任务区，避让 hard 禁飞区）
//
// 算法（刻意选最简单、可复现、可审计的一条路，README 只要求"避让"）：
//   ① 场景 → 局部平面（东/北，米）；
//   ② hard 禁飞区多边形按 `safetyMarginM × 编队膨胀系数` 外扩成障碍；
//   ③ 直连段若穿障 → 在「起点 + 终点 + 障碍顶点」上跑 A*；
//      边的合法性 = 该线段不穿入任何障碍（可见性判定，同一套几何）；
//   ④ 输出折线并校验"逐段不穿原环"；
//   ⑤ 编队成员按槽位把基航路整体偏移（保持相对位形）。
//
// 为什么用"顶点图 + A*"而不是栅格/势场：
//   · 顶点图在**多边形障碍**上是最优解所在（可见图性质），不会出现栅格法的锯齿与漏判；
//   · 无随机、无迭代次数依赖 → 同一输入必然同一输出（SIM-NFR-01）。
#include "internal.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace sim_source {
namespace detail {

/// A* 用的显式节点：起点 / 终点 / 障碍顶点
struct Node {
    Vec2 p;
    int kind = 0;  // 0 = 顶点，1 = 起点，2 = 终点
};

struct OpenItem {
    double f = 0.0;
    double g = 0.0;
    int node = 0;
    int parent = -1;
};

/// f 小的先出；f 相同时节点下标小的先出（**确定性**关键：不依赖堆的内部顺序）
struct OpenGreater {
    bool operator()(const OpenItem& a, const OpenItem& b) const {
        if (a.f != b.f) return a.f > b.f;
        return a.node > b.node;
    }
};

bool edgeLegal(const Vec2& a, const Vec2& b, const std::vector<Obstacle>& hard) {
    for (const Obstacle& o : hard) {
        if (segmentCrossesPolygon(o.ring, a, b)) return false;
    }
    return true;
}

/// 障碍的"外部绕行候选点"：包围盒四角再外扩 `clearanceM`。
///
/// **为什么必须有它**：只用「起点 + 终点 + 障碍顶点」建可见图时，若起点与终点都在障碍的
/// 同一侧"阴影"里，两组顶点之间可能一条可见边都没有（顶点之间的连线全都切进障碍内部），
/// 图会断成两半、A* 报无解 —— 真实踩过的坑（docs/实现报告.md §4）。
/// 绕行点补上"从外侧兜过去"的那一步。
std::vector<Vec2> clearanceNodes(const Obstacle& o, double clearanceM) {
    if (o.ring.empty()) return {};
    double xmin = o.ring.front().x, xmax = o.ring.front().x;
    double ymin = o.ring.front().y, ymax = o.ring.front().y;
    for (const Vec2& v : o.ring) {
        xmin = std::min(xmin, v.x);
        xmax = std::max(xmax, v.x);
        ymin = std::min(ymin, v.y);
        ymax = std::max(ymax, v.y);
    }
    const double mx = std::max(clearanceM * 0.05, (xmax - xmin) * 0.5);
    const double my = std::max(clearanceM * 0.05, (ymax - ymin) * 0.5);
    return {Vec2(xmin - mx, ymin - my), Vec2(xmax + mx, ymin - my), Vec2(xmax + mx, ymax + my),
            Vec2(xmin - mx, ymax + my)};
}

/// 编队膨胀系数：位形偏移不得吃掉绕行缓冲。
///
/// 口径（**待裁决问题 T3**，docs/实现报告.md §5）：成员航路 = 基航路沿"右/前"两轴平移，
/// 斜向成员的实际外扩 ≤ √2 × max(|right|, |fwd|)。故障碍外扩取
/// `safetyMarginM + 1.5 × maxOffset`，把位形占用一并算进缓冲。
/// 这是**保守近似**（不是几何精确偏移），逐平台仍有穿障复核兜底（见 planRoutes 的 warnings）。
double formationFactor(const std::vector<FormationSlot>& slots, double marginM) {
    double maxAbs = 0.0;
    for (const FormationSlot& s : slots) {
        maxAbs = std::max(maxAbs, std::fabs(s.rightM));
        maxAbs = std::max(maxAbs, std::fabs(s.fwdM));
    }
    if (!(maxAbs > 0.0) || !(marginM > 0.0)) return 1.0;
    return (marginM + 1.5 * maxAbs) / marginM;
}

LocalFrame frameFor(const SimScenario& s) {
    double lngMin = 0.0, lngMax = 0.0, latMin = 0.0, latMax = 0.0;
    bool first = true;
    auto feed = [&](double lng, double lat) {
        if (!std::isfinite(lng) || !std::isfinite(lat)) return;
        if (first) {
            lngMin = lngMax = lng;
            latMin = latMax = lat;
            first = false;
            return;
        }
        lngMin = std::min(lngMin, lng);
        lngMax = std::max(lngMax, lng);
        latMin = std::min(latMin, lat);
        latMax = std::max(latMax, lat);
    };
    for (const Area& a : s.areas) {
        for (const auto& pt : a.polygon) feed(pt.first, pt.second);
    }
    for (const Target& t : s.targets) {
        if (t.hasPosition) feed(t.position.first, t.position.second);
        for (const auto& pt : t.route) feed(pt.first, pt.second);
    }
    if (first) return LocalFrame::fromCenter(0.0, 0.0);
    return LocalFrame::fromCenter((lngMin + lngMax) * 0.5, (latMin + latMax) * 0.5);
}

void collectZones(const SimScenario& s, const LocalFrame& frame, std::vector<Obstacle>& hard,
                  std::vector<Poly>& soft) {
    hard.clear();
    soft.clear();
    for (const Area& a : s.areas) {
        if (a.role != AreaRole::NoFly) continue;
        Poly ring;
        ring.reserve(a.polygon.size());
        for (const auto& pt : a.polygon) ring.push_back(frame.toLocal(pt.first, pt.second));
        if (a.hasHardness && a.hardness == Hardness::Soft) {
            soft.push_back(ring);
        } else {
            Obstacle o;
            o.key = a.key;
            o.ring = ring;
            hard.push_back(o);
        }
    }
}

bool areaCentroidLocal(const SimScenario& s, const LocalFrame& frame, const std::string& key,
                       Vec2& out) {
    const Area* a = s.findArea(key);
    if (a == nullptr || a->polygon.empty()) return false;
    double x = 0.0, y = 0.0;
    for (const auto& pt : a->polygon) {
        const Vec2 v = frame.toLocal(pt.first, pt.second);
        x += v.x;
        y += v.y;
    }
    const double n = static_cast<double>(a->polygon.size());
    out = Vec2(x / n, y / n);
    return true;
}

bool planDetour(const Vec2& start, const Vec2& goal, const std::vector<Obstacle>& hard,
                double clearanceM, std::vector<Vec2>& out) {
    out.clear();
    out.push_back(start);
    out.push_back(goal);

    // 直连可行 → 不绕（SIM-ROUTE-02：soft 与"不挡路的 hard"都不该改变航路）
    if (edgeLegal(start, goal, hard)) {
        return true;
    }

    std::vector<Node> nodes;
    for (const Obstacle& o : hard) {
        for (const Vec2& v : o.ring) {
            Node n;
            n.p = v;
            n.kind = 0;
            nodes.push_back(n);
        }
        for (const Vec2& v : clearanceNodes(o, clearanceM)) {
            Node n;
            n.p = v;
            n.kind = 0;
            nodes.push_back(n);
        }
    }
    if (nodes.empty()) return false;
    const int startIdx = static_cast<int>(nodes.size());
    {
        Node n;
        n.p = start;
        n.kind = 1;
        nodes.push_back(n);
    }
    const int goalIdx = static_cast<int>(nodes.size());
    {
        Node n;
        n.p = goal;
        n.kind = 2;
        nodes.push_back(n);
    }
    const int n = static_cast<int>(nodes.size());

    // 可见性邻接（只连合法边；O(n²) 判定，节点数 = 障碍顶点数，场景尺度下足够）
    std::vector<std::vector<int>> adj(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (!edgeLegal(nodes[static_cast<std::size_t>(i)].p, nodes[static_cast<std::size_t>(j)].p,
                           hard)) {
                continue;
            }
            adj[static_cast<std::size_t>(i)].push_back(j);
            adj[static_cast<std::size_t>(j)].push_back(i);
        }
    }

    const double kInf = std::numeric_limits<double>::infinity();
    std::vector<double> g(static_cast<std::size_t>(n), kInf);
    std::vector<int> parent(static_cast<std::size_t>(n), -1);
    std::vector<bool> closed(static_cast<std::size_t>(n), false);
    std::priority_queue<OpenItem, std::vector<OpenItem>, OpenGreater> open;

    g[static_cast<std::size_t>(startIdx)] = 0.0;
    {
        OpenItem it;
        it.f = (goal - start).len();
        it.g = 0.0;
        it.node = startIdx;
        open.push(it);
    }

    while (!open.empty()) {
        const OpenItem cur = open.top();
        open.pop();
        const std::size_t ci = static_cast<std::size_t>(cur.node);
        if (closed[ci]) continue;
        closed[ci] = true;
        if (cur.node == goalIdx) break;

        for (int nb : adj[ci]) {
            const std::size_t ni = static_cast<std::size_t>(nb);
            if (closed[ni]) continue;
            const double w = (nodes[ni].p - nodes[ci].p).len();
            const double ng = g[ci] + w;
            if (ng + 1e-9 < g[ni]) {
                g[ni] = ng;
                parent[ni] = cur.node;
                OpenItem it;
                it.g = ng;
                it.f = ng + (nodes[ni].p - goal).len();
                it.node = nb;
                it.parent = cur.node;
                open.push(it);
            }
        }
    }

    if (!std::isfinite(g[static_cast<std::size_t>(goalIdx)])) return false;

    std::vector<Vec2> path;
    for (int i = goalIdx; i != -1; i = parent[static_cast<std::size_t>(i)]) {
        path.push_back(nodes[static_cast<std::size_t>(i)].p);
        if (i == startIdx) break;
    }
    std::reverse(path.begin(), path.end());
    if (path.size() < 2) return false;
    out = path;
    return true;
}

bool planBaseRoute(const SimScenario& s, const LocalFrame& frame, const std::string& homeAreaKey,
                   const std::string& taskAreaKey, const std::vector<Obstacle>& hard,
                   double clearanceM, std::vector<Vec2>& out, bool& detoured) {
    Vec2 start, goal;
    if (!areaCentroidLocal(s, frame, homeAreaKey, start)) return false;
    if (!areaCentroidLocal(s, frame, taskAreaKey, goal)) return false;

    // 航路两端若落在障碍里：先把端点推到障碍外，否则 A* 永远无解
    auto pushOut = [&](Vec2& v) {
        for (int guard = 0; guard < 8; ++guard) {
            const Obstacle* hit = nullptr;
            for (const Obstacle& o : hard) {
                if (pointInPolygon(o.ring, v)) {
                    hit = &o;
                    break;
                }
            }
            if (hit == nullptr) return;
            // 沿"离最近顶点的连线"向外推，步长按缓冲的 1/4
            Vec2 nearest = hit->ring.front();
            double best = std::numeric_limits<double>::infinity();
            for (const Vec2& c : hit->ring) {
                const double d = (v - c).len();
                if (d < best) {
                    best = d;
                    nearest = c;
                }
            }
            Vec2 dir = v - nearest;
            if (!(dir.len() > 1e-6)) dir = Vec2(1.0, 0.0);
            const double dl = dir.len();
            v = v + Vec2(dir.x / dl, dir.y / dl) * (best + 1.0);
        }
    };
    pushOut(start);
    pushOut(goal);

    detoured = false;
    std::vector<Vec2> path;
    if (!planDetour(start, goal, hard, clearanceM, path)) {
        out.assign({start, goal});
        return true;  // 直连（无解）：调用方负责告警，MUST NOT 静默假装绕开了
    }
    detoured = path.size() > 2;
    out = path;
    return true;
}

/// 按编队槽位把基航路偏移成成员航路（保持相对位形）。
///
/// **口径（SIM-ROUTE-03 / 待裁决问题 T3）**：偏移是**刚体平移** —— 整条航路平移同一个
/// 东/北向量，而不是"逐段各偏一次"。
///   · 刚体平移才能让"相对位形"成为**逐航点恒定**的不变量（可机检、可复现）；
///   · 逐段偏移会在拐点处产生方向不同的位移，位形在拐点被拉散（早期版本如此，
///     自测 route03 立刻挂掉，见 docs/实现报告.md §4）。
/// 因此 `FormationSlot.rightM/fwdM` 的定义是：**相对整条航路初始航向**的右/前偏移
/// （初始航向 = 基航路第一段的方向）。宿主若想让编队随航向转弯，请给出自己的航路。
std::vector<Vec2> offsetRoute(const std::vector<Vec2>& base, const FormationSlot& slot) {
    if (base.empty()) return base;
    if (slot.rightM == 0.0 && slot.fwdM == 0.0) return base;

    // 初始航向（基航路第一段的有效方向）
    Vec2 dir(0.0, 1.0);  // 退化时按正北
    for (std::size_t i = 0; i + 1 < base.size(); ++i) {
        const Vec2 seg = base[i + 1] - base[i];
        const double l = seg.len();
        if (l > 1e-9) {
            dir = Vec2(seg.x / l, seg.y / l);
            break;
        }
    }
    const Vec2 right(dir.y, -dir.x);  // 航向顺时针 90° = 右手侧
    const Vec2 off = right * slot.rightM + dir * slot.fwdM;

    std::vector<Vec2> out;
    out.reserve(base.size());
    for (const Vec2& p : base) out.push_back(p + off);
    return out;
}

std::vector<FormationSlot> defaultFormation(int count) {
    std::vector<FormationSlot> out;
    out.reserve(static_cast<std::size_t>(std::max(0, count)));
    for (int i = 0; i < count; ++i) out.push_back(FormationSlot{});
    return out;
}

}  // namespace detail

// ============================================================================
// 公开自由函数：路段穿入判定（口径与规划器完全一致）
// ============================================================================

bool SimSource::segmentCrossesPolygon(const std::pair<double, double>& a,
                                      const std::pair<double, double>& b,
                                      const std::vector<std::pair<double, double>>& polygon) {
    detail::Poly ring;
    ring.reserve(polygon.size());
    for (const auto& pt : polygon) ring.push_back(detail::Vec2(pt.first, pt.second));
    if (ring.size() < 3) return false;
    return detail::segmentCrossesPolygon(ring, detail::Vec2(a.first, a.second),
                                         detail::Vec2(b.first, b.second));
}

bool SimSource::routeClearOfHardZones(const std::vector<std::pair<double, double>>& route,
                                      const SimScenario& scenario) {
    if (route.size() < 2) return true;
    for (const Area& a : scenario.areas) {
        if (a.role != AreaRole::NoFly) continue;
        const bool hard = !(a.hasHardness && a.hardness == Hardness::Soft);
        if (!hard) continue;
        detail::Poly ring;
        ring.reserve(a.polygon.size());
        for (const auto& pt : a.polygon) ring.push_back(detail::Vec2(pt.first, pt.second));
        if (ring.size() < 3) continue;
        for (std::size_t i = 0; i + 1 < route.size(); ++i) {
            if (detail::segmentCrossesPolygon(ring, detail::Vec2(route[i].first, route[i].second),
                                              detail::Vec2(route[i + 1].first, route[i + 1].second))) {
                return false;
            }
        }
    }
    return true;
}

bool SimSource::containsPoint(const std::vector<std::pair<double, double>>& polygon, double lng,
                              double lat) {
    detail::Poly ring;
    ring.reserve(polygon.size());
    for (const auto& pt : polygon) ring.push_back(detail::Vec2(pt.first, pt.second));
    if (ring.size() < 3) return false;
    return detail::pointInPolygon(ring, detail::Vec2(lng, lat));
}

double SimSource::distanceMeters(double lng1, double lat1, double lng2, double lat2) {
    // 注意参数顺序：LocalFrame::fromCenter(参考经度, 参考纬度)
    const detail::LocalFrame f = detail::LocalFrame::fromCenter(lng1, lat1);
    const detail::Vec2 a = f.toLocal(lng1, lat1);
    const detail::Vec2 b = f.toLocal(lng2, lat2);
    return (b - a).len();
}

double SimSource::headingBetween(double lng1, double lat1, double lng2, double lat2) {
    const detail::LocalFrame f = detail::LocalFrame::fromCenter(lng1, lat1);
    const detail::Vec2 a = f.toLocal(lng1, lat1);
    const detail::Vec2 b = f.toLocal(lng2, lat2);
    const detail::Vec2 d = b - a;
    double deg = std::atan2(d.x, d.y) * 180.0 / detail::kPi;
    if (deg < 0.0) deg += 360.0;
    if (deg >= 360.0) deg -= 360.0;
    return deg;
}

double SimSource::routeLengthMeters(const std::vector<std::pair<double, double>>& route) {
    double total = 0.0;
    for (std::size_t i = 0; i + 1 < route.size(); ++i) {
        total += distanceMeters(route[i].first, route[i].second, route[i + 1].first,
                                route[i + 1].second);
    }
    return total;
}

// ============================================================================
// 规划（纯函数）—— init() 与 planRoutes() 共用这一条路径
// ============================================================================

PlanResult SimSource::planRoutes(const SimScenario& scenario, const SimOptions& opts) {
    return planRoutesInFrame(scenario, opts, detail::frameFor(scenario));
}

// 显式实例化：唯一合法实参是内部局部平面类型（宿主拿不到，也就无法传错）
template PlanResult SimSource::planRoutesInFrame<detail::LocalFrame>(const SimScenario&,
                                                                     const SimOptions&,
                                                                     const detail::LocalFrame&);

/// **规划主路径**：所有几何量必须在**同一个**局部平面里（否则"端点在一个帧、障碍环在另一个帧"
/// 的错位会让穿障判定恒为假 —— 这是一个曾经真实发生过的 bug，见 docs/实现报告.md §4）。
template <typename Frame>
PlanResult SimSource::planRoutesInFrame(const SimScenario& scenario, const SimOptions& opts,
                                        const Frame& frameArg) {
    PlanResult r;
    const ValidationResult vr = validate(scenario, opts);
    if (!vr.ok) {
        r.ok = false;
        r.message = "场景校验未通过：见 issues";
        for (const ScenarioIssue& i : vr.issues) r.warnings.push_back(i.path + "：" + i.reason);
        return r;
    }

    const detail::LocalFrame frame = frameArg;
    const double margin = (opts.safetyMarginM > 0.0) ? opts.safetyMarginM : 1.0;

    std::vector<detail::Obstacle> hardRaw;
    std::vector<detail::Poly> soft;
    detail::collectZones(scenario, frame, hardRaw, soft);
    r.hardZones = static_cast<int>(hardRaw.size());
    r.softZones = static_cast<int>(soft.size());

    // 逐编组：位形偏移会加宽实际占用，故障碍按编队膨胀后再规划（基航路一次，成员共用）
    struct GroupPlan {
        std::string key;
        std::vector<detail::Vec2> base;
        bool detoured = false;
        std::vector<std::string> members;
        std::vector<FormationSlot> slots;
    };

    std::vector<std::string> groupOrder;
    std::vector<GroupPlan> groupPlans;
    // 平台按 deviceId 字典序（去掉对声明顺序的依赖 → 双跑一致）
    std::vector<const Platform*> plats;
    plats.reserve(scenario.platforms.size());
    for (const Platform& p : scenario.platforms) plats.push_back(&p);
    std::sort(plats.begin(), plats.end(), [](const Platform* a, const Platform* b) {
        return a->deviceId < b->deviceId;
    });

    for (const Platform* p : plats) {
        std::string gk = p->groupKey;
        if (gk.empty()) gk = p->deviceId;  // 无编组 → 单独成组
        if (std::find(groupOrder.begin(), groupOrder.end(), gk) == groupOrder.end()) {
            groupOrder.push_back(gk);
        }
    }

    for (const std::string& gk : groupOrder) {
        GroupPlan gp;
        gp.key = gk;
        std::vector<const Platform*> members;
        for (const Platform* p : plats) {
            std::string pgk = p->groupKey;
            if (pgk.empty()) pgk = p->deviceId;
            if (pgk == gk) members.push_back(p);
        }
        if (members.empty()) continue;
        for (const Platform* p : members) {
            gp.members.push_back(p->deviceId);
            gp.slots.push_back(p->startOffset);
        }

        std::vector<detail::Obstacle> hardForGroup;
        hardForGroup.reserve(hardRaw.size());
        const double factor = detail::formationFactor(gp.slots, margin);
        for (const detail::Obstacle& o : hardRaw) {
            detail::Obstacle io;
            io.key = o.key;
            io.ring = detail::inflatePolygon(o.ring, margin * factor);
            hardForGroup.push_back(io);
        }

        std::vector<detail::Vec2> base;
        bool detoured = false;
        if (!detail::planBaseRoute(scenario, frame, members.front()->homeAreaKey,
                                   members.front()->taskAreaKey, hardForGroup, margin, base,
                                   detoured)) {
            r.warnings.push_back("编组 " + gk + "：部署区/任务区不存在或为空，无法规划");
            continue;
        }
        gp.base = base;
        gp.detoured = detoured;
        if (detoured) ++r.detoured;
        groupPlans.push_back(gp);
    }

    // 逐平台输出航路（基航路 → 位形偏移 → 穿障复核）
    for (const Platform* p : plats) {
        std::string pgk = p->groupKey;
        if (pgk.empty()) pgk = p->deviceId;
        const GroupPlan* gp = nullptr;
        for (const GroupPlan& g : groupPlans) {
            if (g.key == pgk) {
                gp = &g;
                break;
            }
        }
        if (gp == nullptr) continue;

        std::size_t slotIdx = 0;
        for (std::size_t i = 0; i < gp->members.size(); ++i) {
            if (gp->members[i] == p->deviceId) {
                slotIdx = i;
                break;
            }
        }
        const FormationSlot slot =
            (slotIdx < gp->slots.size()) ? gp->slots[slotIdx] : FormationSlot{};
        const std::vector<detail::Vec2> local = detail::offsetRoute(gp->base, slot);

        // 复核：偏移后的航路 MUST NOT 穿入**原环**（膨胀只是为了留缓冲）
        std::vector<detail::Poly> originals;
        for (const detail::Obstacle& o : hardRaw) originals.push_back(o.ring);
        if (detail::polylineCrossesAny(local, originals)) {
            r.warnings.push_back("平台 " + p->deviceId +
                                 "：编队偏移后航路贴入 hard 禁飞区，建议减小位形偏移或加大 safetyMarginM");
        }

        std::vector<std::pair<double, double>> wgs;
        wgs.reserve(local.size());
        for (const detail::Vec2& v : local) wgs.push_back(frame.toWgs(v));
        r.platformIds.push_back(p->deviceId);
        r.routes.push_back(wgs);
        r.groupKeys.push_back(pgk);
    }

    r.ok = !r.platformIds.empty() || scenario.platforms.empty();
    if (!r.ok) r.message = "没有任何平台被成功规划";
    if (r.ok && r.message.empty()) r.message = "ok";
    return r;
}

// ============================================================================
// PlanResult 便捷取值
// ============================================================================

const std::vector<std::pair<double, double>>* PlanResult::routeOf(
    const std::string& platformId) const {
    for (std::size_t i = 0; i < platformIds.size(); ++i) {
        if (platformIds[i] == platformId) return &routes[i];
    }
    return nullptr;
}

}  // namespace sim_source
