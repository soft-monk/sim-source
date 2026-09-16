// src/geometry.cc · 局部平面几何（纯函数，无状态、无 IO、无时间）
//
// 这一层刻意保持"只算几何"：它不认识区域角色、不认识禁飞区硬度，只回答
// "这条线段穿过多边形了吗""这个多边形外扩之后什么样"。
// 绕行策略在 plan.cc，运动学在 engine.cc。
#include "internal.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>

namespace sim_source {
namespace detail {

namespace {
/// 平面判定容差（米）：只用来吸收浮点噪声，不承担"安全间距"职责
/// （安全间距由 SimOptions::safetyMarginM 的外扩承担）。
constexpr double kGeomEps = 1e-9;

/// 点在多边形内（射线法）。
///
/// `boundaryCountsAsInside` 这个开关是**必须的**：
///   · true  —— "这个坐标是不是落在禁飞区里"（执法/报告口径）：边界上算在里面；
///   · false —— 绕行判定的"穿入内部"：必须与"擦过角点"区分开。
/// 早期版本只有 true，于是"航路顶点正好落在障碍顶点上"被误判成穿障，
/// A* 的可见图里每条边都被判非法 → 整图无解（真实踩过的坑，见 docs/实现报告.md §4）。
bool pointInPolygonImpl(const Poly& p, const Vec2& q, bool boundaryCountsAsInside) {
    const std::size_t n = p.size();
    if (n < 3) return false;
    bool inside = false;
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        const Vec2& a = p[i];
        const Vec2& b = p[j];
        if (pointSegmentDistance(q, a, b) <= kGeomEps) return boundaryCountsAsInside;
        const bool straddleX = (a.x > q.x) != (b.x > q.x);
        if (!straddleX) continue;
        const double t = (q.x - a.x) / (b.x - a.x);
        const double yAt = a.y + t * (b.y - a.y);
        if (yAt > q.y) inside = !inside;
    }
    return inside;
}
}  // namespace

// ---------------------------------------------------------------- Vec2 / LocalFrame

double Vec2::len() const { return std::sqrt(x * x + y * y); }

LocalFrame LocalFrame::fromCenter(double lng, double lat) {
    LocalFrame f;
    f.refLng = lng;
    f.refLat = lat;
    double c = std::cos(lat * kPi / 180.0);
    // 极区退化保护：|cos| 太小会让 x 方向尺度爆炸
    if (!(c > 0.01)) c = 0.01;
    f.cosLat0 = c;
    return f;
}

Vec2 LocalFrame::toLocal(double lng, double lat) const {
    const double x = (lng - refLng) * kMetersPerDegLngAtEquator * cosLat0;
    const double y = (lat - refLat) * kMetersPerDegLat;
    return Vec2(x, y);
}

std::pair<double, double> LocalFrame::toWgs(const Vec2& p) const {
    const double lng = refLng + p.x / (kMetersPerDegLngAtEquator * cosLat0);
    const double lat = refLat + p.y / kMetersPerDegLat;
    return {lng, lat};
}

// ---------------------------------------------------------------- 多边形基础

double polygonArea2(const Poly& p) {
    if (p.size() < 3) return 0.0;
    double a = 0.0;
    for (std::size_t i = 0, n = p.size(); i < n; ++i) {
        const Vec2& u = p[i];
        const Vec2& v = p[(i + 1) % n];
        a += u.cross(v);
    }
    return a;
}

bool validRing(const Poly& p) {
    if (p.size() < 3) return false;
    const double a2 = polygonArea2(p);
    if (!(std::fabs(a2) > 1e-6)) return false;
    for (const Vec2& v : p) {
        if (!std::isfinite(v.x) || !std::isfinite(v.y)) return false;
    }
    return true;
}

bool pointInPolygon(const Poly& p, const Vec2& q) {
    return pointInPolygonImpl(p, q, true);
}

bool pointStrictlyInPolygon(const Poly& p, const Vec2& q) {
    return pointInPolygonImpl(p, q, false);
}

double pointSegmentDistance(const Vec2& q, const Vec2& a, const Vec2& b) {
    const Vec2 ab = b - a;
    const double ab2 = ab.dot(ab);
    if (!(ab2 > 0.0)) return (q - a).len();
    double t = (q - a).dot(ab) / ab2;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    const Vec2 proj = a + ab * t;
    return (q - proj).len();
}

namespace {

/// 顶点贴合容差（米）：小于它的"交点落在顶点上"被当作**擦过**（合法），
/// 与"横穿进去"区分开。取 1e-4 m = 0.1 mm：远小于任何真实安全间距，
/// 又足够吸收 1e4 m 量级坐标上的浮点噪声。
constexpr double kVertexEps = 1e-4;

/// 两条线段的相交判定。返回 true = **横穿**（应判为穿障）。
///
/// 三种情况分开处理（这是绕行的正确性关键 —— 两个真实踩过的坑见 docs/实现报告.md §4）：
///   ① 交点落在对方线段**端点/顶点**上 → 视为**擦过**，不算穿入（可见图里"绕角点"就必须允许）；
///   ② 共线重叠 → 沿边界跑，不允许；
///   ③ 一般横穿 → 不允许。
/// 早期版本用无量纲参数 t 做容差（`t > 0.1/length`）：在 8 km 的长边上等价于 0.1 mm，
/// 于是一整个障碍的所有顶点边都被判成"相交"，A* 可见图退化成孤岛 → 整图无解。
bool segmentsProperlyCross(const Vec2& p1, const Vec2& p2, const Vec2& p3, const Vec2& p4) {
    const Vec2 r = p2 - p1;
    const Vec2 s = p4 - p3;
    const double rl = r.len();
    const double sl = s.len();
    if (!(rl > 0.0) || !(sl > 0.0)) return false;
    const Vec2 qp = p3 - p1;
    const double denom = r.cross(s);
    const double epsT = kVertexEps / rl;  // 换算成 r 方向上的参数容差（对应 kVertexEps 米）
    const double epsU = kVertexEps / sl;

    if (std::fabs(denom) <= kVertexEps * rl * sl) {
        // 平行：共线且重叠才算穿越（沿边界跑 = 不行）
        if (std::fabs(qp.cross(r)) > kVertexEps * rl) return false;
        const double rr = r.dot(r);
        double t0 = qp.dot(r) / rr;
        double t1 = t0 + s.dot(r) / rr;
        if (t0 > t1) std::swap(t0, t1);
        return (t1 > epsT) && (t0 < 1.0 - epsT);
    }
    const double t = qp.cross(s) / denom;
    const double u = qp.cross(r) / denom;
    if (!((t > epsT) && (t < 1.0 - epsT) && (u > epsU) && (u < 1.0 - epsU))) {
        // 不相交，或**交点落在某条线段的端点上**（擦过顶点）
        return false;
    }
    // 交点落在对方线段端点上？（0.1 mm 内即视为顶点贴合）
    if (u <= epsU * 2.0 || u >= 1.0 - epsU * 2.0) return false;
    if (t <= epsT * 2.0 || t >= 1.0 - epsT * 2.0) return false;
    return true;
}

}  // namespace

bool segmentCrossesPolygon(const Poly& p, const Vec2& a, const Vec2& b) {
    const std::size_t n = p.size();
    if (n < 3) return false;
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        if (segmentsProperlyCross(a, b, p[j], p[i])) return true;
    }
    // 端点 / 中点落在**内部**（不是边界）才算穿入 —— 擦过顶点是合法的绕行
    if (pointStrictlyInPolygon(p, a)) return true;
    if (pointStrictlyInPolygon(p, b)) return true;
    // 补一条中间点：防"两端都在外、却从顶点附近擦过内部"的漏判
    const Vec2 mid((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
    return pointStrictlyInPolygon(p, mid);
}

bool polylineCrossesAny(const std::vector<Vec2>& line, const std::vector<Poly>& obstacles) {
    if (line.size() < 2) {
        return false;
    }
    for (std::size_t i = 0; i + 1 < line.size(); ++i) {
        for (const Poly& o : obstacles) {
            if (segmentCrossesPolygon(o, line[i], line[i + 1])) return true;
        }
    }
    return false;
}

double polylineLength(const std::vector<Vec2>& line) {
    double total = 0.0;
    for (std::size_t i = 0; i + 1 < line.size(); ++i) total += (line[i + 1] - line[i]).len();
    return total;
}

// ---------------------------------------------------------------- 多边形外扩

namespace {

/// 两条直线的交点（`a1 + t*da` 与 `b1 + s*db`）；平行 → false
bool lineIntersect(const Vec2& a1, const Vec2& da, const Vec2& b1, const Vec2& db, Vec2& out) {
    const double denom = da.cross(db);
    if (std::fabs(denom) <= 1e-12) return false;
    const Vec2 qp = b1 - a1;
    const double t = qp.cross(db) / denom;
    out = a1 + da * t;
    return true;
}

}  // namespace

Poly inflatePolygon(const Poly& p, double margin) {
    const std::size_t n = p.size();
    if (n < 3 || !(margin > 0.0)) return p;

    // 统一成逆时针，使"外法线 = 边方向左转 90°"
    Poly ring = p;
    if (polygonArea2(ring) < 0.0) std::reverse(ring.begin(), ring.end());

    Poly out;
    out.reserve(n);
    const double degenerate = margin * 1e-6;
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2& prev = ring[(i + n - 1) % n];
        const Vec2& cur = ring[i];
        const Vec2& next = ring[(i + 1) % n];

        Vec2 e1 = cur - prev;
        Vec2 e2 = next - cur;
        if (!(e1.len() > degenerate) || !(e2.len() > degenerate)) {
            // 退化边（重复点）：按内角平分线方向简单外移，避免除零
            const Vec2 d = (next - prev);
            const double dl = d.len();
            const Vec2 nrm = (dl > 0.0) ? Vec2(d.y / dl, -d.x / dl) : Vec2(0.0, 0.0);
            out.push_back(cur + nrm * margin);
            continue;
        }
        const Vec2 n1(e1.y, -e1.x);  // 外法线（逆时针环）
        const Vec2 n2(e2.y, -e2.x);
        const double l1 = n1.len();
        const double l2 = n2.len();
        const Vec2 u1(n1.x / l1, n1.y / l1);
        const Vec2 u2(n2.x / l2, n2.y / l2);

        // 尖角保护：外角越小，miter 越长；超过 4×margin 改用截断（等价于圆角近似）
        const double cosTurn = u1.dot(u2);
        const double miterLen = margin / std::sqrt(std::max(0.05, (1.0 + cosTurn) * 0.5));
        const double m = (miterLen > margin * 4.0) ? margin * 4.0 : miterLen;

        Vec2 q;
        if (std::fabs(e1.cross(e2)) <= degenerate * (e1.len() + e2.len()) ||
            !lineIntersect(cur + u1 * margin, e1, cur + u2 * margin, e2, q)) {
            q = cur + (u1 + u2) * (margin * 0.5);
        } else {
            // 交点必须落在两条偏移线的正向侧，否则退回平分线近似
            const Vec2 v = q - cur;
            if (v.dot(u1) <= 0.0 || v.dot(u2) <= 0.0 || v.len() > m) {
                q = cur + (u1 + u2) * (margin * 0.5);
            }
        }
        out.push_back(q);
    }
    return out;
}

// ---------------------------------------------------------------- 确定性格式化

std::string formatFixed(double v, int decimals) {
    if (!std::isfinite(v)) return "null";
    std::ostringstream os;
    os.imbue(std::locale::classic());
    os << std::fixed << std::setprecision(decimals) << v;
    return os.str();
}

json numOrNull(double v) {
    if (!std::isfinite(v)) return json(nullptr);
    return json(v);
}

json fixedNum(double v, int decimals) {
    if (!std::isfinite(v)) return json(nullptr);
    // 经由定点字符串回转：值稳定、且不会出现 -0.0 与科学计数法
    const std::string s = formatFixed(v, decimals);
    return json(std::stod(s));
}

// ---------------------------------------------------------------- 摘要

std::string fnv1a64(const std::string& bytes) {
    std::uint64_t h = 1469598103934665603ULL;
    for (unsigned char c : bytes) {
        h ^= static_cast<std::uint64_t>(c);
        h *= 1099511628211ULL;
    }
    static const char* kHex = "0123456789abcdef";
    std::string out(16, '0');
    for (int i = 15; i >= 0; --i) {
        out[static_cast<std::size_t>(i)] = kHex[h & 0xFULL];
        h >>= 4;
    }
    return out;
}

}  // namespace detail
}  // namespace sim_source
