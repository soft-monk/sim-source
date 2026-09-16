# sim-source · 仿真与感知源

> **没有实装设备时，地图上会动的实体、以及它们在哪，从哪来。**

一个可独立构建、可复用、可整包交付的 C++ 模块。

---

## 它解决什么问题

演示与联调阶段没有真实无人机。但"随便造几个点让它动"是不够的——演示数据要有**业务含义**：
从部署区起飞、按航线飞行、避让禁飞区、按编队保持位形。

本模块把这件事做干净：**吃中立结构（不是业务表），吐归一化事件**。

## 做 / 不做

| 做 | 不做 |
|---|---|
| 编队航路规划（部署区 → 任务区，**避让硬禁飞区**） | 🚫 **不做协议收发**（`device-ingest` 的活） |
| 实体运动学：位置 / 航向 / 速度 / 电量按注入时钟推进 | 🚫 **不做渲染**（`map-2d` 的活） |
| 节拍控制：实时 / 倍速（1× 8× 60×）/ 暂停 / 单步 | 🚫 **不做业务判断**（阶段、评级、编组） |
| 目标运动（静止 / 航线 / 中途出现） | 🚫 **不内置探测模型**（交 `sensor-model`） |
| 事件出口：逐字段等于 `device-ingest` 归一化对象 | 🚫 **不查业务表**（那会把它绑死在某个 schema 上） |

## 一句话边界

> **本模块只管"实体怎么动"；"传感器看见了什么"交 `sensor-model`；"数据怎么上路"交 `device-ingest`。**

三者**互不 import**，靠注入的接口与中立结构对接。

## 为什么它该独立

`device-ingest` 的 `device_source.h` 已经把这条边界写死了：

> 「模块只管把设备报文收进来、解析、归一、发出去；
> **"演示数据长什么样、任务进度怎么走"是主仓的业务模拟（BusinessSim）**」

并提供了 `IDeviceSource` 反向接口。**做成独立模块正是落实那个边界**，而不是违背它。

## 文档

`docs/需求/sim-source需求专篇.md`（待写入）

`docs/实现报告.md` —— 实现口径、踩过的坑、**待裁决问题（T1–T6）**、复现命令与实测结果。

### 已裁决的口径（P0.5 后定，T1–T6 全部收口）

| # | 议题 | 裁决 | 依据 |
|---|---|---|---|
| **T1** | 契约 §2 的 `tsSource`/`recvAt`/`source` 是 MUST，但只有接入层知道 | **维持现状**：本引擎**只产 11 字段**，那三个由 `device-ingest` 在接入时补 | 契约允许 `tsSource:"device"`（用报文内 ts）。本引擎产出的 `ts` 存在且就是事件时刻 → 接入层会判为 `"device"` 并自填 `recvAt`/`source` |
| **T2** | `kind` 为空时的回落 | **宿主必须显式注入**（`SimOptions::defaultKind`）。本项目取值 **`uav.pos`** | 契约 L196：`kind` 与事件名**一一对应**（`uav.pos` ↔ `telemetry.uav.pos`），前端按后者分发。若回落成别的值 → 变成未知事件 → **实体不上图** |
| **T3** | 编队位形语义 | **刚体平移**（相对位形逐航点恒定、可机检）。**不要求随航向转弯** | 本阶段无需转弯保持；若要，须先定义拐点处位形口径（属后续迭代） |
| **T4** | `loop` 一圈的里程口径 | `[A,B]` = **往返**（2×\|AB\|）；`[A,B,C]` 闭合成多边形。演示用**多点航线**，避免歧义 | 两点航线只能往返才连续 |
| **T5** | 内部子步 `substepMs` 缺省 100 ms | **接受** | 保证 60× 倍速下不跳过拐点；与宿主 tick 频率解耦 |
| **T6** | `tick05`/`route03` 强绑定本实现口径 | **保留**，并在断言名与报告中标注"绑定 T3/T5 口径" | 口径若变，这两条断言需同步改——已留痕 |

## 快速上手

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
build\bin\Release\example_minimal.exe                              # 最小闭环（绕行禁飞区）
build\bin\Release\example_full_flow.exe                            # 全流程（节拍/目标/传感器）
build\bin\Release\selftest.exe                                     # 零依赖自测
powershell -ExecutionPolicy Bypass -File scripts\acceptance.ps1    # 独立验收（退出码 0/1）
```

宿主只需要 `#include <sim_source/sim_source.h>`：喂 `SimScenario`（中立结构）进来，
实现 `ISimSink` / `ISensorModel`（可选 `IClock`）注入回去，拿归一化事件。

## 许可

Apache License 2.0，见 [LICENSE](LICENSE)。
