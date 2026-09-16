# tests/ · 零依赖自测（一个可执行 + 手写断言，不引任何测试框架）
#
# 覆盖 README「做 / 不做」与《外设接入契约》§2 的可机检清单（SIM-ROUTE/KIN/TICK/TGT/ELC/
# SENSOR/IN/OUT/NFR）。**不需要外部服务、不需要网络、不依赖真实时间、不依赖当前工作目录**
# （时钟一律注入假时钟 / 时间一律由 step() 显式推进）。
#
# 运行： selftest            → 跑全部
#        selftest --list     → 只列用例名
#        selftest --json     → 机检输出（acceptance.ps1 读它做需求↔用例对账）
#        selftest <名字片段>  → 只跑名字里含该片段的用例

add_executable(selftest selftest.cc)
target_link_libraries(selftest PRIVATE sim_source)

set_target_properties(selftest PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

if(MSVC)
  # /wd4819 /wd4474：源文件是 UTF-8，但 MSVC 在控制台代码页被切到 UTF-8 时
  # 会报"当前代码页(0)"—— 属环境噪声，非源码问题。
  target_compile_options(selftest PRIVATE /utf-8 /W4 /wd4819 /wd4474)
  target_compile_definitions(selftest PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
else()
  target_compile_options(selftest PRIVATE -Wall -Wextra)
endif()

add_test(NAME sim_source_selftest COMMAND selftest)
