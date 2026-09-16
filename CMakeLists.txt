# sim-source · 仿真与感知源（独立构建）
#
# 边界依据：README.md「做 / 不做」+ include/sim_source/sim_source.h 头部（五条不可协商口径）
# 契约依据：../device-ingest/docs/契约/外设接入契约.md §2 归一化对象
#
# 目标：
#   sim_source        静态库（宿主唯一依赖）
#   example_*         examples/ 下的示例
#   selftest          tests/ 下的零依赖自测（注册为 ctest 用例）
#
# 依赖：仅 nlohmann/json（优先系统包，找不到时回落内置单头 third_party/nlohmann/json.hpp）。
# 本模块**永远不需要** SQL / 网络库 / Web 框架（README「不做」列 + 结构守卫 C10/C13/C14）。
cmake_minimum_required(VERSION 3.20)
project(sim_source
  VERSION 0.1.0
  DESCRIPTION "仿真与感知源：编队航路规划 / 实体运动学 / 时间推进 / 归一化事件产出"
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "构建类型" FORCE)
endif()

# ---------------------------------------------------------------- 选项
option(SIM_SOURCE_BUILD_TESTS    "编译 tests/（零依赖自测）" ON)
option(SIM_SOURCE_BUILD_EXAMPLES "编译 examples/（最小示例、全流程演示）" ON)
option(SIM_SOURCE_WARNINGS_AS_ERRORS "把编译告警当错误" OFF)

# ---------------------------------------------------------------- 依赖
find_package(nlohmann_json CONFIG QUIET)
if(nlohmann_json_FOUND)
  set(SIM_SOURCE_JSON_TARGET nlohmann_json::nlohmann_json)
  message(STATUS "[sim-source] nlohmann/json: 使用已安装的包")
else()
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlohmann/json.hpp")
    message(FATAL_ERROR
      "[sim-source] 未找到 nlohmann/json：既没有安装包，也没有内置副本 "
      "third_party/nlohmann/json.hpp。请安装 nlohmann-json 或放入内置副本。")
  endif()
  add_library(sim_source_nlohmann INTERFACE)
  target_include_directories(sim_source_nlohmann INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party")
  set(SIM_SOURCE_JSON_TARGET sim_source_nlohmann)
  message(STATUS "[sim-source] nlohmann/json: 使用内置单头副本 third_party/nlohmann/json.hpp")
endif()

# ---------------------------------------------------------------- 库
file(GLOB_RECURSE SIM_SOURCE_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc")

add_library(sim_source STATIC ${SIM_SOURCE_SOURCES})
add_library(sim_source::sim_source ALIAS sim_source)

target_include_directories(sim_source
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src")

# 只依赖 nlohmann/json。**不链任何线程/网络/数据库库**（本模块不使用它们）
target_link_libraries(sim_source PUBLIC ${SIM_SOURCE_JSON_TARGET})

target_compile_definitions(sim_source PUBLIC
  SIM_SOURCE_VERSION="${PROJECT_VERSION}")

if(MSVC)
  target_compile_options(sim_source PRIVATE /utf-8 /W4 /permissive-)
  target_compile_definitions(sim_source PRIVATE
    NOMINMAX WIN32_LEAN_AND_MEAN _CRT_SECURE_NO_WARNINGS)
  if(SIM_SOURCE_WARNINGS_AS_ERRORS)
    target_compile_options(sim_source PRIVATE /WX)
  endif()
else()
  target_compile_options(sim_source PRIVATE -Wall -Wextra)
  if(SIM_SOURCE_WARNINGS_AS_ERRORS)
    target_compile_options(sim_source PRIVATE -Werror)
  endif()
endif()

set_target_properties(sim_source PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
  ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# ---------------------------------------------------------------- 示例 / 测试
if(SIM_SOURCE_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()
if(SIM_SOURCE_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

# ---------------------------------------------------------------- 汇总
message(STATUS "[sim-source] ${PROJECT_NAME} ${PROJECT_VERSION} · C++${CMAKE_CXX_STANDARD}")
message(STATUS "[sim-source]   构建类型 : ${CMAKE_BUILD_TYPE}")
message(STATUS "[sim-source]   examples/tests : "
               "${SIM_SOURCE_BUILD_EXAMPLES}/${SIM_SOURCE_BUILD_TESTS}")
