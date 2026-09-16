# examples/ · 可跑示例（各自独立可执行 + 独立退出码）
#
# 两个示例都不联网、不落库、不读当前工作目录（SIM-NFR-05）：场景直接内置在源码里
# （那是**演示数据**，合法住所就是 examples/），事件出口用自带的收集器 Sink。

add_executable(example_minimal minimal/main.cc)
target_link_libraries(example_minimal PRIVATE sim_source)

add_executable(example_full_flow full_flow/main.cc)
target_link_libraries(example_full_flow PRIVATE sim_source)

foreach(tgt example_minimal example_full_flow)
  set_target_properties(${tgt} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
  if(MSVC)
    # /wd4819 /wd4474：源文件是 UTF-8，MSVC 在控制台代码页异常时会报"当前代码页(0)"——环境噪声
    target_compile_options(${tgt} PRIVATE /utf-8 /W4 /wd4819 /wd4474)
    target_compile_definitions(${tgt} PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
  else()
    target_compile_options(${tgt} PRIVATE -Wall -Wextra)
  endif()
endforeach()
