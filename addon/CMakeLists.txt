cmake_minimum_required(VERSION 3.20)
project(RealFluid LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Static CRT: the add-on then needs no Visual C++ redistributable.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded")

include(FetchContent)
# Only ReShade's header-only add-on SDK (include/reshade.hpp etc.) is used.
# NOT VERIFIED against your ReShade build: if configuring fails because this tag does
# not exist, pick any v6.x tag from https://github.com/crosire/reshade/tags and pass
#   -DRESHADE_TAG=<tag>
set(RESHADE_TAG "v6.1.1" CACHE STRING "ReShade git tag providing the add-on headers")
FetchContent_Declare(reshade
    GIT_REPOSITORY https://github.com/crosire/reshade.git
    GIT_TAG        ${RESHADE_TAG}
    GIT_SHALLOW    TRUE)
FetchContent_MakeAvailable(reshade)

add_library(realfluid SHARED src/main.cpp src/protocol.hpp)
target_include_directories(realfluid PRIVATE src ${reshade_SOURCE_DIR}/include)
target_link_libraries(realfluid PRIVATE ws2_32)
target_compile_definitions(realfluid PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX WIN32_LEAN_AND_MEAN)
if(MSVC)
    target_compile_options(realfluid PRIVATE /W3 /permissive-)
endif()
set_target_properties(realfluid PROPERTIES PREFIX "" OUTPUT_NAME "RealFluid" SUFFIX ".addon")
