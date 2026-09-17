cmake_minimum_required(VERSION 3.20)

project(HAI917I_IG_Avance_TPs LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

FetchContent_Declare(
    Eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 5.0.1
    GIT_SHALLOW TRUE
    EXCLUDE_FROM_ALL
)

FetchContent_MakeAvailable(Eigen)

add_subdirectory(deps/polyscope)

add_subdirectory(TP1_HPSS)

add_subdirectory(TP2_Dual_Contouring)
