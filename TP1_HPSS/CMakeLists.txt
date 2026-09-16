set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(DEPS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps)

include(FetchContent)

# Dependencies
FetchContent_Declare(
        Eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 5.0.1
        GIT_SHALLOW TRUE
        EXCLUDE_FROM_ALL
)

FetchContent_MakeAvailable(Eigen)

# Dependencies
if(NOT TARGET polyscope)
    add_subdirectory(${DEPS_DIR}/polyscope)
endif()

# Application
add_executable(TP1_HPSS
        src/main.cpp
)

target_include_directories(TP1_HPSS PRIVATE
        ${DEPS_DIR}/libigl/include
        ${DEPS_DIR}/deps/
        ${DEPS_DIR}
)

target_compile_definitions(TP1_HPSS PRIVATE
        TP1_HPSS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(TP1_HPSS PRIVATE
        polyscope
        Eigen3::Eigen
)
