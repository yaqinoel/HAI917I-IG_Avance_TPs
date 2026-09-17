add_executable(TP1_HPSS
        src/main.cpp
)

target_include_directories(TP1_HPSS PRIVATE
        ${PROJECT_SOURCE_DIR}/deps/libigl/include
        ${PROJECT_SOURCE_DIR}/deps
)

target_link_libraries(TP1_HPSS PRIVATE
        polyscope
        Eigen3::Eigen
)

target_compile_definitions(TP1_HPSS PRIVATE
    TP1_HPSS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
