add_executable(TP2_Dual_Contouring
        src/main.cpp
)

target_include_directories(TP2_Dual_Contouring PRIVATE
        ${PROJECT_SOURCE_DIR}/deps/libigl/include
        ${PROJECT_SOURCE_DIR}/deps
)

target_link_libraries(TP2_Dual_Contouring PRIVATE
        polyscope
        Eigen3::Eigen
)

target_compile_definitions(TP2_Dual_Contouring PRIVATE
    TP2_Dual_Contouring_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)