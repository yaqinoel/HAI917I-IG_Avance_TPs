add_executable(TP3_Recalage
        src/main.cpp
)

target_include_directories(TP3_Recalage PRIVATE
        ${PROJECT_SOURCE_DIR}/deps/libigl/include
        ${PROJECT_SOURCE_DIR}/deps
)

target_link_libraries(TP3_Recalage PRIVATE
        polyscope
        Eigen3::Eigen
)

target_compile_definitions(TP3_Recalage PRIVATE
    TP3_Recalage_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)