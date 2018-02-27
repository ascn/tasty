find_package(Eigen3 REQUIRED)
add_library(eigen INTERFACE IMPORTED)

# Generate the local Makefile
if(CREATE_LOCAL_MAKEFILES)
    file(RELATIVE_PATH CISPBA_RELATIVE_PATH ${CISPBA_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
    configure_file(${CISPBA_SOURCE_DIR}/Scripts/Makefile.in ${CMAKE_CURRENT_SOURCE_DIR}/Makefile @ONLY)
endif()

add_cispba_executable(main globalincludes.h main.cpp tetra.cpp tetra.h particles.h)
target_include_directories(main PUBLIC ${CISPBA_SOURCE_DIR}/tetgen/)
target_include_directories(main SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
target_compile_options(main PUBLIC -DTETLIBRARY)
target_link_libraries(main partio eigen ${CISPBA_SOURCE_DIR}/tetgen/libtet.a)
