#Generate the local Makefile
if(CREATE_LOCAL_MAKEFILES)
  file(RELATIVE_PATH CISPBA_RELATIVE_PATH ${CISPBA_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
  configure_file(${CISPBA_SOURCE_DIR}/Scripts/Makefile.in ${CMAKE_CURRENT_SOURCE_DIR}/Makefile @ONLY)
endif()

add_cispba_executable(example tetcall.cxx)
target_include_directories(example PUBLIC ${CISPBA_SOURCE_DIR}/tetgen/)
target_compile_options(example PUBLIC -DTETLIBRARY)
target_link_libraries(example ${CISPBA_SOURCE_DIR}/tetgen/libtet.a)
