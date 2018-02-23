#.rst:
# FindTetGen
# --------
#
# Find TetGen library
#
# Find the TetGen includes and library This module defines
#
# ::
#
#   TETGEN_INCLUDE_DIRS, where to find triangle.h.
#   TETGEN_LIBRARIES, libraries to link against to use triangle.
#   TETGEN_FOUND, If false, do not try to use TETGEN.
#
#
#
#
#
#=============================================================================

find_path(TETGEN_INCLUDE_DIR tetgen.h
          DOC "The TetGen include directory")

set(TETGEN_NAMES ${TETGEN_NAMES} libtetgen tetgen)
find_library(TETGEN_LIBRARY NAMES ${TETGEN_NAMES}
            DOC "The TetGen library")

# handle the QUIETLY and REQUIRED arguments and set TETGEN_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(TETGEN
                                  REQUIRED_VARS TETGEN_LIBRARY
                                                TETGEN_INCLUDE_DIR
                                  VERSION_VAR TETGEN_VERSION_STRING)

if(TETGEN_FOUND)
  set( TETGEN_LIBRARIES ${TETGEN_LIBRARY} )
  set( TETGEN_INCLUDE_DIRS ${TETGEN_INCLUDE_DIR} )
endif()

mark_as_advanced(TETGEN_INCLUDE_DIR TETGEN_LIBRARY)
