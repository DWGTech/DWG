# This module finds the f2c library.
#
# This module sets the following variables:
#  F2C_FOUND - set to true if library is found
#  F2C_DEFINITIONS - compilation options to use f2c
#  F2C_LIBRARIES - f2c library name (using full path name)

if (F2C_LIBRARIES)

  set(F2C_FOUND TRUE)

else(F2C_LIBRARIES)

  set(F2C_DEFINITIONS)

  # F2C shipped with CGAL (as part of TAUCS)?
  # If found, we will search for f2c library in ${CGAL_TAUCS_LIBRARIES_DIR}.
  include(${CMAKE_CURRENT_LIST_DIR}/CGAL_Locate_CGAL_TAUCS.cmake)

  find_library(F2C_LIBRARIES NAMES f2c g2c vcf2c
               PATHS ${CGAL_TAUCS_LIBRARIES_DIR}
               DOC "F2C library"
              )

  if(F2C_LIBRARIES)
    set(F2C_FOUND TRUE)
  else()
    set(F2C_FOUND FALSE)
  endif()

  if(NOT F2C_FIND_QUIETLY)
    if(F2C_FOUND)
      message(STATUS "f2c library found.")
    else(F2C_FOUND)
      if(F2C_FIND_REQUIRED)
        message(FATAL_ERROR "f2c library not found. Please specify library location.")
      else()
        message(STATUS "f2c library not found. Please specify library location.")
      endif()
    endif(F2C_FOUND)
  endif(NOT F2C_FIND_QUIETLY)

endif(F2C_LIBRARIES)
