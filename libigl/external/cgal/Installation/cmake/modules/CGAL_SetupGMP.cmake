#.rst:
# CGAL_SetupGMP
# -------------
#
# The module searchs for the `GMP` and `MPFR` headers and libraries,
# by calling
#
# .. code-block:: cmake
#
#    find_package(GMP)
#    find_package(MPFR)
#
# and defines the function :command:`use_CGAL_GMP_support`.

if(CGAL_SetupGMP_included OR CGAL_DISABLE_GMP)
  return()
endif()
set(CGAL_SetupGMP_included TRUE)

find_package(GMP REQUIRED)
find_package(MPFR REQUIRED)

if(NOT DEFINED WITH_GMPXX)
  option(CGAL_WITH_GMPXX "Use CGAL with GMPXX: use C++ classes of GNU MP instead of CGAL wrappers" OFF)
endif()
set(CGAL_GMPXX_find_package_keyword QUIET)
if(WITH_GMPXX OR CGAL_WITH_GMPXX)
  set(CGAL_GMPXX_find_package_keyword REQUIRED)
endif()
find_package(GMPXX ${CGAL_GMPXX_find_package_keyword})

#.rst:
# Provided Functions
# ^^^^^^^^^^^^^^^^^^
#
# .. command:: use_CGAL_GMP_support
#
#    Link the target with the `GMP` and `MPFR` libraries::
#
#      use_CGAL_GMP_support( target [INTERFACE] )
#
#    If the option ``INTERFACE`` is passed, the dependencies are
#    added using :command:`target_link_libraries` with the ``INTERFACE``
#    keyword, or ``PUBLIC`` otherwise.

function(use_CGAL_GMP_support target)
  if(ARGV1 STREQUAL INTERFACE)
    set(keyword INTERFACE)
  else()
    set(keyword PUBLIC)
  endif()
  if(NOT GMP_FOUND OR NOT MPFR_FOUND)
    message(FATAL_ERROR "CGAL requires GMP and MPFR.")
    return()
  endif()

  if(NOT GMP_IN_CGAL_AUXILIARY)
    target_include_directories(${target} SYSTEM ${keyword} ${GMP_INCLUDE_DIR})
  else()
    target_include_directories(${target} SYSTEM ${keyword}
      $<BUILD_INTERFACE:${GMP_INCLUDE_DIR}>
      $<INSTALL_INTERFACE:include>)
  endif()
  target_link_libraries(${target} ${keyword} ${GMP_LIBRARIES})
  if(NOT MPFR_IN_CGAL_AUXILIARY)
    target_include_directories(${target} SYSTEM ${keyword} ${MPFR_INCLUDE_DIR})
  else()
    target_include_directories(${target} SYSTEM ${keyword}
      $<BUILD_INTERFACE:${MPFR_INCLUDE_DIR}>
      $<INSTALL_INTERFACE:include>)
  endif()
  target_link_libraries(${target} ${keyword} ${MPFR_LIBRARIES})
  if(WITH_GMPXX OR CGAL_WITH_GMPXX)
    target_include_directories(${target} SYSTEM ${keyword}  ${GMPXX_INCLUDE_DIR})
    target_link_libraries(${target}  ${keyword} ${GMPXX_LIBRARIES})
    target_compile_definitions(${target} ${keyword} CGAL_USE_GMPXX=1)
  endif()
endfunction()
