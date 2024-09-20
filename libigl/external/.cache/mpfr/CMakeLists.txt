# Distributed under the OSI-approved MIT License.  See accompanying
# file LICENSE or https://github.com/Crascit/DownloadProject for details.

cmake_minimum_required(VERSION 2.8.2)

project(mpfr-download NONE)

include(ExternalProject)
ExternalProject_Add(mpfr-download
                    TLS_VERIFY;OFF;GIT_CONFIG;advice.detachedHead=false;URL;https://cgal.geometryfactory.com/CGAL/precompiled_libs/auxiliary/x64/MPFR/3.0.0/mpfr-all-CGAL-3.9.zip;URL_MD5;48840454eef0ff18730050c05028734b
                    SOURCE_DIR          "C:/dev/libigl/cmake/../external/mpfr"
                    BINARY_DIR          "C:/dev/libigl/build/mpfr-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
