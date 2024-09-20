# Distributed under the OSI-approved MIT License.  See accompanying
# file LICENSE or https://github.com/Crascit/DownloadProject for details.

cmake_minimum_required(VERSION 2.8.2)

project(tetgen-download NONE)

include(ExternalProject)
ExternalProject_Add(tetgen-download
                    TLS_VERIFY;OFF;GIT_CONFIG;advice.detachedHead=false;GIT_REPOSITORY;https://github.com/jdumas/tetgen.git;GIT_TAG;c63e7a6434652b8a2065c835bd9d6d298db1a0bc
                    SOURCE_DIR          "C:/dev/libigl/cmake/../external/tetgen"
                    BINARY_DIR          "C:/dev/libigl/build/tetgen-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
