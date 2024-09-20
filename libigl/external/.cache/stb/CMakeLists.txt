# Distributed under the OSI-approved MIT License.  See accompanying
# file LICENSE or https://github.com/Crascit/DownloadProject for details.

cmake_minimum_required(VERSION 2.8.2)

project(stb-download NONE)

include(ExternalProject)
ExternalProject_Add(stb-download
                    TLS_VERIFY;OFF;GIT_CONFIG;advice.detachedHead=false;GIT_REPOSITORY;https://github.com/libigl/libigl-stb.git;GIT_TAG;cd0fa3fcd90325c83be4d697b00214e029f94ca3
                    SOURCE_DIR          "C:/dev/libigl/cmake/../external/stb"
                    BINARY_DIR          "C:/dev/libigl/build/stb-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
