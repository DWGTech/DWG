# Distributed under the OSI-approved MIT License.  See accompanying
# file LICENSE or https://github.com/Crascit/DownloadProject for details.

cmake_minimum_required(VERSION 2.8.2)

project(libigl-imgui-download NONE)

include(ExternalProject)
ExternalProject_Add(libigl-imgui-download
                    TLS_VERIFY;OFF;GIT_CONFIG;advice.detachedHead=false;GIT_REPOSITORY;https://github.com/libigl/libigl-imgui.git;GIT_TAG;99f0643089b19f6daf5b3efd9544a65c9a851966
                    SOURCE_DIR          "C:/dev/libigl/cmake/../external/libigl-imgui"
                    BINARY_DIR          "C:/dev/libigl/build/libigl-imgui-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
