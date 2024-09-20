# Distributed under the OSI-approved MIT License.  See accompanying
# file LICENSE or https://github.com/Crascit/DownloadProject for details.

cmake_minimum_required(VERSION 2.8.2)

project(imgui-download NONE)

include(ExternalProject)
ExternalProject_Add(imgui-download
                    TLS_VERIFY;OFF;GIT_CONFIG;advice.detachedHead=false;GIT_REPOSITORY;https://github.com/ocornut/imgui.git;GIT_TAG;v1.76
                    SOURCE_DIR          "C:/dev/libigl/cmake/../external/imgui"
                    BINARY_DIR          "C:/dev/libigl/build/imgui-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
