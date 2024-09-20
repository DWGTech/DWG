# Distributed under the OSI-approved MIT License.  See accompanying
# file LICENSE or https://github.com/Crascit/DownloadProject for details.

cmake_minimum_required(VERSION 2.8.2)

project(triangle-download NONE)

include(ExternalProject)
ExternalProject_Add(triangle-download
                    TLS_VERIFY;OFF;GIT_CONFIG;advice.detachedHead=false;GIT_REPOSITORY;https://github.com/libigl/triangle.git;GIT_TAG;5a70326574b34d6a51d9eaf6a9f78813657ee108
                    SOURCE_DIR          "C:/dev/libigl/cmake/../external/triangle"
                    BINARY_DIR          "C:/dev/libigl/build/triangle-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
