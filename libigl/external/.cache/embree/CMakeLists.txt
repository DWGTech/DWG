# Distributed under the OSI-approved MIT License.  See accompanying
# file LICENSE or https://github.com/Crascit/DownloadProject for details.

cmake_minimum_required(VERSION 2.8.2)

project(embree-download NONE)

include(ExternalProject)
ExternalProject_Add(embree-download
                    TLS_VERIFY;OFF;GIT_CONFIG;advice.detachedHead=false;GIT_REPOSITORY;https://github.com/embree/embree.git;GIT_TAG;v3.5.2
                    SOURCE_DIR          "C:/dev/libigl/cmake/../external/embree"
                    BINARY_DIR          "C:/dev/libigl/build/embree-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
