# Distributed under the OSI-approved MIT License.  See accompanying
# file LICENSE or https://github.com/Crascit/DownloadProject for details.

cmake_minimum_required(VERSION 2.8.2)

project(glad-download NONE)

include(ExternalProject)
ExternalProject_Add(glad-download
                    TLS_VERIFY;OFF;GIT_CONFIG;advice.detachedHead=false;GIT_REPOSITORY;https://github.com/libigl/libigl-glad.git;GIT_TAG;09b4969c56779f7ddf8e6176ec1873184aec890f
                    SOURCE_DIR          "C:/dev/libigl/cmake/../external/glad"
                    BINARY_DIR          "C:/dev/libigl/build/glad-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
