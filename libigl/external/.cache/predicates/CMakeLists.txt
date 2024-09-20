# Distributed under the OSI-approved MIT License.  See accompanying
# file LICENSE or https://github.com/Crascit/DownloadProject for details.

cmake_minimum_required(VERSION 2.8.2)

project(predicates-download NONE)

include(ExternalProject)
ExternalProject_Add(predicates-download
                    TLS_VERIFY;OFF;GIT_CONFIG;advice.detachedHead=false;GIT_REPOSITORY;https://github.com/libigl/libigl-predicates.git;GIT_TAG;488242fa2b1f98a9c5bd1441297fb4a99a6a9ae4
                    SOURCE_DIR          "C:/dev/libigl/cmake/../external/predicates"
                    BINARY_DIR          "C:/dev/libigl/build/predicates-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
