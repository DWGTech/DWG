# Distributed under the OSI-approved MIT License.  See accompanying
# file LICENSE or https://github.com/Crascit/DownloadProject for details.

cmake_minimum_required(VERSION 2.8.2)

project(tutorial_data-download NONE)

include(ExternalProject)
ExternalProject_Add(tutorial_data-download
                    TLS_VERIFY;OFF;GIT_CONFIG;advice.detachedHead=false;GIT_REPOSITORY;https://github.com/libigl/libigl-tutorial-data;GIT_TAG;1f8fab844173890f1e326d93c9ca6c50dad1c6b2
                    SOURCE_DIR          "C:/dev/libigl/cmake/../external/../tutorial/data"
                    BINARY_DIR          "C:/dev/libigl/tutorial/build/tutorial_data-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
