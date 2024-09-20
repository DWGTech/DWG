# Distributed under the OSI-approved MIT License.  See accompanying
# file LICENSE or https://github.com/Crascit/DownloadProject for details.

cmake_minimum_required(VERSION 2.8.2)

project(CoMISo-download NONE)

include(ExternalProject)
ExternalProject_Add(CoMISo-download
                    TLS_VERIFY;OFF;GIT_CONFIG;advice.detachedHead=false;GIT_REPOSITORY;https://github.com/libigl/CoMISo.git;GIT_TAG;d60aa4759fba76b0b793b1efb090b7a771dd7c56
                    SOURCE_DIR          "C:/dev/libigl/cmake/../external/CoMISo"
                    BINARY_DIR          "C:/dev/libigl/build/CoMISo-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
)
