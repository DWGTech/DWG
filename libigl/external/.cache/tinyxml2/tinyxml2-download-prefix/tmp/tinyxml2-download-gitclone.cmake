
if(NOT "C:/dev/libigl/external/.cache/tinyxml2/tinyxml2-download-prefix/src/tinyxml2-download-stamp/tinyxml2-download-gitinfo.txt" IS_NEWER_THAN "C:/dev/libigl/external/.cache/tinyxml2/tinyxml2-download-prefix/src/tinyxml2-download-stamp/tinyxml2-download-gitclone-lastrun.txt")
  message(STATUS "Avoiding repeated git clone, stamp file is up to date: 'C:/dev/libigl/external/.cache/tinyxml2/tinyxml2-download-prefix/src/tinyxml2-download-stamp/tinyxml2-download-gitclone-lastrun.txt'")
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "C:/dev/libigl/cmake/../external/tinyxml2"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: 'C:/dev/libigl/cmake/../external/tinyxml2'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "C:/Program Files/Git/mingw64/bin/git.exe" -c http.sslVerify=false clone --no-checkout --config "advice.detachedHead=false" "https://github.com/leethomason/tinyxml2.git" "tinyxml2"
    WORKING_DIRECTORY "C:/dev/libigl/cmake/../external"
    RESULT_VARIABLE error_code
    )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once:
          ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/leethomason/tinyxml2.git'")
endif()

execute_process(
  COMMAND "C:/Program Files/Git/mingw64/bin/git.exe" -c http.sslVerify=false checkout d175e9de0be0d4db75d0a8cf065599a435a87eb6 --
  WORKING_DIRECTORY "C:/dev/libigl/cmake/../external/tinyxml2"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: 'd175e9de0be0d4db75d0a8cf065599a435a87eb6'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "C:/Program Files/Git/mingw64/bin/git.exe" -c http.sslVerify=false submodule update --recursive --init 
    WORKING_DIRECTORY "C:/dev/libigl/cmake/../external/tinyxml2"
    RESULT_VARIABLE error_code
    )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: 'C:/dev/libigl/cmake/../external/tinyxml2'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy
    "C:/dev/libigl/external/.cache/tinyxml2/tinyxml2-download-prefix/src/tinyxml2-download-stamp/tinyxml2-download-gitinfo.txt"
    "C:/dev/libigl/external/.cache/tinyxml2/tinyxml2-download-prefix/src/tinyxml2-download-stamp/tinyxml2-download-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: 'C:/dev/libigl/external/.cache/tinyxml2/tinyxml2-download-prefix/src/tinyxml2-download-stamp/tinyxml2-download-gitclone-lastrun.txt'")
endif()

