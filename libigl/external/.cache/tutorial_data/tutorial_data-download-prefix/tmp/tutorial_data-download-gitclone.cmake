
if(NOT "C:/dev/libigl/external/.cache/tutorial_data/tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-gitinfo.txt" IS_NEWER_THAN "C:/dev/libigl/external/.cache/tutorial_data/tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-gitclone-lastrun.txt")
  message(STATUS "Avoiding repeated git clone, stamp file is up to date: 'C:/dev/libigl/external/.cache/tutorial_data/tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-gitclone-lastrun.txt'")
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "C:/dev/libigl/cmake/../external/../tutorial/data"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: 'C:/dev/libigl/cmake/../external/../tutorial/data'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "C:/Program Files/Git/mingw64/bin/git.exe" -c http.sslVerify=false clone --no-checkout --config "advice.detachedHead=false" "https://github.com/libigl/libigl-tutorial-data" "data"
    WORKING_DIRECTORY "C:/dev/libigl/cmake/../external/../tutorial"
    RESULT_VARIABLE error_code
    )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once:
          ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/libigl/libigl-tutorial-data'")
endif()

execute_process(
  COMMAND "C:/Program Files/Git/mingw64/bin/git.exe" -c http.sslVerify=false checkout 1f8fab844173890f1e326d93c9ca6c50dad1c6b2 --
  WORKING_DIRECTORY "C:/dev/libigl/cmake/../external/../tutorial/data"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: '1f8fab844173890f1e326d93c9ca6c50dad1c6b2'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "C:/Program Files/Git/mingw64/bin/git.exe" -c http.sslVerify=false submodule update --recursive --init 
    WORKING_DIRECTORY "C:/dev/libigl/cmake/../external/../tutorial/data"
    RESULT_VARIABLE error_code
    )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: 'C:/dev/libigl/cmake/../external/../tutorial/data'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy
    "C:/dev/libigl/external/.cache/tutorial_data/tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-gitinfo.txt"
    "C:/dev/libigl/external/.cache/tutorial_data/tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: 'C:/dev/libigl/external/.cache/tutorial_data/tutorial_data-download-prefix/src/tutorial_data-download-stamp/tutorial_data-download-gitclone-lastrun.txt'")
endif()

