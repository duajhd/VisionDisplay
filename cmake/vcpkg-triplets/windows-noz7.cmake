if(DEFINED Z_VCPKG_ROOT_DIR)
    set(_vision_vcpkg_root "${Z_VCPKG_ROOT_DIR}")
elseif(DEFINED _VCPKG_ROOT_DIR)
    set(_vision_vcpkg_root "${_VCPKG_ROOT_DIR}")
else()
    set(_vision_vcpkg_root "D:/vcpkg/vcpkg-master/vcpkg-master")
endif()

if(DEFINED ENV{VCToolsInstallDir})
    string(STRIP "$ENV{VCToolsInstallDir}" _vision_vctools_dir)
    file(TO_CMAKE_PATH "${_vision_vctools_dir}/bin/Hostx64/x64/cl.exe" _vision_cl_path)
else()
    set(_vision_vctools_dir "D:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/MSVC/14.43.34808")
    set(_vision_cl_path "D:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/MSVC/14.43.34808/bin/Hostx64/x64/cl.exe")
endif()

set(CMAKE_C_COMPILER "${_vision_cl_path}" CACHE FILEPATH "" FORCE)
set(CMAKE_CXX_COMPILER "${_vision_cl_path}" CACHE FILEPATH "" FORCE)

if(DEFINED ENV{WindowsSdkDir} AND DEFINED ENV{WindowsSDKVersion})
    string(STRIP "$ENV{WindowsSdkDir}" _vision_windows_sdk_dir)
    string(STRIP "$ENV{WindowsSDKVersion}" _vision_windows_sdk_version)
    file(TO_CMAKE_PATH "${_vision_windows_sdk_dir}/bin/${_vision_windows_sdk_version}/x64/rc.exe" _vision_rc_path)
    file(TO_CMAKE_PATH "${_vision_windows_sdk_dir}/bin/${_vision_windows_sdk_version}/x64/mt.exe" _vision_mt_path)
else()
    set(_vision_windows_sdk_dir "D:/Windows Kits/10")
    set(_vision_windows_sdk_version "10.0.26100.0")
    set(_vision_rc_path "D:/Windows Kits/10/bin/10.0.26100.0/x64/rc.exe")
    set(_vision_mt_path "D:/Windows Kits/10/bin/10.0.26100.0/x64/mt.exe")
endif()

set(CMAKE_RC_COMPILER "${_vision_rc_path}" CACHE FILEPATH "" FORCE)
set(CMAKE_MT "${_vision_mt_path}" CACHE FILEPATH "" FORCE)

include("${_vision_vcpkg_root}/scripts/toolchains/windows.cmake")

set(_vision_linker_libpaths
    "/LIBPATH:\"${_vision_vctools_dir}/lib/x64\""
    "/LIBPATH:\"${_vision_windows_sdk_dir}/lib/${_vision_windows_sdk_version}/ucrt/x64\""
    "/LIBPATH:\"${_vision_windows_sdk_dir}/lib/${_vision_windows_sdk_version}/um/x64\"")
string(JOIN " " _vision_linker_libpath_flags ${_vision_linker_libpaths})

set(_vision_include_paths
    "/I\"${_vision_vctools_dir}/include\""
    "/I\"${_vision_windows_sdk_dir}/Include/${_vision_windows_sdk_version}/ucrt\""
    "/I\"${_vision_windows_sdk_dir}/Include/${_vision_windows_sdk_version}/um\""
    "/I\"${_vision_windows_sdk_dir}/Include/${_vision_windows_sdk_version}/shared\""
    "/I\"${_vision_windows_sdk_dir}/Include/${_vision_windows_sdk_version}/winrt\"")
string(JOIN " " _vision_include_flags ${_vision_include_paths})

foreach(_vision_compile_flags_var CMAKE_C_FLAGS CMAKE_CXX_FLAGS)
    set(${_vision_compile_flags_var} "${${_vision_compile_flags_var}} ${_vision_include_flags}" CACHE STRING "" FORCE)
endforeach()

set(CMAKE_RC_FLAGS "${CMAKE_RC_FLAGS} ${_vision_include_flags}" CACHE STRING "" FORCE)

foreach(_vision_linker_flags_var
        CMAKE_EXE_LINKER_FLAGS
        CMAKE_SHARED_LINKER_FLAGS
        CMAKE_MODULE_LINKER_FLAGS
        CMAKE_EXE_LINKER_FLAGS_DEBUG
        CMAKE_SHARED_LINKER_FLAGS_DEBUG
        CMAKE_MODULE_LINKER_FLAGS_DEBUG
        CMAKE_EXE_LINKER_FLAGS_RELEASE
        CMAKE_SHARED_LINKER_FLAGS_RELEASE
        CMAKE_MODULE_LINKER_FLAGS_RELEASE)
    set(${_vision_linker_flags_var} "${${_vision_linker_flags_var}} ${_vision_linker_libpath_flags}" CACHE STRING "" FORCE)
endforeach()

foreach(_vision_flags_var
        CMAKE_C_FLAGS_DEBUG
        CMAKE_CXX_FLAGS_DEBUG
        CMAKE_C_FLAGS_RELEASE
        CMAKE_CXX_FLAGS_RELEASE)
    if(DEFINED ${_vision_flags_var})
        string(REPLACE " /Z7" "" _vision_flags_value "${${_vision_flags_var}}")
        string(REPLACE "/Z7 " "" _vision_flags_value "${_vision_flags_value}")
        string(REPLACE "/Z7" "" _vision_flags_value "${_vision_flags_value}")
        set(${_vision_flags_var} "${_vision_flags_value}" CACHE STRING "" FORCE)
    endif()
endforeach()
