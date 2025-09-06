cmake_minimum_required (VERSION 3.14)

project (Str1QRWasm)

include (${CMAKE_CURRENT_SOURCE_DIR}/../../zxing.cmake)
zxing_add_package_stb()

set (CMAKE_CXX_STANDARD 20)

option (ZXING_WRITERS "Build with writer support (encoders)" ON)
option (ZXING_READERS "Build with reader support (decoders)" ON)
set    (ZXING_EMSCRIPTEN_ENVIRONMENT "web" CACHE STRING "Optimize build for given emscripten runtime environment (web/node/shell/worker)")

# provide backward compatibility for deprecated BUILD_... options
if (DEFINED BUILD_READERS)
    set (ZXING_READERS ${BUILD_READERS})
endif()
if (DEFINED BUILD_WRITERS)
    set (ZXING_WRITERS ${BUILD_WRITERS})
endif()
if (DEFINED BUILD_EMSCRIPTEN_ENVIRONMENT)
    set (ZXING_EMSCRIPTEN_ENVIRONMENT ${BUILD_EMSCRIPTEN_ENVIRONMENT})
endif()

if (NOT CMAKE_BUILD_TYPE)
    set (CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
endif()

add_definitions ("-s DISABLE_EXCEPTION_CATCHING=0")

add_subdirectory (${CMAKE_CURRENT_SOURCE_DIR}/../../core ${CMAKE_BINARY_DIR}/ZXing)

set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --bind -s ENVIRONMENT=${ZXING_EMSCRIPTEN_ENVIRONMENT} -s DISABLE_EXCEPTION_CATCHING=0 -s FILESYSTEM=0 -s MODULARIZE=1 -s EXPORT_NAME=Str1QR -s EXPORTED_FUNCTIONS=\"['_malloc', '_free']\" -s EXPORTED_RUNTIME_METHODS=HEAPU8 -s ALLOW_MEMORY_GROWTH=1")

if (ZXING_READERS AND ZXING_WRITERS)
    add_executable (str1qr BarcodeReader.cpp BarcodeWriter.cpp)
    target_link_libraries (str1qr ZXing::ZXing stb::stb)
endif()

if (ZXING_READERS)
    configure_file(demo_cam_reader.html demo_cam_reader.html COPYONLY)
    configure_file(demo_reader.html demo_reader.html COPYONLY)
    add_executable (str1qr_reader BarcodeReader.cpp)
    target_link_libraries (str1qr_reader ZXing::ZXing stb::stb)
endif()

if (ZXING_WRITERS)
    configure_file(demo_writer.html demo_writer.html COPYONLY)
    add_executable (str1qr_writer BarcodeWriter.cpp )
    target_link_libraries (str1qr_writer ZXing::ZXing stb::stb)
endif()

