cmake_minimum_required(VERSION 3.14 FATAL_ERROR)
project(bitbot_encos)


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_DOC "Build documentation" OFF)

aux_source_directory(src/bus SRC_BUS)
aux_source_directory(src/device SRC_DEVICE)
aux_source_directory(src/kernel SRC_KERNEL)
aux_source_directory(lib/yesense_sdk YESENSE_SDK)

add_subdirectory(lib/soem)
add_subdirectory(doc)

message("
Bitbot Encos is a bitbot instance for encos motor.")
message("Visit http://opensource.zzshub.cn/libBitbotEncos for detailed documentation.")

message("+-------------------------------------------------------------------------------+
| BBBB   III  TTTTT  BBBB    OOO   TTTTT     EEEEE  N   N   CCCC   OOO    SSSS  |
| B   B   I     T    B   B  O   O    T       E      NN  N  C      O   O  S      |
| BBBB    I     T    BBBB   O   O    T       EEEEE  N N N  C      O   O   SSS   |
| B   B   I     T    B   B  O   O    T       E      N  NN  C      O   O       S |
| BBBB   III    T    BBBB    OOO     T       EEEEE  N   N   CCCC   OOO    SSSS  |
+-------------------------------------------------------------------------------+
")

LINK_LIBRARIES(m)

add_library(BitbotEncos STATIC
            ./include/kernel/Encos_kernel.hpp
            ${SRC_BUS}
            ${SRC_KERNEL}
            ${SRC_DEVICE}
            ${YESENSE_SDK}
)

target_include_directories(BitbotEncos PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/)
target_include_directories(BitbotEncos PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib)

target_include_directories(BitbotEncos PUBLIC
   ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(BitbotEncos
    BitbotKernel
    soem
)