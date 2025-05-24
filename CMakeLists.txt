cmake_minimum_required(VERSION 3.26 FATAL_ERROR)
project(bitbot_encos)

set(BITBOT_DEPENDENCY_USE_PROXY OFF)
set(BITBOT_DEPENDENCY_USE_LOCAL_FILE TRUE)
set(BITBOT_DEPENDENCY_LOCAL_FILE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/dependencies)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(bitbot.cmake)

option(BUILD_DOC "Build documentation" OFF)
option(BUILD_SIMULATION "Build simulation" ON)
add_subdirectory(doc)

if(BUILD_SIMULATION)
    #add_definitions(-DBUILD_SIMULATION)
    set(BitbotEncosDefinitions "-DBUILD_SIMULATION" PARENT_SCOPE)
    BitbotAddDependency(
        NAME bitbot_mujoco
        FILE_NAME "bitbot_mujoco-main.zip"
        FILE_PATH ${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}
        USE_LOCAL_FILE TRUE
    )
    set(BitbotEncos_LIBS BitbotMujoco PARENT_SCOPE)
else()
    set(BitbotEncosDefinitions "" PARENT_SCOPE)
    BitbotAddDependency(
        NAME bitbot_kernel
        FILE_NAME "bitbot_kernel-main.zip"
        FILE_PATH ${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}
        USE_LOCAL_FILE TRUE
    )

    aux_source_directory(src/bus SRC_BUS)
    aux_source_directory(src/device SRC_DEVICE)
    aux_source_directory(src/kernel SRC_KERNEL)
    aux_source_directory(lib/yesense_sdk YESENSE_SDK)

    add_subdirectory(lib/soem)

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
    target_compile_options(BitbotEncos PUBLIC)
    target_compile_features(BitbotEncos PUBLIC cxx_std_20)
    set(BitbotEncos_LIBS BitbotEncos BitbotKernel PARENT_SCOPE)
endif()


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

