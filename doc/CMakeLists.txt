cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

project(doc_doxygen)

find_package(Doxygen)

if (DOXYGEN_FOUND)
    message("Doxygen build started")

    set(DOXYGEN_PROJECT_NAME "BitbotEncos")
    set(DOXYGEN_PROJECT_BRIEF "Bitbot Encos is a bitbot instance for encos motor.")
    set(DOXYGEN_IMAGE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
    set(DOXYGEN_HTML_EXTRA_STYLESHEET ${CMAKE_CURRENT_SOURCE_DIR}/doxygen-awesome-css-main/doxygen-awesome.css)
    set(DOXYGEN_HAVE_DOT YES)
    set(DOXYGEN_INHERIT_GRAPH YES)
    set(DOXYGEN_GENERATE_TREEVIEW YES)
    set(DOXYGEN_GENERATE_HTML YES)
    set(DOXYGEN_DISABLE_INDEX NO)
    set(DOXYGEN_MARKDOWN_SUPPORT YES)
    set(DOXYGEN_USE_MATHJAX = YES)
    set(DOXYGEN_OUTPUT_LANGUAGE "Chinese")
    
    if(BUILD_DOC)
        doxygen_add_docs(
        ${PROJECT_NAME}
        ../include
        ./BitbotEncosBusConfig.md
        ./BitbotEncosConfig.md
        ./BitbotEncosMotorMotion.md
        ../README.md
        )
    endif(BUILD_DOC)

else (DOXYGEN_FOUND)
  message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)
