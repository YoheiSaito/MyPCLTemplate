# cmakeの要求バージョン
cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

# 実行ファイル名
project(main)

# コンパイラの指定
# set(CMAKE_CXX_COMPILER ccache)
# set(CMAKE_CXX_COMPILER_ARG1 clang++)
set(CMAKE_CXX_COMPILER clang++)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS -Qunused-arguments)
# set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# PCLの指定
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# BOOSTの指定
find_package(Boost COMPONENTS timer REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

# ソースの指定
add_executable(${PROJECT_NAME}
	${PROJECT_NAME}.cpp
)

# リンクするライブラリの指定
target_link_libraries(${PROJECT_NAME}
	${PCL_LIBRARIES}
	${Boost_LIBRARIES}
)

