cmake_minimum_required(VERSION 3.10)
project(AlignED)

# C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Looking for OpenCV(>=4.0)
find_package(OpenCV REQUIRED)


file(GLOB SRCS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

add_executable(AlignED ${SRCS})

# header files path
target_include_directories(AlignED PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# link OpenCV
target_link_libraries(AlignED ${OpenCV_LIBS})