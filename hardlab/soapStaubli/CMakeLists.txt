
cmake_minimum_required(VERSION 2.6)

#common commands for building c++ executables and libraries
INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/include )
# ADD_LIBRARY(soapC src/soapC.cpp)
# ADD_LIBRARY(Server0 src/soapCS8ServerV0Proxy.cpp)
# ADD_LIBRARY(Server1 src/soapCS8ServerV1Proxy.cpp)
# ADD_LIBRARY(Server3 src/soapCS8ServerV3Proxy.cpp)
# ADD_LIBRARY(stdsoap2 src/stdsoap2.cpp)
# ADD_LIBRARY(tx60l src/TX90.cpp)

ADD_LIBRARY(soapC src/soapC.cpp src/soapCS8ServerV0Proxy.cpp
            src/soapCS8ServerV1Proxy.cpp
            src/soapCS8ServerV3Proxy.cpp
            src/stdsoap2.cpp
            src/TX90.cpp)

ADD_EXECUTABLE(staubli_controller src/controller.cpp )
target_link_libraries(staubli_controller soapC)
