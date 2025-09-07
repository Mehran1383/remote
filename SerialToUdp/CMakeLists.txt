cmake_minimum_required(VERSION 3.14)
project(SerialToUdpApp VERSION 1.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable automatic handling of Qt6 resources, UI files, and moc files
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Find Qt6 components
find_package(Qt6 REQUIRED COMPONENTS Core SerialPort Network)

# Find ZeroMQ
#find_package(PkgConfig REQUIRED)

# Define the executable
add_executable(SerialToUdpApp
    src/main.cpp
    src/GpsSerialHandler.cpp
    src/ReceiverSerialHandler.cpp
    src/udpthreadmanager.cpp
    src/UdpWorker.cpp
    src/ntripclient.cpp
)

# Include the headers
target_include_directories(SerialToUdpApp PRIVATE ${CMAKE_SOURCE_DIR}/src)

# Link the Qt6 libraries to your executable
target_link_libraries(SerialToUdpApp
    PRIVATE
    Qt6::Core
    Qt6::SerialPort
    Qt6::Network
)

# Install the target
include(GNUInstallDirs)
install(TARGETS SerialToUdpApp
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
