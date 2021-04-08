cmake_minimum_required(VERSION 3.5)

project(imuRead)

############################################################
# Create a library
############################################################

#Generate the static library from the library sources
# add_library(hello_library STATIC 
#     src/Hello.cpp
# )

# include_directories(
#   inc/serial/include
#   )


# add_subdirectory(inc/serial)
set(SOURCES
src/main.c
src/com.cpp
src/atomprotocol.cpp
)

############################################################
# Create an executable
############################################################

# Add an executable with the above sources
# add_executable(imuRead ${SOURCES}
# )

# # link the new hello_library target with the hello_binary target
# target_link_libraries( imuRead
# pthread

# )


# add_executable(imuRead2 src/test.cpp 
# src/imu.cpp

# )

# # link the new hello_library target with the hello_binary target
# target_link_libraries( imuRead2
# pthread
# serial
# )


add_executable(imuClass src/testclass.cpp 
src/imu.cpp
src/com.cpp
src/atomprotocol.cpp
)

# link the new hello_library target with the hello_binary target
target_link_libraries( imuClass
pthread
)