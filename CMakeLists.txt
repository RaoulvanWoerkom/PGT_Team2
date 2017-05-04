#/*
#-----------------------------------------------------------------------------
#Filename:    CMakeLists.txt
#-----------------------------------------------------------------------------
#
#This source file is part of the
#   ___                 __    __ _ _    _ 
#  /___\__ _ _ __ ___  / / /\ \ (_) | _(_)
# //  // _` | '__/ _ \ \ \/  \/ / | |/ / |
#/ \_// (_| | | |  __/  \  /\  /| |   <| |
#\___/ \__, |_|  \___|   \/  \/ |_|_|\_\_|
#      |___/                              
#      Tutorial Framework
#      http://www.ogre3d.org/tikiwiki/
#-----------------------------------------------------------------------------
#*/
cmake_minimum_required(VERSION 2.6)

set(APP "OgreApp")

file(GLOB HDRS ./PGT2/Game/src/*.h)

file(GLOB SRCS ./PGT2/Game/src/*.cpp)

file(GLOB HDRS ./PGT2/UnitTest2/*.h)

file(GLOB HDRS ./PGT2/UnitTest2/*.cpp)
 
include("tools/cmake/common.cmake") 
