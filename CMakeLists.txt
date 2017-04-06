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
 
set(HDRS
	./PGT2/Game/src/Ball.h
	./PGT2/Game/src/Baller.h
	./PGT2/Game/src/BaseApplication.h
	./PGT2/Game/src/CustomCamera.h
	./PGT2/Game/src/CustomTimer.h
	./PGT2/Game/src/ForceGenerator.h
	./PGT2/Game/src/Helper.h
	./PGT2/Game/src/Highscore.h
	./PGT2/Game/src/InputManager.h
	./PGT2/Game/src/OgreText.h
	./PGT2/Game/src/RigidBody.h
	./PGT2/Game/src/UnitTestSample.h
	./PGT2/Game/src/World.h
)
 
set(SRCS
	./PGT2/Game/src/Ball.cpp
	./PGT2/Game/src/Baller.cpp
	./PGT2/Game/src/BaseApplication.cpp
	./PGT2/Game/src/CustomCamera.cpp
	./PGT2/Game/src/CustomTimer.cpp
	./PGT2/Game/src/ForceGenerator.cpp
	./PGT2/Game/src/Helper.cpp
	./PGT2/Game/src/Highscore.cpp
	./PGT2/Game/src/InputManager.cpp
	./PGT2/Game/src/RigidBody.cpp
	./PGT2/Game/src/UnitTestSample.cpp
	./PGT2/Game/src/World.cpp
)
 
include("tools/cmake/common.cmake") 
