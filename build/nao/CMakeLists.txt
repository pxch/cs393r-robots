cmake_minimum_required(VERSION 2.8)

project(naointerface)

include(../common.cmake)

file(GLOB_RECURSE SRCS 
  ${INTERFACE_DIR}/nao/*.cpp
  ${INTERFACE_DIR}/nao/*.h
)

qi_create_lib(naointerface SHARED ${SRCS})
qi_stage_lib(naointerface)

qi_use_lib(naointerface core ALCOMMON ALMEMORYFASTACCESS)
#target_link_libraries(naointerface ${LINK_LIBS})
