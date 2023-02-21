CMAKE_MINIMUM_REQUIRED(VERSION 3.4.0)

PROJECT(fracture_reduction_planning)

CMAKE_POLICY(SET CMP0020 NEW)
CMAKE_POLICY(SET CMP0028 NEW)
CMAKE_POLICY(SET CMP0053 NEW)

SET(CMAKE_INCLUDE_CURRENT_DIR ON)

#---设置自动生成moc文件,不可缺少---------
SET(CMAKE_AUTOMOC ON)

#------------------------------------------
FIND_PACKAGE(Qt5Widgets)
FIND_PACKAGE(Qt5Core)
FIND_PACKAGE(Qt5Gui)
FIND_PACKAGE(Qt5OpenGL)
FIND_PACKAGE(Qt5Xml)


FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

#FIND_PACKAGE(PCL REQUIRED)
#INCLUDE(${PCL_USE_FILE})
#link_directories(${PCL_LIBRARY_DIRS})

#--------查找当前文件中的所有源码----------
INCLUDE_DIRECTORIES("eigen")
#INCLUDE_DIRECTORIES("UrAPI")
#INCLUDE_DIRECTORIES("NdiApi")


#INCLUDE_DIRECTORIES("libmodbus/windows64/includes")
#LINK_DIRECTORIES("libmodbus/windows64/lib")
#LINK_DIRECTORIES("NdiApi/bin/win64")

FILE(GLOB SRC_FILES "./*.cpp")
FILE(GLOB HEAD_FILES "./*.h")
FILE(GLOB UI_FILES "./*.ui")
#FILE(GLOB UR_SRC "./UrAPI/*.cpp")
#FILE(GLOB UR_HDR "./UrAPI/*.h")
#FILE(GLOB NDI_SRC "./NdiApi/src/*.cpp")
#FILE(GLOB NDI_HDR "./NdiApi/src/*.h")

#SET(PROJECT_RESOURCE fracture_reduction_planning.qrc)

#--------生成ui文件的头文件----------------
QT5_WRAP_UI(WRAP_FILES ${UI_FILES})
QT5_ADD_RESOURCES(PROJECT_RESOURCE_RCC ${PROJECT_RESOURCE})


#--------创建工程文件-----------------------
ADD_EXECUTABLE(${PROJECT_NAME}
               ${HEAD_FILES}
               ${SRC_FILES} 		   
               ${UI_FILES}
			#   ${UR_SRC}
			#  ${UR_HDR}
			#   ${NDI_SRC}
			#   ${NDI_HDR}
			   ${WRAP_FILES}
               ${WRAP_SRCS_H}
			   ${PROJECT_RESOURCE_RCC}
			   ${PROJECT_RESOURCE}
               )

TARGET_LINK_LIBRARIES(
                      ${PROJECT_NAME}
                      Qt5::Widgets
                      Qt5::Core
                      Qt5::Gui
                      Qt5::Xml
                      ${VTK_LIBRARIES}
                      ${ITK_LIBRARIES}
                      #${PCL_LIBRARIES}
                      ) 
					  
					  
#---------添加筛选器----------------				  
#source_group("UrAPI\\Source Files" FILES ${UR_SRC})
#source_group("UrAPI\\Header Files" FILES ${UR_HDR})
#source_group("NdiApi\\Source Files" FILES ${NDI_SRC})
#source_group("NdiApi\\Header Files" FILES ${NDI_HDR})
source_group("Forms" FILES ${UI_FILES})
source_group("Forms\\WPAP_UI" FILES ${WRAP_FILES})
source_group("Head Files" FILES ${HEAD_FILES})
