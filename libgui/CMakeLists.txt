# This CMakeFile is used con create libgui(.lib / .a )

FILE( GLOB FORMS_UI *.ui )
QT4_WRAP_UI( QTUI_H_SRC ${FORMS_UI} )

FILE( GLOB QT_MOC_HDR *.h )
# Assegurar-se que no es prenen els fitxers *.h~
# message("---------------------------------\n" ${QT_MOC_HDR})

QT4_WRAP_CPP( QT_MOC_SRC ${QT_MOC_HDR} )

FILE( GLOB SRC_FILES *.cpp )
FILE( GLOB HDR_FILES *.h )
FILE( GLOB RES_FILES *.qrc )

ADD_DEFINITIONS( -DSOQT_DLL -DCOIN_DLL )

INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/..
                     ${mt_DIR}
                     ${SOQT_INCLUDE_DIR} 
                     ${QT_INCLUDE_DIR})

QT4_ADD_RESOURCES( CPP_RES_FILES ${RES_FILES} )

ADD_LIBRARY( libgui ${CPP_RES_FILES} ${SRC_FILES} ${HDR_FILES} 
                    ${QT_MOC_SRC} ${QTUI_H_SRC} )

#TARGET_LINK_LIBRARIES( libgui ${QT_LIBRARIES} libdevice libproblem libplanner 
#                       libsampling libkthutil libguiding ${SOQT_LIBRARY} ${COIN_LIBRARY} )

TARGET_LINK_LIBRARIES( libgui ${QT_LIBRARIES} libproblem libplanner 
                       libsampling libkthutil ${SOQT_LIBRARY} ${COIN_LIBRARY} )

IF(UNIX)
  SET_TARGET_PROPERTIES( libgui PROPERTIES OUTPUT_NAME gui )
ENDIF(UNIX)

###########
# Install #
###########
SET(INSTALL_DIR_RAW ${CMAKE_INSTALL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION})
install(TARGETS libgui
  ARCHIVE DESTINATION ${INSTALL_DIR_RAW}
  LIBRARY DESTINATION ${INSTALL_DIR_RAW}
  RUNTIME DESTINATION ${INSTALL_DIR_RAW}
)