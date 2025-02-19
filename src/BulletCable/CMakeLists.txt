
INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src

)

#SUBDIRS( Solvers )

SET(BulletCable_SRCS
	btCable.cpp
)


SET(BulletCable_HDRS
	btCable.h
)


ADD_LIBRARY(BulletCable  ${BulletCable_SRCS} ${BulletCable_HDRS})
SET_TARGET_PROPERTIES(BulletCable PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(BulletCable PROPERTIES SOVERSION ${BULLET_VERSION})
IF (BUILD_SHARED_LIBS)
	TARGET_LINK_LIBRARIES(BulletSoftBody BulletDynamics)
ENDIF (BUILD_SHARED_LIBS)

IF (INSTALL_LIBS)
	IF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
		IF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.5)
			IF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
				INSTALL(TARGETS BulletCable DESTINATION .)
			ELSE (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
			INSTALL(TARGETS BulletCable RUNTIME DESTINATION bin
							LIBRARY DESTINATION lib${LIB_SUFFIX}
							ARCHIVE DESTINATION lib${LIB_SUFFIX})
				INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
DESTINATION ${INCLUDE_INSTALL_DIR} FILES_MATCHING PATTERN "*.h"  PATTERN
".svn" EXCLUDE PATTERN "CMakeFiles" EXCLUDE)
			ENDIF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
		ENDIF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.5)

		IF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
			SET_TARGET_PROPERTIES(BulletCable PROPERTIES FRAMEWORK true)
			SET_TARGET_PROPERTIES(BulletCable PROPERTIES PUBLIC_HEADER "${BulletCable_HDRS}")
		ENDIF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
	ENDIF (NOT INTERNAL_CREATE_DISTRIBUTABLE_MSVC_PROJECTFILES)
ENDIF (INSTALL_LIBS)
