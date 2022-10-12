#     3a. This file is going to look for qpOASES, qpOASES-luamodel, qpOASES-urdfreader,
#         qpOASES-geometry, and qpOASES-muscle. Here we make a bunch of boolean flags
#         to indicate if these resources have been found and initialize them 
#         to false.

SET (qpOASES_FOUND FALSE)


#     3b. All of the variables that this FindqpOASES.cmake file will populate, 
#         if these resources exist, are listed here.

UNSET( qpOASES_INCLUDE_DIR              CACHE)   
UNSET( qpOASES_LIBRARY                  CACHE)    

#     3c. This script has two different modes:
#
#        If there is a CUSTOM_qpOASES_PATH: then this path is used to search 
#        for qpOASES
#
#        If there is no CUSTOM_qpOASES_PATH, then a bunch of typical install
#        locations are used. Note that at the present time these typical install
#        include paths that will only work on linux

IF(CUSTOM_qpOASES_PATH)


#     3d. The validity of each path is checked by using the FIND_PATH 
#         command to look for a specific file. In the case of the qpOASES_INCLUDE_DIR
#         it is the correct path if you can find qpOASES/qpOASES.h from it. 
#         If that is true then qpOASES_INCLUDE_DIR is assigned 
#         ${CUSTOM_qpOASES_PATH}/include (remember the $ converts the variable 
#         to its string representation)

  FIND_PATH (qpOASES_INCLUDE_DIR qpOASES/qpOASES.h
    PATHS 
    ${CUSTOM_qpOASES_PATH}/include 
    NO_DEFAULT_PATH
    )


#     3e. Similarly the validity of a path to a library is checked by looking
#         to see if a specific library exists using the FIND_LIBRARY command.
#         Note that you do not need to put the file type on the end of the 
#         library name, nor a prefix of 'lib': CMake will do this for you 
#         in a way that is cross-platform.

  FIND_LIBRARY (qpOASES_LIBRARY qpOASES
    PATHS
    ${CUSTOM_qpOASES_PATH}/lib
    NO_DEFAULT_PATH
    )

ELSE(CUSTOM_qpOASES_PATH)


#     3f. If there is no CUSTOM_qpOASES_PATH given then FIND_PATH and FIND_LIBRARY
#         commands are used but with substantial HINTS, or places to look

  FIND_PATH (qpOASES_INCLUDE_DIR qpOASES.hpp
    HINTS
    $ENV{HOME}/local/include
    $ENV{qpOASES_PATH}/src
    $ENV{qpOASES_PATH}/include
    $ENV{qpOASES_INCLUDE_PATH}
    /usr/local/include
    /usr/include
    )

  FIND_LIBRARY (qpOASES_LIBRARY qpOASES
    PATHS
    $ENV{HOME}/local/lib
    $ENV{HOME}/local/lib/x86_64-linux-gnu
    $ENV{qpOASES_PATH}/lib
    $ENV{qpOASES_LIBRARY_PATH}
    /usr/local/lib
    /usr/local/lib/x86_64-linux-gnu
    /usr/lib
    /usr/lib/x86_64-linux-gnu
    )
ENDIF(CUSTOM_qpOASES_PATH)


#     3g. If we've gotten to this point then either all include directories 
#         and libraries have been found, some have been found, or none have 
#         been found. All of the code below is going through what the user
#         asked for, seeing if it was found, and if not issuing an error.


IF (qpOASES_INCLUDE_DIR AND qpOASES_LIBRARY)
  SET (qpOASES_FOUND TRUE)
ELSE(qpOASES_INCLUDE_DIR AND qpOASES_LIBRARY)
  IF(qpOASES_FIND_REQUIRED)
    MESSAGE (SEND_ERROR " Could not find qpOASES.")
    MESSAGE (SEND_ERROR " Try setting CUSTOM_qpOASES_PATH in FindqpOASES.cmake force CMake to use the desired directory.")
  ELSE(qpOASES_FIND_REQUIRED)
    MESSAGE (STATUS " Could not find qpOASES.")
    MESSAGE (STATUS " Try setting CUSTOM_qpOASES_PATH in FindqpOASES.cmake force CMake to use the desired directory.")
  ENDIF(qpOASES_FIND_REQUIRED)
ENDIF (qpOASES_INCLUDE_DIR AND qpOASES_LIBRARY)


IF (qpOASES_FOUND)
   IF (NOT qpOASES_FIND_QUIETLY)
      MESSAGE(STATUS "Found qpOASES: ${qpOASES_LIBRARY}")
   ENDIF (NOT qpOASES_FIND_QUIETLY)

   foreach ( COMPONENT ${qpOASES_FIND_COMPONENTS} )
     IF (qpOASES_${COMPONENT}_FOUND)
       IF (NOT qpOASES_FIND_QUIETLY)
         MESSAGE(STATUS "Found qpOASES ${COMPONENT}: ${qpOASES_${COMPONENT}_LIBRARY}")
       ENDIF (NOT qpOASES_FIND_QUIETLY)
     ELSE (qpOASES_${COMPONENT}_FOUND)
       MESSAGE(ERROR " Could not find qpOASES ${COMPONENT}")
     ENDIF (qpOASES_${COMPONENT}_FOUND)
   endforeach ( COMPONENT )

ENDIF (qpOASES_FOUND)


#     3h. Here all of the specific paths and libraries are marked as advanced
#         which means that they will not appear in the CMake gui unless the 
#         user toggles to the advanced mode.                

MARK_AS_ADVANCED (
  qpOASES_INCLUDE_DIR
  qpOASES_LIBRARY
  )
