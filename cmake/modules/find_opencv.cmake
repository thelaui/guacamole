##############################################################################
# search paths
##############################################################################
SET(OPENCV_INCLUDE_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/include/OpenCV/include
  ${OPENCV_INCLUDE_SEARCH_DIR}
  /opt/OpenCV/current/include
  /usr/include/
)

SET(OPENCV_LIBRARY_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/lib
  ${OPENCV_LIBRARY_SEARCH_DIR}
  ../
  /opt/OpenCV/current/lib/
  /usr/lib/x86_64-linux-gnu/
)

##############################################################################
# feedback to provide user-defined paths to search for OpenCV
##############################################################################
MACRO (request_opencv_search_directories)

    IF ( NOT OPENCV_INCLUDE_DIRS AND NOT OPENCV_LIBRARY_DIRS )
        SET(OPENCV_INCLUDE_SEARCH_DIR "Please provide OpenCV include path." CACHE PATH "path to OpenCV headers.")
        SET(OPENCV_LIBRARY_SEARCH_DIR "Please provide OpenCV library path." CACHE PATH "path to OpenCV libraries.")
        MESSAGE(FATAL_ERROR "find_OpenCV.cmake: unable to find OpenCV.")
    ENDIF ( NOT OPENCV_INCLUDE_DIRS AND NOT OPENCV_LIBRARY_DIRS )

    IF ( NOT OPENCV_INCLUDE_DIRS )
        SET(OPENCV_INCLUDE_SEARCH_DIR "Please provide OpenCV include path." CACHE PATH "path to OpenCV headers.")
        MESSAGE(FATAL_ERROR "find_OpenCV.cmake: unable to find OpenCV headers.")
    ELSE ( NOT OPENCV_INCLUDE_DIRS )
        UNSET(OPENCV_INCLUDE_SEARCH_DIR CACHE)
    ENDIF ( NOT OPENCV_INCLUDE_DIRS )

    IF ( NOT OPENCV_LIBRARY_DIRS )
        SET(OPENCV_LIBRARY_SEARCH_DIR "Please provide OpenCV library path." CACHE PATH "path to OpenCV libraries.")
        MESSAGE(FATAL_ERROR "find_OpenCV.cmake: unable to find OpenCV libraries.")
    ELSE ( NOT OPENCV_LIBRARY_DIRS )
        UNSET(OPENCV_LIBRARY_SEARCH_DIR CACHE)
    ENDIF ( NOT OPENCV_LIBRARY_DIRS )

ENDMACRO (request_opencv_search_directories)

##############################################################################
# check for OpenCV
##############################################################################
message(STATUS "-- checking for OpenCV")

IF ( NOT OPENCV_INCLUDE_DIRS )

    FOREACH(_SEARCH_DIR ${OPENCV_INCLUDE_SEARCH_DIRS})
        FIND_PATH(_CUR_SEARCH
                NAMES opencv2/opencv.hpp
                PATHS ${_SEARCH_DIR}
                NO_DEFAULT_PATH)
        IF (_CUR_SEARCH)
            LIST(APPEND _OPENCV_FOUND_INC_DIRS ${_CUR_SEARCH})
        ENDIF(_CUR_SEARCH)
        SET(_CUR_SEARCH _CUR_SEARCH-NOTFOUND CACHE INTERNAL "internal use")
    ENDFOREACH(_SEARCH_DIR ${OPENCV_INCLUDE_SEARCH_DIRS})

    IF (NOT _OPENCV_FOUND_INC_DIRS)
        request_opencv_search_directories()
    ENDIF (NOT _OPENCV_FOUND_INC_DIRS)

    FOREACH(_INC_DIR ${_OPENCV_FOUND_INC_DIRS})
        LIST(APPEND _OPENCV_INCLUDE_DIRS ${_INC_DIR})
    ENDFOREACH(_INC_DIR ${_BOOST_FOUND_INC_DIRS})

    IF (_OPENCV_FOUND_INC_DIRS)
        SET(OPENCV_INCLUDE_DIRS ${_OPENCV_INCLUDE_DIRS} CACHE PATH "path to OpenCV headers.")
    ENDIF (_OPENCV_FOUND_INC_DIRS)

ENDIF ( NOT OPENCV_INCLUDE_DIRS )

IF ( OPENCV_INCLUDE_DIRS AND ( NOT OPENCV_LIBRARY_DIRS OR NOT OPENCV_LIBRARIES))

    FOREACH(_SEARCH_DIR ${OPENCV_LIBRARY_SEARCH_DIRS})

        IF (UNIX)
          FIND_PATH(_CUR_SEARCH
              NAMES libopencv_core.so
              PATHS ${_SEARCH_DIR}
              NO_DEFAULT_PATH)
        ELSEIF(WIN32)
          # FIND_PATH(_CUR_SEARCH
          #     NAMES scm_gl_core.lib
          #     PATHS ${_SEARCH_DIR}
          #     PATH_SUFFIXES debug release
          #     NO_DEFAULT_PATH)
        ENDIF(UNIX)

        IF (_CUR_SEARCH)
            LIST(APPEND _OPENCV_FOUND_LIB_DIR ${_SEARCH_DIR})
        ENDIF(_CUR_SEARCH)

        SET(_CUR_SEARCH _CUR_SEARCH-NOTFOUND CACHE INTERNAL "internal use")

    ENDFOREACH(_SEARCH_DIR ${OPENCV_LIBRARY_SEARCH_DIRS})

    IF (NOT _OPENCV_FOUND_LIB_DIR)
        request_opencv_search_directories()
    ELSE (NOT _OPENCV_FOUND_LIB_DIR)
        SET(OPENCV_LIBRARY_DIRS ${_OPENCV_FOUND_LIB_DIR} CACHE STRING "The OpenCV library directory.")
    ENDIF (NOT _OPENCV_FOUND_LIB_DIR)

    SET(_OPENCV_LIBRARIES "")

    FOREACH(_LIB_DIR ${_OPENCV_FOUND_LIB_DIR})
        IF (UNIX)
          file(GLOB _OPENCV_LIBRARIES ${_LIB_DIR}/libopencv*.so)
        ELSEIF(WIN32)
          file(GLOB _OPENCV_LIBRARY_ABSOLUTE_PATHS ${_LIB_DIR}/release/scm*.lib)
          FOREACH (_OPENCV_LIB_PATH ${_OPENCV_LIBRARY_ABSOLUTE_PATHS})
            SET(_OPENCV_LIB_FILENAME "")
            GET_FILENAME_COMPONENT(_OPENCV_LIB_FILENAME ${_OPENCV_LIB_PATH} NAME)
            LIST(APPEND _OPENCV_LIBRARIES ${_OPENCV_LIB_FILENAME})
          ENDFOREACH(_OPENCV_LIB_PATH)
        ENDIF(UNIX)
    ENDFOREACH(_LIB_DIR ${_OPENCV_FOUND_INC_DIRS})

    IF (_OPENCV_FOUND_LIB_DIR)
        SET(OPENCV_LIBRARIES ${_OPENCV_LIBRARIES} CACHE STRING "OpenCV libraries.")
    ENDIF (_OPENCV_FOUND_LIB_DIR)

ENDIF ( OPENCV_INCLUDE_DIRS AND ( NOT OPENCV_LIBRARY_DIRS OR NOT OPENCV_LIBRARIES))

##############################################################################
# verify
##############################################################################
IF ( NOT OPENCV_INCLUDE_DIRS OR NOT OPENCV_LIBRARY_DIRS )
    request_opencv_search_directories()
ELSE ( NOT OPENCV_INCLUDE_DIRS OR NOT OPENCV_LIBRARY_DIRS )
    UNSET(OPENCV_INCLUDE_SEARCH_DIR CACHE)
    UNSET(OPENCV_LIBRARY_SEARCH_DIR CACHE)
    MESSAGE(STATUS "--  found matching OpenCV version")
ENDIF ( NOT OPENCV_INCLUDE_DIRS OR NOT OPENCV_LIBRARY_DIRS )



