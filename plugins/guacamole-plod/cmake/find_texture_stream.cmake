##############################################################################
# set search directories
##############################################################################

SET(TEXTURE_STREAM_INCLUDE_SEARCH_DIRS
    ${TEXTURE_STREAM_INCLUDE_SEARCH_DIR}
    ${CMAKE_SYSTEM_INCLUDE_PATH}
    ${CMAKE_INCLUDE_PATH}
    /usr/include
)

SET(TEXTURE_STREAM_LIBRARY_SEARCH_DIRS
    ${TEXTURE_STREAM_LIBRARY_SEARCH_DIR}
    ${CMAKE_SYSTEM_LIBRARY_PATH}
    ${CMAKE_LIBRARY_PATH}
    /usr/lib64
)

##############################################################################
# feedback to provide user-defined paths to search for texture_stream
##############################################################################
MACRO (request_texture_stream_search_directories)

    IF ( NOT TEXTURE_STREAM_INCLUDE_DIRS AND NOT TEXTURE_STREAM_LIBRARY_DIRS )
        SET(TEXTURE_STREAM_INCLUDE_SEARCH_DIR "Please provide texture_stream include path." CACHE PATH "path to texture_stream headers.")
        SET(TEXTURE_STREAM_LIBRARY_SEARCH_DIR "Please provide texture_stream library path." CACHE PATH "path to texture_stream libraries.")
        MESSAGE(FATAL_ERROR "find_texture_stream.cmake: unable to find texture_stream.")
    ENDIF ( NOT TEXTURE_STREAM_INCLUDE_DIRS AND NOT TEXTURE_STREAM_LIBRARY_DIRS )

    IF ( NOT TEXTURE_STREAM_INCLUDE_DIRS )
        SET(TEXTURE_STREAM_INCLUDE_SEARCH_DIR "Please provide texture_stream include path." CACHE PATH "path to texture_stream headers.")
        MESSAGE(FATAL_ERROR "find_texture_stream.cmake: unable to find texture_stream headers.")
    ELSE ( NOT TEXTURE_STREAM_INCLUDE_DIRS )
        UNSET(TEXTURE_STREAM_INCLUDE_SEARCH_DIR CACHE)
    ENDIF ( NOT TEXTURE_STREAM_INCLUDE_DIRS )

    IF ( NOT TEXTURE_STREAM_LIBRARY_DIRS )
        SET(TEXTURE_STREAM_LIBRARY_SEARCH_DIR "Please provide texture_stream library path." CACHE PATH "path to texture_stream libraries.")
        MESSAGE(FATAL_ERROR "find_texture_stream.cmake: unable to find texture_stream libraries.")
    ELSE ( NOT TEXTURE_STREAM_LIBRARY_DIRS )
        UNSET(TEXTURE_STREAM_LIBRARY_SEARCH_DIR CACHE)
    ENDIF ( NOT TEXTURE_STREAM_LIBRARY_DIRS )

ENDMACRO (request_texture_stream_search_directories)

##############################################################################
# start search
##############################################################################

message(STATUS "-- checking for texture_stream")

IF ( NOT TEXTURE_STREAM_INCLUDE_DIRS )

    SET(_TEXTURE_STREAM_FOUND_INC_DIRS "")
    FOREACH(_SEARCH_DIR ${TEXTURE_STREAM_INCLUDE_SEARCH_DIRS})
        FIND_PATH(_CUR_SEARCH
                NAMES texture_stream.hpp
                PATHS ${_SEARCH_DIR}/texture_stream
                NO_DEFAULT_PATH)
        IF (_CUR_SEARCH)
            LIST(APPEND _TEXTURE_STREAM_FOUND_INC_DIRS ${_CUR_SEARCH})
        ENDIF(_CUR_SEARCH)
        SET(_CUR_SEARCH _CUR_SEARCH-NOTFOUND CACHE INTERNAL "internal use")
    ENDFOREACH(_SEARCH_DIR ${TEXTURE_STREAM_INCLUDE_SEARCH_DIRS})

    IF (NOT _TEXTURE_STREAM_FOUND_INC_DIRS)
        request_texture_stream_search_directories()
    ELSE (NOT _TEXTURE_STREAM_FOUND_INC_DIRS)
        SET(TEXTURE_STREAM_INCLUDE_DIRS ${_TEXTURE_STREAM_FOUND_INC_DIRS})
    ENDIF (NOT _TEXTURE_STREAM_FOUND_INC_DIRS)

ENDIF ( NOT TEXTURE_STREAM_INCLUDE_DIRS )


IF ( TEXTURE_STREAM_INCLUDE_DIRS AND NOT TEXTURE_STREAM_LIBRARY_DIRS )

    SET(_TEXTURE_STREAM_FOUND_LIB_DIR "")

    if (UNIX)
        LIST(APPEND _TEXTURE_STREAM_LIB   "libtexture_stream.so")
    elseif (WIN32)
        # LIST(APPEND _TEXTURE_STREAM_LIB   "${CMAKE_STATIC_LIBRARY_PREFIX}libtexture_stream_filesystem-${COMPILER_SUFFIX}-mt-${TEXTURE_STREAM_LIB_VERSION}${CMAKE_STATIC_LIBRARY_SUFFIX}"
        #                                         "${CMAKE_STATIC_LIBRARY_PREFIX}libtexture_stream_filesystem-mt${CMAKE_STATIC_LIBRARY_SUFFIX}")
    endif (UNIX)

    FOREACH(_SEARCH_DIR ${TEXTURE_STREAM_LIBRARY_SEARCH_DIRS})

        FIND_PATH(_CUR_SEARCH
                NAMES ${_TEXTURE_STREAM_LIB}
                PATHS ${_SEARCH_DIR}
                PATH_SUFFIXES debug release .
                NO_DEFAULT_PATH)
        IF (_CUR_SEARCH)
            LIST(APPEND _TEXTURE_STREAM_FOUND_LIB_DIR ${_SEARCH_DIR})
        ENDIF(_CUR_SEARCH)
        SET(_CUR_SEARCH _CUR_SEARCH-NOTFOUND CACHE INTERNAL "internal use")

    ENDFOREACH(_SEARCH_DIR ${TEXTURE_STREAM_LIBRARY_SEARCH_DIRS})

    IF (NOT _TEXTURE_STREAM_FOUND_LIB_DIR)
        request_texture_stream_search_directories()
    ELSE (NOT _TEXTURE_STREAM_FOUND_LIB_DIR)
        SET(TEXTURE_STREAM_LIBRARY_DIRS ${_TEXTURE_STREAM_FOUND_LIB_DIR} CACHE PATH "texture_stream library path.")
        IF (UNIX)
            FILE(GLOB _TEXTURE_STREAM_LIBRARIES ${_TEXTURE_STREAM_FOUND_LIB_DIR}/*.so)
            SET(TEXTURE_STREAM_LIBRARIES ${_TEXTURE_STREAM_LIBRARIES} CACHE PATH "texture_stream libraries")
        ELSEIF (MSVC)
            # use texture_stream auto link
        ENDIF (UNIX)
    ENDIF (NOT _TEXTURE_STREAM_FOUND_LIB_DIR)


ENDIF ( TEXTURE_STREAM_INCLUDE_DIRS AND NOT TEXTURE_STREAM_LIBRARY_DIRS )

##############################################################################
# verify
##############################################################################
IF ( NOT TEXTURE_STREAM_INCLUDE_DIRS OR NOT TEXTURE_STREAM_LIBRARY_DIRS )
    request_texture_stream_search_directories()
ELSE ( NOT TEXTURE_STREAM_INCLUDE_DIRS OR NOT TEXTURE_STREAM_LIBRARY_DIRS )
    # UNSET(TEXTURE_STREAM_INCLUDE_SEARCH_DIR CACHE)
    # UNSET(TEXTURE_STREAM_LIBRARY_SEARCH_DIR CACHE)
    MESSAGE(STATUS "--  found matching texture_stream version")
ENDIF ( NOT TEXTURE_STREAM_INCLUDE_DIRS OR NOT TEXTURE_STREAM_LIBRARY_DIRS )
