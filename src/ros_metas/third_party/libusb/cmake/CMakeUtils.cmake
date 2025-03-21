# Reference from Belledonne Communications, Grenoble France.
macro(utils_apply_compile_flags SOURCE_FILES)
    if(${SOURCE_FILES})
        set(options "")
        foreach(a ${ARGN})
            if(${a})
                string(REPLACE ";" " " options_${a} "${${a}}")
                set(options "${options} ${options_${a}}")
            endif()
        endforeach()
        if(options)
            set_source_files_properties(${${SOURCE_FILES}} PROPERTIES COMPILE_FLAGS "${options}")
        endif()
    endif()
endmacro()

# Check sizeof a type. It will be required by the following utils.
include(CheckTypeSize)

macro(utils_check_type_exists TYPE FILES VARIABLE)
    set(CMAKE_EXTRA_INCLUDE_FILES "${FILES}")
    check_type_size("${TYPE}" ${TYPE}_SIZE)
    if(HAVE_${TYPE}_SIZE)
        set(${VARIABLE} 1)
    endif()
    set(CMAKE_EXTRA_INCLUDE_FILES "")
endmacro()

# Check if a symbol exists as a function, variable, or macro. It will be required by the following utils.
include(CheckSymbolExists)

macro(utils_check_symbol_exists SYMBOL FILES VARIABLE)
    check_symbol_exists("${SYMBOL}" "${FILES}" HAVE_${SYMBOL})
    if(HAVE_${SYMBOL})
        set(${VARIABLE} 1)
    endif()
endmacro()

macro(utils_check_symbol_exists_with_libraries SYMBOL FILES VARIABLE LIBRARIES)
    set(CMAKE_REQUIRED_LIBRARIES "${LIBRARIES}")
    check_symbol_exists("${SYMBOL}" "${FILES}" HAVE_${SYMBOL})
    if(HAVE_${SYMBOL})
        set(${VARIABLE} 1)
    endif()
    set(CMAKE_REQUIRED_LIBRARIES "")
endmacro()
