set(LIBRARY_NAME "${PROJECT_NAME_LOWER}")

set(SOURCE_FILES_C
    ctrl.c
    ctrl-gen.c
    device.c
    diag.c
    frame.c
#    frame-mjpeg.c
    init.c
    misc.c
    stream.c
)

set(LIBS
    ${COMMON_LIBS}
)

set(INCLUDE_DIRS
    ${COMMON_INCLUDE_DIRS}
)

#utils_apply_compile_flags(SOURCE_FILES_C STRICT_OPTIONS_CPP STRICT_OPTIONS_C)

if(ENABLE_STATIC_UVC)
    add_library(${LIBRARY_NAME}-static STATIC ${SOURCE_FILES_C})
    target_link_libraries(${LIBRARY_NAME}-static INTERFACE ${LIBS})
    target_include_directories(${LIBRARY_NAME}-static PUBLIC ${INCLUDE_DIRS})
    # install(TARGETS ${LIBRARY_NAME}-static EXPORT ${PROJECT_NAME}Targets
    #     ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    #     PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
    # )
endif()
if(ENABLE_SHARED_UVC)
    add_library(${LIBRARY_NAME} SHARED ${SOURCE_FILES_C})
    target_link_libraries(${LIBRARY_NAME} PRIVATE ${LIBS})
    target_include_directories(${LIBRARY_NAME} PUBLIC ${INCLUDE_DIRS})
    # install(TARGETS ${LIBRARY_NAME} EXPORT ${PROJECT_NAME}Targets
    #     RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    #     LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    #     ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    #     PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
    # )
endif()

set(LIBRARY_NAME "${LIBRARY_NAME}" PARENT_SCOPE)
