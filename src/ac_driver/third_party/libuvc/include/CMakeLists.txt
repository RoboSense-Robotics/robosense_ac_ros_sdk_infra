set(INCLUDE_SUBDIR_NAME "${PROJECT_NAME_LOWER}")

set(HEADER_FILES
    ${CMAKE_CURRENT_LIST_DIR}/libuvc/libuvc.h
    ${PROJECT_BINARY_DIR}/libuvc/libuvc_config.h
)

# install(FILES ${HEADER_FILES}
#     DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/libuvc
#     PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ
# )
