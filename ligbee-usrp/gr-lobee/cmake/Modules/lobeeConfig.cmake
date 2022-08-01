INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_LOBEE lobee)

FIND_PATH(
    LOBEE_INCLUDE_DIRS
    NAMES lobee/api.h
    HINTS $ENV{LOBEE_DIR}/include
        ${PC_LOBEE_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    LOBEE_LIBRARIES
    NAMES gnuradio-lobee
    HINTS $ENV{LOBEE_DIR}/lib
        ${PC_LOBEE_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(LOBEE DEFAULT_MSG LOBEE_LIBRARIES LOBEE_INCLUDE_DIRS)
MARK_AS_ADVANCED(LOBEE_LIBRARIES LOBEE_INCLUDE_DIRS)

