# Copyright 2011,2012,2016 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

########################################################################
# Setup library
########################################################################
include(GrPlatform) #define LIB_SUFFIX

include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

list(APPEND lobee_sources
    quad_to_byte_fb_impl.cc
    lobee_cpp_bp_impl.cc
)

set(lobee_sources "${lobee_sources}" PARENT_SCOPE)
if(NOT lobee_sources)
	MESSAGE(STATUS "No C++ sources... skipping lib/")
	return()
endif(NOT lobee_sources)

add_library(gnuradio-lobee SHARED ${lobee_sources})
target_link_libraries(gnuradio-lobee ${Boost_LIBRARIES} ${GNURADIO_ALL_LIBRARIES})
set_target_properties(gnuradio-lobee PROPERTIES DEFINE_SYMBOL "gnuradio_lobee_EXPORTS")

if(APPLE)
    set_target_properties(gnuradio-lobee PROPERTIES
        INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    )
endif(APPLE)

########################################################################
# Install built library files
########################################################################
include(GrMiscUtils)
GR_LIBRARY_FOO(gnuradio-lobee RUNTIME_COMPONENT "lobee_runtime" DEVEL_COMPONENT "lobee_devel")

########################################################################
# Build and register unit test
########################################################################
include(GrTest)

include_directories(${CPPUNIT_INCLUDE_DIRS})

list(APPEND test_lobee_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/test_lobee.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/qa_lobee.cc
)

add_executable(test-lobee ${test_lobee_sources})

target_link_libraries(
  test-lobee
  ${GNURADIO_RUNTIME_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CPPUNIT_LIBRARIES}
  gnuradio-lobee
)

GR_ADD_TEST(test_lobee test-lobee)

########################################################################
# Print summary
########################################################################
message(STATUS "Using install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Building for version: ${VERSION} / ${LIBVER}")

