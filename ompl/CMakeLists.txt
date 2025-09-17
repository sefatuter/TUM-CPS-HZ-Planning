# This file is part of OMPL.
#
# OMPL is free software: you can redistribute it and/or modify
# it under the terms of the Lesser GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# OMPL is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# Lesser GNU General Public License for more details.
#
# You should have received a copy of the Lesser GNU General Public License
# along with OMPL. If not, see <http://www.gnu.org/licenses/>.

# Author: Ioan Sucan

# Find necessary external libraries
find_package(Boost REQUIRED COMPONENTS serialization filesystem system)
find_package(Eigen3 REQUIRED)

# === ACTION: Find CGAL and its required dependencies ===
find_package(CGAL REQUIRED)
find_package(GMP REQUIRED)
find_package(MPFR REQUIRED)

# include parent directory
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/..")
# include Eigen include directory
include_directories("${EIGEN3_INCLUDE_DIR}")
# include Boost include directories
include_directories("${Boost_INCLUDE_DIRS}")

# === ACTION: Add CGAL include directories to the build ===
include_directories(${CGAL_INCLUDE_DIRS})

# The configuration header is generated in the build directory
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/config.h.in" "${CMAKE_CURRENT_BINARY_DIR}/config.h")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/config.h"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/ompl"
    COMPONENT ompl)

file(GLOB_RECURSE OMPL_SOURCE_CODE datastructures/*.cpp util/*.cpp base/*.cpp geometric/*.cpp control/*.cpp tools/*.cpp)
file(GLOB_RECURSE OMPL_HEADERS datastructures/*.h util/*.h base/*.h geometric/*.h control/*.h tools/*.h)

list(APPEND OMPL_SOURCE_CODE "${CMAKE_CURRENT_SOURCE_DIR}/libs/polypartition/src/polypartition.cpp")

#############################################
# Add enabled extensions to the source code #
#############################################

if (OMPL_EXTENSION_MORSE)
    file(GLOB_RECURSE OMPL_MORSE_EXTENSION_SOURCE_CODE extensions/morse/*.cpp)
    file(GLOB_RECURSE OMPL_MORSE_EXTENSION_HEADERS extensions/morse/*.h)
    set(OMPL_SOURCE_CODE ${OMPL_SOURCE_CODE} ${OMPL_MORSE_EXTENSION_SOURCE_CODE})
    set(OMPL_HEADERS ${OMPL_HEADERS} ${OMPL_MORSE_EXTENSION_HEADERS})
endif()

if (OMPL_EXTENSION_ODE)
    file(GLOB_RECURSE OMPL_ODE_EXTENSION_SOURCE_CODE extensions/ode/*.cpp)
    file(GLOB_RECURSE OMPL_ODE_EXTENSION_HEADERS extensions/ode/*.h)
    set(OMPL_SOURCE_CODE ${OMPL_SOURCE_CODE} ${OMPL_ODE_EXTENSION_SOURCE_CODE})
    set(OMPL_HEADERS ${OMPL_HEADERS} ${OMPL_ODE_EXTENSION_HEADERS})
    list(APPEND OMPL_LINK_LIBRARIES "${ODE_LIBRARIES}")
endif()

if (OMPL_EXTENSION_TRIANGLE)
    file(GLOB_RECURSE OMPL_TRIANGLE_EXTENSION_SOURCE_CODE extensions/triangle/*.cpp)
    file(GLOB_RECURSE OMPL_TRIANGLE_EXTENSION_HEADERS extensions/triangle/*.h)
    set(OMPL_SOURCE_CODE ${OMPL_SOURCE_CODE} ${OMPL_TRIANGLE_EXTENSION_SOURCE_CODE})
    set(OMPL_HEADERS ${OMPL_HEADERS} ${OMPL_TRIANGLE_EXTENSION_HEADERS})
    list(APPEND OMPL_LINK_LIBRARIES "${TRIANGLE_LIBRARY}")
endif()

source_group("OMPL Source" FILES "${OMPL_SOURCE_CODE}")
source_group("OMPL Headers" FILES "${OMPL_HEADERS}")

# build the library
if(MSVC)
    add_library(ompl STATIC ${OMPL_SOURCE_CODE})
else(MSVC)
    add_library(ompl SHARED ${OMPL_SOURCE_CODE})
endif(MSVC)

target_include_directories(ompl PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../external/zonocpp/include
    ${CMAKE_CURRENT_SOURCE_DIR}/polypartition/src
)

# Link the OMPL library to its dependencies
target_link_libraries(ompl
    ${OMPL_LINK_LIBRARIES}
    ${Boost_SERIALIZATION_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${CMAKE_THREAD_LIBS_INIT}
    # === ACTION: Add the CGAL and dependency libraries to the link step ===
    ${CGAL_LIBRARIES}
    ${GMP_LIBRARIES}
    ${MPFR_LIBRARIES}
)

if (OMPL_EXTENSION_ODE)
    if (NOT CMAKE_VERSION VERSION_LESS 3.13)
        target_link_directories(ompl PUBLIC ${ODE_LIBRARY_DIRS})
    else()
        link_directories(ompl ${ODE_LIBRARY_DIRS})
    endif()
endif()

if (OMPL_HAVE_SPOT)
    target_link_libraries(ompl ${SPOT_LIBRARIES})
endif()

if (MSVC)
    set_target_properties(ompl PROPERTIES VERSION "${PROJECT_VERSION}" STATIC_LIBRARY_FLAGS "psapi.lib ws2_32.lib")
else (MSVC)
    if (MINGW)
        target_link_libraries(ompl psapi ws2_32)
        set_target_properties(ompl PROPERTIES LINK_FLAGS "-Wl,--export-all-symbols")
    endif (MINGW)
    set_target_properties(ompl PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION "${OMPL_ABI_VERSION}")
endif (MSVC)

# install the library
install(TARGETS ompl
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
    COMPONENT ompl)
if (NOT MSVC)
    add_custom_command(TARGET ompl POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy "$<TARGET_FILE:ompl>"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../py-bindings/ompl/util/libompl${CMAKE_SHARED_LIBRARY_SUFFIX}"
        WORKING_DIRECTORY "${LIBRARY_OUTPUT_PATH}")
endif (NOT MSVC)