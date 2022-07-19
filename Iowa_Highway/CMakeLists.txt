#=============================================================================
# CMake configuration file for the SynChrono Highway project - an extended
#   demo of a highway environment, interfaced to a human-driven simulator
#=============================================================================

#--------------------------------------------------------------
# List of all executables
#--------------------------------------------------------------

set(DEMOS
	test_SYN_highway
	test_SYN_highway_syn
)

#--------------------------------------------------------------
# Find the Chrono package with required and optional components
#--------------------------------------------------------------

# Invoke find_package in CONFIG mode
find_package(Chrono
             COMPONENTS SynChrono Vehicle Sensor Irrlicht
             CONFIG
)

# If Chrono and/or the required component(s) were not found, return now.
if(NOT Chrono_FOUND)
  message("Could not find requirements for the SynChrono Highway Project")
  return()
endif()

# Get the path to the highway mesh
set(HIGHWAY_DATA_DIR "" CACHE FILEPATH "Path to directory contain highway assets")
#add_compile_definitions(HIGHWAY_COL_PATH="${HIGHWAY_DIR}Highway_new.obj")
#add_compile_definitions(HIGHWAY_VIS_PATH="${HIGHWAY_DIR}Highway_new.obj")
add_compile_definitions(HIGHWAY_DATA_DIR=${HIGHWAY_DATA_DIR})

#---------------
# Find DDS
#---------------
string(FIND "${CHRONO_LIBRARIES}" "fastcdr" matchres)
if(NOT ${matchres} EQUAL -1)
	message(STATUS "Using Fast DDS...")
	add_compile_definitions(USE_FAST_DDS=1)
	set(FAST_DDS_INSTALL_DIR "" CACHE PATH "Path to FastDDS install dir")
	set(fastcdr_DIR "${FAST_DDS_INSTALL_DIR}/lib/cmake/fastcdr" CACHE FILEPATH "Path to the FastCDR cmake config file" FORCE)
	set(fastrtps_DIR "${FAST_DDS_INSTALL_DIR}/share/fastrtps/cmake" CACHE FILEPATH "Path to the FastDDS cmake config file" FORCE)
	set(foonathan_memory_DIR "${FAST_DDS_INSTALL_DIR}/lib/foonathan_memory/cmake" CACHE FILEPATH "Path to the foonathan memory cmake config file" FORCE)
	if (NOT EXISTS ${foonathan_memory_DIR})
		set(foonathan_memory_DIR "${FAST_DDS_INSTALL_DIR}/lib64/foonathan_memory/cmake" CACHE PATH "Path to the foonathan memory cmake config file" FORCE)
	endif()
	find_package(fastcdr REQUIRED)
	find_package(fastrtps REQUIRED)
endif()

#--------------------------------------------------------------
# Include paths and libraries
#--------------------------------------------------------------

include_directories(
    ${CHRONO_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}
)

set(EXT_LIBRARIES 
	ChronoEngine_hil
)



#--------------------------------------------------------------
# Append to the parent's list of DLLs
#--------------------------------------------------------------

list(APPEND ALL_DLLS "${CHRONO_DLLS}")
set(ALL_DLLS "${ALL_DLLS}" PARENT_SCOPE)

#--------------------------------------------------------------
# Compilation flags
#--------------------------------------------------------------

set(COMPILE_FLAGS ${CHRONO_CXX_FLAGS})

# Disable some warnings triggered by Irrlicht (Windows only)
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    SET(COMPILE_FLAGS "${COMPILE_FLAGS} /wd4275")
endif()

#--------------------------------------------------------------
# Loop over all demo programs and build them
#--------------------------------------------------------------

message(STATUS "Projects for SynChrono Highway...")

foreach(PROGRAM ${DEMOS})

  message(STATUS "...add ${PROGRAM}")

  add_executable(${PROGRAM}  "${PROGRAM}.cpp")
  source_group(""  FILES "${PROGRAM}.cpp")
  set_target_properties(${PROGRAM} PROPERTIES
    FOLDER demos
    COMPILE_FLAGS "${COMPILE_FLAGS}"
    COMPILE_DEFINITIONS "PROJECTS_DATA_DIR=\"${PROJECTS_DATA_DIR}\";CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
    LINK_FLAGS "${CHRONO_LINKER_FLAGS}"
  )

	target_link_libraries(${PROGRAM} ${EXT_LIBRARIES} ${CHRONO_LIBRARIES} "-L/usr/local/cuda/lib64")# -lcudart")

endforeach(PROGRAM)