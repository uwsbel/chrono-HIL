message(STATUS "==== Chrono::HIL Module ====")

set(DRIVER_FILES
    #driver/ChCSLDriver.h
    #driver/ChCSLDriver.cpp
    driver/ChLidarWaypointDriver.h
    driver/ChLidarWaypointDriver.cpp
    driver/ChNSF_Drivers.h
    driver/ChNSF_Drivers.cpp
    #driver/joystick.h
)
source_group("driver" FILES ${DRIVER_FILES})

set(TIMER_FILES
    timer/ChRealtimeCumulative.h
)
source_group("timer" FILES ${TIMER_FILES})

set(SOUND_FILES
    sound/ChCSLSoundEngine.h
    sound/ChCSLSoundEngine.cpp
)
source_group("sound" FILES ${SOUND_FILES})

set(CXX_FLAGS ${CH_CXX_FLAGS})
set(LIBRARIES ${CHRONO_LIBRARIES})


add_library(ChronoEngine_hil SHARED 
            ${DRIVER_FILES}
            ${TIMER_FILES}
            ${SOUND_FILES}
)

set_target_properties(ChronoEngine_hil PROPERTIES
                      COMPILE_FLAGS "${CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}")
                      
include_directories(${CHRONO_INCLUDE_DIRS})


target_compile_definitions(ChronoEngine_hil PRIVATE "CH_API_COMPILE_HIL")
target_compile_definitions(ChronoEngine_hil PRIVATE "CH_IGNORE_DEPRECATED")

target_link_libraries(ChronoEngine_hil ${LIBRARIES})

install(TARGETS ChronoEngine_hil
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_hil
        FILES_MATCHING PATTERN "*.h")
install(FILES ChApiModels.h DESTINATION include/chrono_hil)



