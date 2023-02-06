option(ENABLE_TESTS "Build the Chrono_HIL test programs" OFF)

if(ENABLE_TESTS)
  message(STATUS "\n==== Chrono HIL tests ====")
  add_subdirectory(sdl_driver)
  add_subdirectory(Rom_Models)
endif()
