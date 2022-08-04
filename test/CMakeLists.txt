option(ENABLE_TESTS "Build the Chrono_HIL test programs" OFF)

if(ENABLE_TESTS)
  message(STATUS "\n==== Chrono HIL tests ====")
  add_subdirectory(camera)
  add_subdirectory(rom_models)
endif()
