option(ENABLE_PROJECTS "Build the Chrono_HIL project programs" OFF)

if(ENABLE_PROJECTS)
  message(STATUS "\n==== Chrono HIL projects ====")
  add_subdirectory(Iowa_Highway)
  add_subdirectory(SanFran_City)
  add_subdirectory(Unity_Couple)
endif()

