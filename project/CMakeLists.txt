option(ENABLE_PROJECTS "Build the Chrono_HIL project programs" OFF)

if(ENABLE_PROJECTS)
  message(STATUS "\n==== Chrono HIL projects ====")
  add_subdirectory(Iowa_Highway)
  add_subdirectory(SanFran_City)
  add_subdirectory(Unity_Couple)
  add_subdirectory(HMMWV_SCM)
  add_subdirectory(Jsonfy_Env)

  #---------------
  string(FIND "${CHRONO_LIBRARIES}" "fastcdr" matchres)
  if(NOT ${matchres} EQUAL -1)
    add_subdirectory(HMMWV_MHIL)
  endif()
endif()
