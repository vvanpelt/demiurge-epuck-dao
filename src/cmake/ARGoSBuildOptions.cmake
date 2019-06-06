#
# What is ARGoS being built for?
# Accepted values: "simulator" or a robot name (lowercase)
#
if(NOT DEFINED ARGOS_BUILD_FOR)
  # Variable was not set, set to default value
  set(ARGOS_BUILD_FOR "simulator" CACHE STRING "What is ARGoS being built for? \"simulator\" or a robot name (lowercase)")
else(NOT DEFINED ARGOS_BUILD_FOR)
  # Variable was set, make it public
  set(ARGOS_BUILD_FOR ${ARGOS_BUILD_FOR} CACHE STRING "What is ARGoS being built for? \"simulator\" or a robot name (lowercase)")
endif(NOT DEFINED ARGOS_BUILD_FOR)
# Set a macro according to value set in ARGOS_BUILD_FOR
add_definitions(-DARGOS_${ARGOS_BUILD_FOR}_BUILD)
#
# Create convenience variables for checks in the CMake files
#
set(ARGOS_BUILD_FOR_SIMULATOR  FALSE)
set(ARGOS_BUILD_FOR_XPUCK      FALSE)
set(ARGOS_BUILD_FOR_LOCALXPUCK FALSE)
# ARGOS_BUILD_FOR_SIMULATOR
if(ARGOS_BUILD_FOR STREQUAL "simulator")
  set(ARGOS_BUILD_FOR_SIMULATOR TRUE)
endif(ARGOS_BUILD_FOR STREQUAL "simulator")
# ARGOS_BUILD_FOR_XPUCK
if(ARGOS_BUILD_FOR STREQUAL "xpuck")
  set(ARGOS_BUILD_FOR_XPUCK TRUE)
endif(ARGOS_BUILD_FOR STREQUAL "xpuck")
# ARGOS_BUILD_FOR_LOCALXPUCK
if(ARGOS_BUILD_FOR STREQUAL "localxpuck")
  set(ARGOS_BUILD_FOR_LOCALXPUCK TRUE)
endif(ARGOS_BUILD_FOR STREQUAL "localxpuck")
