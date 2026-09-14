#
# drivers — custom hardware drivers / low-level modules
#
# Each subdirectory defines its own STATIC library, following the same
# pattern as application/: sources + public include dir + link to the
# 'stm32cubemx' INTERFACE target for HAL/CMSIS headers.
#

add_subdirectory(motor_ctrl)
add_subdirectory(as5600)

# Placeholder modules — uncomment when sources are added:
# add_subdirectory(pid)
# add_subdirectory(protocol)
