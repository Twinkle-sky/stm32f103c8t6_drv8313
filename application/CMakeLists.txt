#
# Application layer — static library
#
# The app library contains all business logic, isolated from
# the CubeMX-generated firmware. It depends on the STM32 HAL
# but not on any particular main.c or BSP configuration.
#

add_library(app STATIC)

target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/app.c
)

target_include_directories(app PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# Link against the STM32CubeMX interface library for HAL/CMSIS headers.
# This target is defined in firmware/cmake/stm32cubemx/CMakeLists.txt.
# motor_ctrl provides the spwm_* API used by app.c.
target_link_libraries(app PUBLIC
    stm32cubemx
    motor_ctrl
)
