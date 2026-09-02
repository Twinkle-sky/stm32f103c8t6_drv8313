#
# motor_ctrl — motor control drivers (static library)
#
# Currently provides the SPWM three-phase sinusoidal PWM generator.
# Depends on the CubeMX-generated HAL for TIM2/GPIO access
# (spwm.c includes main.h and uses htim2 / MOTOR_EN).
#

add_library(motor_ctrl STATIC)

target_sources(motor_ctrl PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/spwm.c
)

# Public: consumers (app) include spwm.h directly.
target_include_directories(motor_ctrl PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# PUBLIC: spwm.c includes main.h (HAL handles, pin labels), so consumers
# must also see the CubeMX include paths transitively.
# m (libm) must be linked AFTER this library on the linker command line:
# spwm.c calls sinf(), and GNU ld resolves symbols strictly left-to-right.
# CMake emits a static library's PUBLIC deps right after itself, so the
# '-lm' here ends up in the correct position (after libmotor_ctrl.a).
target_link_libraries(motor_ctrl PUBLIC
    stm32cubemx
    m
)
