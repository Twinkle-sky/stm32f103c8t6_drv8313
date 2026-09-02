cmake_minimum_required(VERSION 3.22)

#
# Top-level CMake for blink-f103
# ---------------------------------------------------------------------
# Project structure:
#   firmware/     — CubeMX generated code (HAL, CMSIS, BSP, startup, linker)
#   application/  — Application business logic (isolated from firmware)
#   components/   — Third-party libraries (shell, logging, etc.)
#   drivers/      — Custom hardware drivers
#   tests/        — Unit / integration tests
#   docs/         — Design documents
#   hardware/     — Schematics, PCB, BOM
#   scripts/      — Build, flash, CI helpers
#

project(blink-f103 C ASM)

# Fix linker-script path: CubeMX toolchain references
#   ${CMAKE_SOURCE_DIR}/STM32F103xx_FLASH.ld
# but the actual file lives under firmware/.  Patch CMAKE_EXE_LINKER_FLAGS
# after the toolchain has set it, so we don't need a copy of the ld script
# at the repository root.
string(REPLACE "${CMAKE_SOURCE_DIR}/STM32F103xx_FLASH.ld"
               "${CMAKE_SOURCE_DIR}/firmware/STM32F103xx_FLASH.ld"
               CMAKE_EXE_LINKER_FLAGS
               "${CMAKE_EXE_LINKER_FLAGS}")

# ---- Sub-projects -------------------------------------------------------

# Order matters: firmware must come first — it defines the
# 'stm32cubemx' INTERFACE library that 'app' depends on.
add_subdirectory(firmware)

# Custom drivers (static libraries, e.g. motor_ctrl)
add_subdirectory(drivers)

# Application layer (static library)
add_subdirectory(application)

# ---- Top-level dependencies ---------------------------------------------
#
# Link the application library into the firmware executable.
# The executable target name equals CMAKE_PROJECT_NAME as set by
# firmware/CMakeLists.txt (inherited CubeMX convention).
#
# NOTE: Must use plain signature — CubeMX-generated cmake/stm32cubemx
# also uses plain target_link_libraries, and CMake forbids mixing styles.
target_link_libraries(firmware app m)
