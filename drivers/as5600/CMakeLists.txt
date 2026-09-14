#
# as5600 — AS5600 magnetic encoder driver (static library)
#
# Reads the 12-bit raw angle over I2C1 with DMA (request started by
# AS5600_Read_Angle(), result published from HAL_I2C_MemRxCpltCallback).
# Depends on the CubeMX-generated HAL for I2C1 access
# (as5600.c includes main.h / i2c.h and uses hi2c1).
#

add_library(as5600 STATIC)

target_sources(as5600 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/as5600.c
)

# Public: consumers (app) include as5600.h directly.
target_include_directories(as5600 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# PUBLIC: as5600.c includes main.h / i2c.h (CubeMX handles), so consumers
# must also see the CubeMX include paths transitively.
target_link_libraries(as5600 PUBLIC
    stm32cubemx
)
