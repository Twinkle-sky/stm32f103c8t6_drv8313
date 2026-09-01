/**
 * @file    app_config.h
 * @brief   Application configuration — compile-time constants and feature toggles.
 *
 * Include this header wherever application-wide settings are needed.
 * Change values here to tune behaviour without touching business logic.
 */

#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/*  Version                                                                   */
/* -------------------------------------------------------------------------- */

#define APP_VERSION_MAJOR    0
#define APP_VERSION_MINOR    1
#define APP_VERSION_PATCH    0

/* -------------------------------------------------------------------------- */
/*  Feature toggles (1 = enabled, 0 = disabled)                               */
/* -------------------------------------------------------------------------- */

#define APP_FEATURE_SHELL       1   /* Letter_Shell serial console            */
#define APP_FEATURE_LOGGER      1   /* EasyLogger runtime log output          */

/* -------------------------------------------------------------------------- */
/*  Debug / Logging                                                           */
/* -------------------------------------------------------------------------- */

#define APP_DEBUG_ENABLE        1
#define APP_LOG_LEVEL           3   /* 0=off  1=error  2=warn  3=info  4=debug */

/* -------------------------------------------------------------------------- */
/*  Hardware constants                                                        */
/* -------------------------------------------------------------------------- */

#define APP_SYSTICK_PERIOD_MS   1   /* SysTick interval for HAL timebase      */
#define APP_SERIAL_BAUDRATE     115200

/* -------------------------------------------------------------------------- */
/*  RTOS / timing (reserved for future use)                                   */
/* -------------------------------------------------------------------------- */

#define APP_TASK_STACK_MIN      256
#define APP_HEARTBEAT_PERIOD_MS 500

#ifdef __cplusplus
}
#endif

#endif /* __APP_CONFIG_H */
