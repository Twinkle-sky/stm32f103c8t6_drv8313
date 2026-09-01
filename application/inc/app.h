/**
 ******************************************************************************
 * @file    app.h
 * @brief   Application layer public interface
 ******************************************************************************
 * @attention
 *
 * This file contains the public API for the application layer.
 * The application layer is hardware-independent business logic
 * that runs on top of the firmware (BSP/HAL) layer.
 *
 ******************************************************************************
 */

#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Public Functions -----------------------------------------------------------*/

/**
 * @brief  Application initialization.
 * @note   Called once after hardware initialization in main().
 *         Use this to initialize app-level modules, state machines, etc.
 */
void app_init(void);

/**
 * @brief  Application main loop iteration.
 * @note   Called repeatedly in the main while(1) loop.
 *         This function should return promptly; long-running operations
 *         should be split across iterations via state machines.
 */
void app_run(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
