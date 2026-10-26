/**
 * @file gripper_auto_calibration.h
 * @brief Consignas compartidas de la pala y referencias geometricas nominales.
 * Los valores RC son microsegundos, no grados medidos. Los presets de altura
 * son referencias de plataforma. La secuencia activa puede usar los objetivos
 * de auto_box_calibration.h y el perfil de auto_pickup_height.h.
 * Un ACK RC confirma la consigna, no la posicion mecanica de la compuerta.
 */
#ifndef GRIPPER_AUTO_CALIBRATION_H
#define GRIPPER_AUTO_CALIBRATION_H

#define GRIP_Z_PICKUP_US 1730u
#define GRIP_X_PICKUP_US 1510u
#define GRIP_X_CARRY_US 1090u /* Inclinar con la compuerta cerrada antes de elevar. */

#define GRIP_PICKUP_Z_MM -342.24f
#define GRIP_DROP_Z_MM -262.19f  

#define GRIP_Z_DROP_US 2500u
#define GRIP_X_LEVEL_US 1390u  
#define GRIP_X_DROP_US 1800u  

#define GRIP_CAMERA_TO_PICKUP_MM 500.0f

#define GRIP_PUSH_WAIT_US 1480u  
#define GRIP_PUSH_CLOSED_US 2500u
#define GRIP_PUSH_OPEN_US 500u
#define GRIP_OPEN_SETTLE_MS 300u  
#define GRIP_AUTO_READY 1u  
#endif
