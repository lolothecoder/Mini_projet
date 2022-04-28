#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

/*
 * To be able to retrieve the value of moving in other modules
 */
int8_t get_moving (void);


/*
 * To be able to affect the value of moving in from other modules
 */
void set_moving (int8_t new_moving);

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
