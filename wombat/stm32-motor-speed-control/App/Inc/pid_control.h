#ifndef INC_PID_CONTROL_H_
#define INC_PID_CONTROL_H_

#include "main.h"

typedef struct
{
    float k_p; /* proportional gain [Vs/rad] */
    float k_i; /* integral gain [V/rad]*/
    float k_d; /* derivative gain [Vs^2/rad] */
    float last_error; /* last error needed to compute derivative term, [rad/s] */
    float error_integral; /* culmulative error needed to compute integral term [rad/s] */
    float u_output; /* motor output voltage [V], capped to battery 12 V */
    float pwm_output; /* pwm output % scaled from motor output voltage, -100 to 100 */
    uint16_t sam_rate; /* sampling rate [Hz] */
    float integral_max; /* maximum of the integral term */
    float pid_max;
}pid_instance;

typedef enum
{
    pid_ok = 0,
    pid_numerical = 1,
}pid_typedef;

pid_typedef apply_pid(pid_instance *pid, float input_error);
void reset_pid(pid_instance *pid);
void set_pid(pid_instance *pid, float p, float i, float d);

#endif /* INC_PID_CONTROL_H_ */