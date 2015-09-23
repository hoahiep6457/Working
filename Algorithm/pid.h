#ifndef PID_H_
#define PID_H_

#ifdef __cplusplus
 extern "C" {
#endif

#define pid_sampletime  0.01
//variable PID
#define   Kp  100 //value;
#define   Ki  50
#define   Kd  5

#define   Outmax  800
#define   Outmin  800

typedef struct {
  float lastError;
  float Kp_Value;
  float Ki_Value;
  float Kd_Value;
  float Output;
} pid_t;
float PID_Adjustment(pid_t *pid, float setpoint, float target);
void PID_Init(void);

#ifdef __cplusplus
}
#endif
#endif