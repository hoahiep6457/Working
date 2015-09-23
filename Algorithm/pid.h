#ifndef PID_H_
#define PID_H_

#ifdef __cplusplus
 extern "C" {
#endif

#define pid_sampletime  0.01
//variable PID
#define   Kp  ? //value;
#define   Ki  ??
#define   Kd  ???

#define   Outmax  ??,
#define   Outmin  fdf

typedef struct {
  float lastError;
  float Kp_Value;
  float Ki_Value;
  float Kd_Value;
  float Output;
} pid_t;
float PID_Adjustment(pid_t pid, float setpoint, float target);
void PID_Init(void);

#ifdef __cplusplus
}
#endif
#endif