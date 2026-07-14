#ifndef _TASK_PID_H_ 
#define _TASK_PID_H_


void Control_PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float min_output, float max_output);
float PID_Compute(PID_Controller_t *pid,float y_error);
void PID_Control_Outer_Only(void);
void PID_Outer_Task(void);
void PID_Control_Outer_Only(void);

#endif // !_TASK_PID_H_