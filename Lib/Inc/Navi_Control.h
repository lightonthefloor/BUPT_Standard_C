//
// Created by Trisoil on 2022/4/17.
//

#ifndef BASE_MOVING_NAVI_CONTROL_H
#define BASE_MOVING_NAVI_CONTROL_H

typedef struct {
		float kp;
		float ki;
		float kd;
		float kp_output;
		float ki_output;
		float kd_output;
		float output;
		float err_sum;
		float now_err;
		float last_err;
		float deadband;
		float max_output;
		float integral_limit;
		float max_err;
}Navi_PID_Number;

extern Navi_PID_Number X_PID,Y_PID,Angle_PID;

extern void Navi_Control_Init();
extern void Point_Navi_Move(float Set_X, float Set_Y, float Set_Angle);

#endif //BASE_MOVING_NAVI_CONTROL_H
