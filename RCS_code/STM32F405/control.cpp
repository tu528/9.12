#include "control.h"
#include "tim.h"
#include "judgement.h"
#include "HTmotor.h"

void CONTROL::Init(std::vector<Motor*> motor)
{
	int num1{}, num2{}, num3{}, num4{};
	for (int i = 0; i < motor.size(); i++)
	{
		switch (motor[i]->function)
		{
		case(function_type::chassis):
			chassis_motor[num1++] = motor[i];
			break;
		case(function_type::pantile):
			pantile_motor[num2++] = motor[i];
			break;
		case(function_type::shooter):
			shooter_motor[num3++] = motor[i];
			break;
		case(function_type::supply):
			supply_motor[num4]->spinning = false;
			supply_motor[num4]->need_curcircle = false;
			supply_motor[num4++] = motor[i];
		default:
			break;
		}
	}
	pantile_motor[PANTILE::TYPE::PITCH]->setangle = para.initial_pitch;
	pantile_motor[PANTILE::TYPE::YAW]->setangle = para.initial_yaw;
}


void CONTROL::Control_Pantile(int32_t ch_yaw, int32_t ch_pitch)
{
	ch_pitch *= (-1.f);
	ch_yaw *= (1.f);//方向相反修改这里正负
	float adjangle = this->pantile.sensitivity * 2;

	ctrl.pantile.mark_pitch -= (float)(adjangle * ch_pitch);
	ctrl.pantile.mark_yaw -= (float)(adjangle * ch_yaw);
}

void CONTROL::PANTILE::Keep_Pantile(float angleKeep, PANTILE::TYPE type,IMU frameOfReference)
{
	float delta = 0, adjust = sensitivity;
	if (type == YAW)
	{
		delta = degreeToMechanical(ctrl.GetDelta(angleKeep - frameOfReference.GetAngleYaw()));

		if (delta <= -4096.f)
			delta += 8192.f;
		else if (delta >= 4096.f)
			delta -= 8192.f;
		if (abs(delta) >= 10.f)
			mark_yaw += pantile_PID[PANTILE::YAW].Delta(delta);

	}
	else if (type == PITCH)
	{
		delta = degreeToMechanical(ctrl.GetDelta(angleKeep - frameOfReference.GetAnglePitch()));
		if (delta <= -4096.f)
			delta += 8192.f;
		else if (delta >= 4096.f)
			delta -= 8192.f;
		if (abs(delta) >= 10.f)
		{
			mark_pitch += pantile_PID[PANTILE::PITCH].Delta(delta);
		}
	}
}

void CONTROL::CHASSIS::Keep_Direction()
{
	

}

void CONTROL::CHASSIS::Update()
{
	extern Motor can1_motor[4];


	if (ctrl.mode == RESET)
	{
		speedx = 0;
		speedy = 0;
		speedz = 0;
	}
	else if (ctrl.mode == CONTROL::TEST)
	{

	}
}

void CONTROL::PANTILE::Update()
{
	if (ctrl.mode == RESET)
	{
		mark_yaw = para.initial_yaw;
		mark_pitch = para.initial_pitch;
	}
}

void CONTROL::SHOOTER::Update()
{
	if (ctrl.mode == RESET)
	{

	}
}

float CONTROL::CHASSIS::Ramp(float setval, float curval, uint32_t RampSlope)
{

	if ((setval - curval) >= 0)
	{
		curval += RampSlope;
		curval = std::min(curval, setval);
	}
	else
	{
		curval -= RampSlope;
		curval = std::max(curval, setval);
	}

	return curval;
}

float CONTROL::GetDelta(float delta)
{
	if (delta <= -180.f)
	{
		delta += 360.f;
	}

	if (delta > 180.f)
	{
		delta -= 360.f;
	}
	return delta;
}

int16_t CONTROL::Setrange(const int16_t original, const int16_t range)
{
	return fmaxf(fminf(range, original), -range);
}

