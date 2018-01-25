#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "helpers.h"

float max_angle_limiting_factor(float des_pitch, float des_roll, float max_angle)
{
	float cos_max = cos(max_angle);
	float limiting_factor = 1.0;
	for(float factor_test = 1.0 ; factor_test >= 0.6; factor_test -= 0.02)
	{
		float p2 = factor_test * des_pitch;
		float r2 = factor_test * des_roll;
		if(cos(p2)*cos(r2) > cos_max)
		{
			limiting_factor = factor_test;
			break;
		}
	}
		return limiting_factor;
}

float heading_mod_2pi(float a)
{
	// Returns negative float modulus in rad
    float ans = fmodf(a, (2 * M_PI));
    if(ans < 0)
    {
        ans += (2 * M_PI);
    }
    return ans;
}

float GetHeadingError(float initial_heading, float final_heading)
{
	float diff = final_heading - initial_heading;
	float absDiff = abs(diff);
	if (absDiff <= M_PI)
	{
		return absDiff == M_PI ? absDiff : diff;
	}
	else if (final_heading > initial_heading)
	{
		return absDiff - (2 * M_PI);
	}
	else
	{
		return (2 * M_PI) - absDiff;
	}
}