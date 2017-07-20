//Coded by Salvador Figuerola

#include <bebop_simple_test/ArcDrone.h>
#include <math.h>

float constrainAngle(float x){
    x = fmod(x,360);
    if (x < 0)
        x += 360;
    return x;
}

float calcArc(float target, float current)
{
	float arc1,arc2;
	
	if((target-current)>0)
	{
		arc1 = current + (360-target);
		arc2 = target - current;
		
		if(arc1 > arc2)
		{
			return arc2;
		}
		else
		{
			arc1 = arc1*(-1);
			return arc1;
		}
	}
	else
	{
		arc1 = target + (360-current);
		arc2 = current - target;
		
		if(arc1>arc2)
		{	
			arc2 = arc2 * (-1);
			return arc2;
		}
		else
		{	
			return arc1;
		}
	}
}
