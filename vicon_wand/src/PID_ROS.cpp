//Coded by Daniel Flores and Martin Soto

#include <vicon_wand/PID_ROS.h>
#include <ros/ros.h>

PID::PID(void){}
PID::~PID(void){}

float PID::mis(float V, float Vd)
{  
float resultado=0;
float acon = 0;

ekt = Vd - V;

ros::Time a_little_after_the_beginning(0.200);

der = PID::derivada(ekt, Xo);
integ = PID::integral(ekt, Vo);


Vo=ekt;
Xo=der;

resultado = PID::control(ekt, integ, der);

if(resultado > 0)
{
	resultado = resultado*(0.10);
	if(resultado > 0.20){resultado = 0.20;}
	else{acon = acon;}
}
else
{
	resultado = resultado * (0.10);
	if(resultado < -0.20){resultado = -0.20;}
	else{resultado = resultado;}
}

return resultado;
}

float PID::derivada(float ekt,float Xo)
{
der=Xo + (ekt * t);
return der;
}

float PID::integral(float ekt, float Vo)
{
integ= (ekt - Vo) / t;
return integ;
}

float PID::control(float ekt,float integ, float der)
{
control1= (gpro*ekt) + (gder*der) + (ginteg*integ);
return control1;

}

PID_h::PID_h(void){}
PID_h::~PID_h(void){}

float PID_h::mis(float V, float Vd)
{  
float resultado=0;



ekt = Vd - V;

ros::Time a_little_after_the_beginning(0.200);

der = PID_h::derivada(ekt, Xo);
integ = PID_h::integral(ekt, Vo);


Vo=ekt;
Xo=der;

resultado = (PID_h::control(ekt, integ, der)) * (0.085);
return resultado;
}

float PID_h::derivada(float ekt,float Xo)
{
der=Xo + (ekt * t);
return der;
}

float PID_h::integral(float ekt, float Vo)
{
integ= (ekt - Vo) / t;
return integ;
}

float PID_h::control(float ekt,float integ, float der)
{
control1= (gpro*ekt) + (gder*der) + (ginteg*integ);
return control1;

}
