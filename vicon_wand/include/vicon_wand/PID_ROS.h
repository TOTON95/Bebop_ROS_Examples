//Coded by Daniel Flores and Martin Soto

#include <iostream>
#include <ros/ros.h>

#ifndef PID_ROS_H
#define PID_ROS_H

using namespace std;

class PID {
  const static float t=0.2;   //tiempo de toma de muestras
  public:
	float Vd;             //Velocidad deseado
	float Vo;             //Velocidad inicial
	float V;              //Velocidad medida
	float Xo;

	float ekt;
	float der;            //derivada
	float integ;          //integral
	float control1;

	float gder;           //Ganancia derivada
	float ginteg;         //Ganancia integral
	float gpro;           //Ganancia proporcional

	float mis(float, float);

	float derivada(float, float);

	float integral(float, float);

	float control(float ,float, float);

 	PID();
	~PID();
};
class PID_h {
  const static float t=0.2;   //tiempo de toma de muestras
  public:
	float Vd;             //Velocidad deseado
	float Vo;             //Velocidad inicial
	float V;              //Velocidad medida
	float Xo;

	float ekt;
	float der;            //derivada
	float integ;          //integral
	float control1;

	float gder;           //Ganancia derivada
	float ginteg;         //Ganancia integral
	float gpro;           //Ganancia proporcional

	float mis(float, float);

	float derivada(float, float);

	float integral(float, float);

	float control(float ,float, float);

 	PID_h();
	~PID_h();
};
#endif
