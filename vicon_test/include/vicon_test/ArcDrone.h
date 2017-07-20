//Theory by Salvador Figuerola
//Coded by Alexis Guijarro

#ifndef ADD_H
#define ADD_H

//Constrain the Angle to 0 - 360, obtained from this thread in StackOverflow forum (https://stackoverflow.com/questions/11498169/dealing-with-angle-wrap-in-c-code), user Mystical.
float constrainAngle(float x);
//Used to calculate the lenght of the arcs and determine which one is shorter
float calcArc(float target, float current);

#endif

