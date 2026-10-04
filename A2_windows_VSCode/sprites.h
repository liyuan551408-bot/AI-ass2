#ifndef __SPRITES_H__
#define __SPRITES_H__

#include <stddef.h>
#include <stdio.h>
#include <ctime>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <math.h>

#include "graphics.h"
#include "transform.h"


extern BoundaryType worldBoundary, deviceBoundary;
extern char keyPressed[5];


enum Position {LEFT_SIDE, RIGHT_SIDE};


///////////////////////////////////////////////////////////////
//
// Rod
//
///////////////////////////////////////////////////////////////

class Rod {

public:

    Rod(float sx = 0.0, float sy = 0.0, float a = 0.0);

    void setAngle(float a);

    void setX(const float& _x);

    void draw();


private:

    float x, y;
    float angle;
    float length;
    float halfWidth;
    float halfAngle;
    float pivotHeight;
    float axleHeight;
    float theta_b;
};


///////////////////////////////////////////////////////////////
//
// Cart
//
///////////////////////////////////////////////////////////////

class Cart {

public:

    Cart(float sx = 0.0,
         float sy = 0.0,
         float l = 0.5,
         float h = 0.35,
         float wheelRadius_ = 0.125,
         float s = 0.0);

    void draw();

    void setX(const float& _x);


private:

    float x, y;
    float length;
    float speed;
    float height;
    float wheelRadius;
    float halfWheelBase;
    float cartHalfWidth;
};


#endif