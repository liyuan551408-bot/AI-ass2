#include "sprites.h"


///////////////////////////////////////////////////////////////
//
// Rod
//
///////////////////////////////////////////////////////////////


Rod::Rod(float sx, float sy, float a) {

    x = sx;
    y = sy;
    angle = a;

    length = 2.0;
    halfWidth = 0.078125;
    halfAngle = 0.04;       // not currently used
    pivotHeight = 0.1;
    axleHeight = 0.16;

    theta_b = 0.03904265;   // approximately 0.04
}


///////////////////////////////////////////////////////////////

void Rod::setAngle(float a) {

    angle = a;
}


///////////////////////////////////////////////////////////////

void Rod::setX(const float& _x) {

    x = _x;
}


///////////////////////////////////////////////////////////////

void Rod::draw() {

    int poly[8];
    float points[8];

    // Distance from the pivot point to the upper corner
    // of the rod.
    float r = 0.0;

    r = float(sqrt(pow(halfWidth, 2) + pow(length, 2)));


    //---------------------------------------------------------
    // Calculate the four vertices of the rod.
    //---------------------------------------------------------

    // Top left
    points[0] = x + (sin(angle - theta_b) * r);
    points[1] = cos(angle - theta_b) * r;

    // Top right
    points[2] = x + (sin(angle + theta_b) * r);
    points[3] = cos(angle + theta_b) * r;


    float theta_p = 90.0 * 3.14 / 180.0;


    // Bottom right
    points[4] = x + (sin(angle + theta_p) * halfWidth);
    points[5] = cos(angle + theta_p) * halfWidth;

    // Bottom left
    points[6] = x + (sin(angle - theta_p) * halfWidth);
    points[7] = cos(angle - theta_p) * halfWidth;


    //---------------------------------------------------------
    // Convert world coordinates to device coordinates.
    //---------------------------------------------------------

    for (int i = 0; i < 8; i = i + 2) {

        poly[i] =
            xDev(worldBoundary,
                 deviceBoundary,
                 points[i]);

        poly[i + 1] =
            yDev(worldBoundary,
                 deviceBoundary,
                 points[i + 1]);
    }


    //---------------------------------------------------------
    // Draw the rod.
    //---------------------------------------------------------

    setcolor(WHITE);

    setfillstyle(SOLID_FILL, BLACK);

    fillpoly(4, poly);


    //---------------------------------------------------------
    // Draw the pivot.
    //---------------------------------------------------------

    setfillstyle(SOLID_FILL, DARKGRAY);

    int radius =
        int((halfWidth / 2) *
            ((deviceBoundary.x2 - deviceBoundary.x1) /
             (worldBoundary.x2 - worldBoundary.x1)));

    fillellipse(
        xDev(worldBoundary, deviceBoundary, x),
        yDev(worldBoundary, deviceBoundary, pivotHeight / 3),
        radius,
        radius
    );
}



///////////////////////////////////////////////////////////////
//
// Cart
//
///////////////////////////////////////////////////////////////


Cart::Cart(float sx,
           float sy,
           float l,
           float h,
           float wheelRadius_,
           float s) {

    x = sx;
    y = sy;

    length = l;
    height = h;
    speed = s;
    wheelRadius = wheelRadius_;

    halfWheelBase = 0.265625;
    cartHalfWidth = 0.25;
}


///////////////////////////////////////////////////////////////

void Cart::draw() {

    int poly[8];
    float points[8];


    //---------------------------------------------------------
    // Calculate the four vertices of the cart.
    //---------------------------------------------------------

    // Top left
    points[0] = x - cartHalfWidth;
    points[1] = y + height;

    // Top right
    points[2] = x + cartHalfWidth;
    points[3] = y + height;

    // Bottom right
    points[4] = x + cartHalfWidth;
    points[5] = y;

    // Bottom left
    points[6] = x - cartHalfWidth;
    points[7] = y;


    //---------------------------------------------------------
    // Convert world coordinates to device coordinates.
    //---------------------------------------------------------

    for (int i = 0; i < 8; i = i + 2) {

        poly[i] =
            xDev(worldBoundary,
                 deviceBoundary,
                 points[i]);

        poly[i + 1] =
            yDev(worldBoundary,
                 deviceBoundary,
                 points[i + 1]);
    }


    //---------------------------------------------------------
    // Draw the cart body.
    //---------------------------------------------------------

    setcolor(WHITE);

    setfillstyle(SOLID_FILL, GREEN);

    fillpoly(4, poly);


    //---------------------------------------------------------
    // Calculate wheel radius in device coordinates.
    //---------------------------------------------------------

    int radius =
        int(wheelRadius *
            ((deviceBoundary.x2 - deviceBoundary.x1) /
             (worldBoundary.x2 - worldBoundary.x1)));


    //---------------------------------------------------------
    // Draw the wheels.
    //---------------------------------------------------------

    setcolor(WHITE);

    setfillstyle(SOLID_FILL, DARKGRAY);

    fillellipse(
        xDev(worldBoundary,
             deviceBoundary,
             x + (cartHalfWidth / 2)),
        yDev(worldBoundary,
             deviceBoundary,
             y),
        radius,
        radius
    );

    fillellipse(
        xDev(worldBoundary,
             deviceBoundary,
             x - (cartHalfWidth / 2)),
        yDev(worldBoundary,
             deviceBoundary,
             y),
        radius,
        radius
    );


    //---------------------------------------------------------
    // Draw wheel outlines.
    //---------------------------------------------------------

    setcolor(BLACK);

    circle(
        xDev(worldBoundary,
             deviceBoundary,
             x + (cartHalfWidth / 2)),
        yDev(worldBoundary,
             deviceBoundary,
             y),
        radius
    );

    circle(
        xDev(worldBoundary,
             deviceBoundary,
             x - (cartHalfWidth / 2)),
        yDev(worldBoundary,
             deviceBoundary,
             y),
        radius
    );


    //---------------------------------------------------------
    // Draw wheel spindles.
    //---------------------------------------------------------

    setfillstyle(SOLID_FILL, LIGHTGRAY);

    fillellipse(
        xDev(worldBoundary,
             deviceBoundary,
             x + (cartHalfWidth / 2)),
        yDev(worldBoundary,
             deviceBoundary,
             y),
        radius / 4,
        radius / 4
    );

    fillellipse(
        xDev(worldBoundary,
             deviceBoundary,
             x - (cartHalfWidth / 2)),
        yDev(worldBoundary,
             deviceBoundary,
             y),
        radius / 4,
        radius / 4
    );
}


///////////////////////////////////////////////////////////////

void Cart::setX(const float& _x) {

    x = _x;
}