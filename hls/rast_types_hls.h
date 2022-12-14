#ifndef RAST_TYPES_HLS_H
#define RAST_TYPES_HLS_H

#include <ac_int.h>

#define SIG_FIG 24
#define RADIX 10

// Fixed point representation
typedef ac_int<SIG_FIG, true> SignedFixedPoint;
typedef ac_int<SIG_FIG, false> UnsignedFixedPoint;

typedef struct {
    UnsignedFixedPoint R;
    UnsignedFixedPoint G;
    UnsignedFixedPoint B;
} ColorHLS;

typedef struct {
    // Coordinates
    SignedFixedPoint x;
    SignedFixedPoint y;
    SignedFixedPoint z;
} Vertex3DHLS;

typedef struct {
    Vertex3DHLS v[3];

    // RGB
    UnsignedFixedPoint R;
    UnsignedFixedPoint G;
    UnsignedFixedPoint B;
} TriangleHLS;

typedef struct {
    SignedFixedPoint x;
    SignedFixedPoint y;
} Vertex2DHLS;

typedef struct {
    SignedFixedPoint x;
    SignedFixedPoint y;

    // RGB
    UnsignedFixedPoint R;
    UnsignedFixedPoint G;
    UnsignedFixedPoint B;
} ColorVertex2DHLS;

typedef ColorVertex2DHLS SampleHLS;

typedef struct {
    Vertex2DHLS lower_left;
    Vertex2DHLS upper_right;
} BoundingBoxHLS;

typedef struct {
	SignedFixedPoint width; // width of screen (on x axis)
	SignedFixedPoint height; // height of screen (on y axis)
} ScreenHLS;

typedef struct {
    ac_int<4, false> subsample; // 4 bit unsigned integer
} ConfigHLS;

#endif
