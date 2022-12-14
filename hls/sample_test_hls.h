#ifndef SAMPLE_TEST_HLS
#define SAMPLE_TEST_HLS
#include "rast_types_hls.h"

#pragma hls_design
class SampleTest{
public:
    SampleTest(){}

    #pragma hls_design interface ccore
    bool CCS_BLOCK(run)(TriangleHLS triangle, SampleHLS sample)
    {
        bool isHit;
        TriangleHLS shifted_triangle;
        // START CODE HERE
        for (int vertex = 0; vertex < 3; vertex++) { // iterate over vertices
            shifted_triangle.v[vertex].x = triangle.v[vertex].x - sample.x;
            shifted_triangle.v[vertex].y = triangle.v[vertex].y - sample.y;
        }

        int distances[3];
        distances[0] = (shifted_triangle.v[0].x * shifted_triangle.v[1].y) - (shifted_triangle.v[1].x * shifted_triangle.v[0].y);
        distances[1] = (shifted_triangle.v[1].x * shifted_triangle.v[2].y) - (shifted_triangle.v[2].x * shifted_triangle.v[1].y);
        distances[2] = (shifted_triangle.v[2].x * shifted_triangle.v[0].y) - (shifted_triangle.v[0].x * shifted_triangle.v[2].y);

        bool tests[3];
        tests[0] = distances[0] <= 0.0;
        tests[1] = distances[1] < 0.0;
        tests[2] = distances[2] <= 0.0;

        isHit = tests[0] && tests[1] && tests[2];
        // END CODE HERE
        return isHit;
    }
};

#endif
