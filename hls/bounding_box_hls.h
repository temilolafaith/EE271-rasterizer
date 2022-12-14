#ifndef BOUNDING_BOX_HLS
#define BOUNDING_BOX_HLS

#include "rast_types_hls.h"

#pragma hls_design 
class BoundingBoxGenerator{
public:
    BoundingBoxGenerator() {}

    #pragma hls_design interface
    void CCS_BLOCK(run)(
        ac_channel<TriangleHLS> &triangle_in, 
        ac_channel<ScreenHLS> &screen_in, 
        ac_channel<ConfigHLS> &config_in,
        ac_channel<BoundingBoxHLS> &bbox_out,
        ac_channel<TriangleHLS> &triangle_out,
        ac_channel<ConfigHLS> &config_out
    ){
        #ifndef __SYNTHESIS__
        while(triangle_in.available(1))
        #endif
        {
            TriangleHLS triangle = triangle_in.read();
            ScreenHLS screen = screen_in.read();
            ConfigHLS config = config_in.read();

            BoundingBoxHLS bbox;
            
            // START CODE HERE
            // initialize bounding box
            bbox.lower_left.x = triangle.v[0].x;
            bbox.lower_left.y = triangle.v[0].y;
            bbox.upper_right.x = triangle.v[0].x;
            bbox.upper_right.y = triangle.v[0].y;
            // iterate over remaining vertices
            for (int vertex = 1; vertex < 3; vertex++) {
                bbox.upper_right.x = max(bbox.upper_right.x, triangle.v[vertex].x);
                bbox.upper_right.y = max(bbox.upper_right.y, triangle.v[vertex].y);
                bbox.lower_left.x = min(bbox.lower_left.x, triangle.v[vertex].x);
                bbox.lower_left.y = min(bbox.lower_left.y, triangle.v[vertex].y);
            }
            // round down to subsample grid
            bbox.upper_right.x = floor_ss(bbox.upper_right.x, config);
            bbox.upper_right.y = floor_ss(bbox.upper_right.y, config);
            bbox.lower_left.x = floor_ss(bbox.lower_left.x, config);
            bbox.lower_left.y = floor_ss(bbox.lower_left.y, config);

            // clip to screen
            bbox.upper_right.x = min(bbox.upper_right.x, screen.width);
            bbox.upper_right.y = min(bbox.upper_right.y, screen.height);
            bbox.lower_left.x = max(bbox.lower_left.x, 0);
            bbox.lower_left.y = max(bbox.lower_left.y, 0);
            // check if bbox is valid
            bool valid;
            valid = (bbox.upper_right.x >= 0) && (bbox.upper_right.y >= 0) && (bbox.lower_left.x < screen.width ) && (bbox.lower_left.y < screen.height);
            // write to outputs if bbox is valid
            if (valid) {
                triangle_out.write(triangle);
                bbox_out.write(bbox);
                config_out.write(config);
            }
            // END CODE HERE
        }
    }
private:
    SignedFixedPoint min(SignedFixedPoint a, SignedFixedPoint b)
    {
        // START CODE HERE
        return (a < b) ? a : b;
        // END CODE HERE
    }

    SignedFixedPoint max(SignedFixedPoint a, SignedFixedPoint b)
    {
        // START CODE HERE
        return (a > b) ? a : b;
        // END CODE HERE
    }

    SignedFixedPoint floor_ss(SignedFixedPoint val, ConfigHLS config)
    {
        // START CODE HERE
        
        switch(config.subsample) {
            case 1: 
                val = val & 0b111111111111111110000000;
                break;
            case 2: 
                val = val & 0b111111111111111100000000;
                break;
            case 4: 
                val = val & 0b111111111111111000000000;
                break;
            case 8: 
                val = val & 0b111111111111110000000000;
                break;
        }
        return val;
        // END CODE HERE
    }
};

#endif
