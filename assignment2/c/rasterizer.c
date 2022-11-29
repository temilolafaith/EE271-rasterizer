#include "rasterizer.h"
#include <stdlib.h>

#ifdef __cplusplus
#include <vector>
#endif

/* Utility Functions */

/*
 *   Function: min
 *  Function Description: Returns the minimum value of two integers and b.
*/
int min(int a, int b)
{
  return a<b?a:b;
}

/*
 *   Function: max
 *   Function Description: Returns the maximum value of two integers and b.
*/
int max(int a, int b)
{
  return a>b?a:b;
}

/*
/   Function: floor_ss
/   Function Description: Returns a fixed point value rounded down to the subsample grid.
*/
int floor_ss(int val, int r_shift, int ss_w_lg2)
{
  return (val >> (r_shift - ss_w_lg2)) << (r_shift - ss_w_lg2);
}

/*
 *  Function: rastBBox_bbox_fix
 *  Function Description: Determine a bounding box for the triangle.
 *  Note that this is a fixed point function.
*/
BoundingBox get_bounding_box(Triangle triangle, Screen screen, Config config)
{
  BoundingBox bbox;

  // initialize bounding box to first vertex
  bbox.lower_left.x = triangle.v[0].x;
  bbox.lower_left.y = triangle.v[0].y;
  bbox.upper_right.x = triangle.v[0].x;
  bbox.upper_right.y = triangle.v[0].y;

  // iterate over remaining vertices

  for (int vertex = 1; vertex < 3; vertex++)
  {
      bbox.upper_right.x = max(bbox.upper_right.x, triangle.v[vertex].x);
      bbox.upper_right.y = max(bbox.upper_right.y, triangle.v[vertex].y);
      bbox.lower_left.x = min(bbox.lower_left.x, triangle.v[vertex].x);
      bbox.lower_left.y = min(bbox.lower_left.y, triangle.v[vertex].y);
  }


  // round down to subsample grid

  bbox.upper_right.x = floor_ss(bbox.upper_right.x, config.r_shift, config.ss_w_lg2);
  bbox.upper_right.y = floor_ss(bbox.upper_right.y, config.r_shift, config.ss_w_lg2);
  bbox.lower_left.x = floor_ss(bbox.lower_left.x, config.r_shift, config.ss_w_lg2);
  bbox.lower_left.y = floor_ss(bbox.lower_left.y, config.r_shift, config.ss_w_lg2);

  // clip to screen
  bbox.upper_right.x = min(bbox.upper_right.x, screen.width);
  bbox.upper_right.y = min(bbox.upper_right.y, screen.height);
  bbox.lower_left.x = max(bbox.lower_left.x, 0);
  bbox.lower_left.y = max(bbox.lower_left.y, 0);

  // check if bbox is valid
  bbox.valid = (bbox.upper_right.x >= 0) && (bbox.upper_right.y >= 0) && (bbox.lower_left.x < screen.width ) && (bbox.lower_left.y < screen.height);
  
  return bbox;
}

/*
 *  Function: sample_test
 *  Function Description: Checks if sample lies inside triangle
 *
 *
 */
bool sample_test(Triangle triangle, Sample sample)
{
  bool isHit;

  // START CODE HERE
  Triangle shifted_triangle;
  for (int vertex = 0; vertex < 3; vertex++)
  { // iterate over vertices
    shifted_triangle.v[vertex].x = triangle.v[vertex].x - sample.x;
    shifted_triangle.v[vertex].y = triangle.v[vertex].y - sample.y;
  }

  int distances[3];
  distances[0] = (shifted_triangle.v[0].x * shifted_triangle.v[1].y) -
                 (shifted_triangle.v[1].x * shifted_triangle.v[0].y);
  distances[1] = (shifted_triangle.v[1].x * shifted_triangle.v[2].y) -
                 (shifted_triangle.v[2].x * shifted_triangle.v[1].y);
  distances[2] = (shifted_triangle.v[2].x * shifted_triangle.v[0].y) -
                 (shifted_triangle.v[0].x * shifted_triangle.v[2].y);

  bool tests[3];
  tests[0] = distances[0] <= 0.0;
  tests[1] = distances[1] < 0.0;
  tests[2] = distances[2] <= 0.0;

  isHit = tests[0] && tests[1] && tests[2];

  return isHit;
}

int rasterize_triangle(Triangle triangle, ZBuff *z, Screen screen, Config config)
{
  int hit_count = 0;

  //Calculate BBox
  BoundingBox bbox = get_bounding_box(triangle, screen, config);

  //Iterate over samples and test if in triangle
  Sample sample;
  for (sample.x = bbox.lower_left.x; sample.x <= bbox.upper_right.x; sample.x += config.ss_i)
  {
    for (sample.y = bbox.lower_left.y; sample.y <= bbox.upper_right.y; sample.y += config.ss_i)
    {

      Sample jitter = jitter_sample(sample, config.ss_w_lg2);
      jitter.x = jitter.x << 2;
      jitter.y = jitter.y << 2;

      Sample jittered_sample;
      jittered_sample.x = sample.x + jitter.x;
      jittered_sample.y = sample.y + jitter.y;

      bool hit = sample_test(triangle, jittered_sample);

      if (hit)
      {
        hit_count++;
        if (z != NULL)
        {
          Sample hit_location;
          hit_location.x = sample.x >> config.r_shift;
          hit_location.y = sample.y >> config.r_shift;

          Sample subsample;
          subsample.x = (sample.x - (hit_location.x << config.r_shift)) / config.ss_i;
          subsample.y = (sample.y - (hit_location.y << config.r_shift)) / config.ss_i;

          Fragment f;
          f.z = triangle.v[0].z;
          f.R = triangle.v[0].R;
          f.G = triangle.v[0].G;
          f.B = triangle.v[0].B;

          process_fragment(z, hit_location, subsample, f);
        }
      }
    }
  }

  return hit_count;
}

void hash_40to8(uchar *arr40, ushort *val, int shift)
{
  uchar arr32[4];
  uchar arr16[2];
  uchar arr8;

  ushort mask = 0x00ff;
  mask = mask >> shift;

  arr32[0] = arr40[0] ^ arr40[1];
  arr32[1] = arr40[1] ^ arr40[2];
  arr32[2] = arr40[2] ^ arr40[3];
  arr32[3] = arr40[3] ^ arr40[4];

  arr16[0] = arr32[0] ^ arr32[2];
  arr16[1] = arr32[1] ^ arr32[3];

  arr8 = arr16[0] ^ arr16[1];

  mask = arr8 & mask;
  val[0] = mask;
}

Sample jitter_sample(const Sample sample, const int ss_w_lg2)
{
  long x = sample.x >> 4;
  long y = sample.y >> 4;
  uchar arr40_1[5];
  uchar arr40_2[5];

  long *arr40_1_ptr = (long *)arr40_1;
  long *arr40_2_ptr = (long *)arr40_2;

  ushort val_x[1];
  ushort val_y[1];

  *arr40_1_ptr = (y << 20) | x;
  *arr40_2_ptr = (x << 20) | y;

  hash_40to8(arr40_1, val_x, ss_w_lg2);
  hash_40to8(arr40_2, val_y, ss_w_lg2);

  Sample jitter;
  jitter.x = val_x[0];
  jitter.y = val_y[0];

  return jitter;
}
