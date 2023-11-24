#include <control_core/geometry/geometry_2d.h>


// test cubic spline for arbitary number of points
int main()
{  
  cc::Point2d p1, p2, p3, p4;
  p1 << 0.0, 0.0;
  p2 << 1.0, 0.0;
  p3 << 0.5, -0.5;
  p4 << 0.5, 0.5;

  // cc::LineShape line1(p1, p2);
  // cc::LineShape line2(p3, p4);

  std::cout << cc::distance_segment_segment(p1, p2, p3, p4) << std::endl;

  return 0;
}