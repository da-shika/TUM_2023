#include <control_core/geometry/shapes.h>

// test line fitting through points
int main()
{  
  cc::Points2d points(2, 4);
  points << 1, 1, 2, 3,
            1, 2, 4, 5;

  cc::LineShape line;
  line.fit(points);

  std::cout << "start: " << line.start().transpose() << std::endl;
  std::cout << "end: " << line.end().transpose() << std::endl;

  return 0;
}