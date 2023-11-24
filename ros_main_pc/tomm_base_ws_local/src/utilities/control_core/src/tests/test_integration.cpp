#include <control_core/math.h>
#include <control_core/types.h>

// test cubic spline for arbitary number of points
int main()
{  
  cc::CartesianState state = cc::CartesianState::Zero();

  state.acc().linear().z() = 1;   // body has trust in its z axis (in body frame!)
  state.acc().angular().z() = 1;  // body has torque around z axis

  for(std::size_t i = 0; i < 100; ++i)
  {
    cc::Scalar t = i*0.01;
    cc::integrateStateBody(state, state, 0.01);
    std::cout << "t=" << t << "\n" << state.toString();
  }

  return 0;
}