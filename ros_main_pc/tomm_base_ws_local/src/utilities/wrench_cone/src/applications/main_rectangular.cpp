#include <control_core/types.h>
#include <wrench_cone/wrench_cone_solver.h>

/**
 * @brief Test the rectangularContactHRep function
 * 
 * @return int 
 */
int main()
{
  double X = 0.2;
  double Y = 0.1;

  cc::Contact contact = cc::Contact::Zero();
  contact.friction() = 0.7/std::sqrt(2);
  contact.hull() = cc::PolygonShape(X, Y);

  if(contact.hull().isRectangular())
    ROS_INFO_STREAM("RECTANGULAR");
  else
    ROS_INFO_STREAM("NOT RECTANGULAR");

  wrench_cone::WrenchConeSolver solver;
  solver.compute(contact);

  ROS_INFO_STREAM("GraspMatrix=\n" << contact.graspMatrix());
  ROS_INFO_STREAM("Generators=\n" << contact.rays());
  ROS_INFO_STREAM("HalfSpace=\n" << contact.halfSpace());

  auto he = contact.hull().halfExtends();
  auto H_rect = cc::rectangularContactHRep(contact.friction(), he.x(), he.y());

  ROS_INFO_STREAM("H Rectangular=\n" << H_rect);

  return 0;
}