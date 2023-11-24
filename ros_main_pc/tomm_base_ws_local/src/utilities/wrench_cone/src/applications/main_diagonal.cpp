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
  double Y = 0.12;
  double W = 0.06/sqrt(2);
  cc::Points2d vertices(2,6);
  vertices << 
    -X, -X+W, X, X, X-W, -X,
    -Y, -Y, Y-W, Y, Y, -Y+W;
  
  ROS_INFO_STREAM("vertices=" << vertices);
  
  cc::Contact contact = cc::Contact::Zero();
  contact.friction() = 0.7/std::sqrt(2);
  contact.hull().vertices() = vertices;

  wrench_cone::WrenchConeSolver solver;
  solver.compute(contact);

  if(contact.hull().isRectangular())
    ROS_INFO_STREAM("RECTANGULAR");
  else
    ROS_INFO_STREAM("NOT RECTANGULAR");

  ROS_INFO_STREAM("GraspMatrix=\n" << contact.graspMatrix());
  ROS_INFO_STREAM("Generators=\n" << contact.rays());
  ROS_INFO_STREAM("HalfSpace=\n" << contact.halfSpace());

  auto he = contact.hull().halfExtends();
  auto H_rect = cc::rectangularContactHRep(contact.friction(), he.x(), he.y());

  ROS_INFO_STREAM("H Rectangular=\n" << H_rect);

  // test if we can copy the solution
  cc::Contact contact2 = contact;

  ROS_INFO_STREAM("Copy GraspMatrix=\n" << contact.graspMatrix());
  ROS_INFO_STREAM("Copy Generators=\n" << contact.rays());
  ROS_INFO_STREAM("Copy HalfSpace=\n" << contact.halfSpace());

  return 0;
}