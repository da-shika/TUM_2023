#include <walking_core/types.h>
#include <control_core/types.h>

int main()
{ 
  cc::CartesianPosition pose = cc::CartesianPosition::Zero();
  cc::Scalar min_x = -0.08;
  cc::Scalar max_x = 0.12;
  cc::Scalar min_y = -0.065;
  cc::Scalar max_y = 0.065;
  cc::Vector3 offset = cc::Vector3::Zero();

  cc::Contact contact(pose, min_x, max_x, min_y, max_y, offset);

  std::cout << "contact.pose=" << contact.pose().toString() << std::endl;
  std::cout << "contact.hullLocal=\n" << contact.verticesLocal() << std::endl;
  std::cout << "contact.verticesGlobal=\n" << contact.verticesGlobal() << std::endl;
  std::cout << "contact.target=" << contact.target().toString() << std::endl;
  std::cout << "contact.minVertexWorld=" << contact.minVertexWorld().toString();
  std::cout << "contact.maxVertexWorld=" << contact.maxVertexWorld().toString();

  return 0;
}