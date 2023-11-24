#include <wrench_cone/wrench_cone_solver.h>

namespace wrench_cone
{
  WrenchConeSolver::WrenchConeSolver()
  {
  }

  WrenchConeSolver::~WrenchConeSolver()
  {
  }

  void WrenchConeSolver::compute(cc::Contact& contact)
  {
    auto& hull = contact.hull();
    size_t n = hull.vertices().cols();

    auto& G = contact.graspMatrix();
    auto& V = contact.rays();
    auto& H = contact.halfSpace();

    // compute grasp matrix
    G.resize(6, n*3);
    cc::Vector3 p;
    for(size_t i = 0; i < n; ++i)
    {
      p << hull.vertices().col(i), 0;
      G.block<6,3>(0, i*3) = cc::pointContactGraspMatrix(p);
    }

    // compute vertices
    V.setZero(n*4, n*3);
    for(size_t i = 0; i < n; ++i)
    {
      V.block<4,3>(i*4, i*3) = cc::pointContactVRep(contact.friction());
    }

    if(!hull.isRectangular())
    {
      // no closed form solution, use double description

      // compute halfspace
      Eigen::VectorXd v = Eigen::VectorXd::Zero(V.rows());
      Eigen::MatrixXd V_grasp = V*G.transpose();
      poly_.setVRepresentation(V_grasp, v);
      Eigen::VectorXd h; 
      poly_.hRepresentation(H, v);
      H = (-1)*H;
    }
    else
    {
      // closed form solution available, use instead
      auto he = contact.hull().halfExtends();
      H = cc::rectangularContactHRep(contact.friction(), he.x(), he.y());
    }
  }

}