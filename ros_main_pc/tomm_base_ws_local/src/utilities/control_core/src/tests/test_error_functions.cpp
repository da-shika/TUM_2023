#include <control_core/math.h>
#include <control_core/types.h>

// test cubic spline for arbitary number of points
int main()
{  
  cc::AngularPosition Q_d_w = cc::Rotation3::RPY(1.0, -0.9, 1.4);

  cc::Vector3 err_vec_d(0.1, -0.2, 0.05);
  cc::AngularPosition Q_err_d = cc::Rotation3::RPY(err_vec_d);
  cc::AngularPosition Q_cur_w = Q_d_w*Q_err_d;

  cc::Vector3 err_vec_w = Q_d_w*err_vec_d;
  cc::Vector3 err_vec_cur = Q_cur_w.inverse()*Q_d_w*err_vec_d;

  // computes the error vector wrt to world frame w
  cc::Vector3 err_w = cc::logErrorWorld(Q_d_w, Q_cur_w);

  // computes the error vector wrt to current frame cur
  cc::Vector3 err_d = cc::logErrorBody(Q_d_w, Q_cur_w);

  std::cout << "err wrt d=  " << err_vec_d.toString() << std::endl;
  std::cout << "err wrt cur=" << err_vec_cur.toString() << std::endl;
  std::cout << "err wrt w=  " << err_vec_w.toString() << std::endl;
  
  std::cout << "err computed w=   " << err_w.toString() << std::endl;
  std::cout << "err computed cur= " << err_d.toString() << std::endl;

  return 0;
}