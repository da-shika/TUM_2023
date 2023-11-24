#include <control_core/algorithms.h>
#include <control_core/test_utilities/plot.h>

void build_trajectory_signal(
  std::vector<double>& time_sig, std::vector<cc::Vector3>& eul, 
  std::vector<cc::AngularPosition>& quat, double f_sample, double f_sig, double f_noise, int n)
{
  time_sig.resize(n); eul.resize(n); quat.resize(n);

  double omega_sig = 2*M_PI*f_sig;
  double omega_noise = 2*M_PI*f_noise;
  for(size_t i = 0; i < n; ++i)
  {
    time_sig[i] = i/f_sample;
    eul[i] << 
      M_PI/4 * std::sin(omega_sig*time_sig[i]),
      M_PI/8,
      -M_PI/12 * std::sin(omega_noise*time_sig[i]);
    quat[i] = cc::AngularPosition(eul[i].x(), eul[i].y(), eul[i].z());
  }
}

void add_noise(
  const std::vector<cc::Vector3>& eul,
  std::vector<cc::Vector3>& eul_noise,
  std::vector<cc::AngularPosition>& quat_noise,
  double std_dev)
{
  eul_noise.resize(eul.size()); quat_noise.resize(eul.size());
  for(size_t i = 0; i < eul.size(); ++i)
  {
    eul_noise[i] = eul[i] + std_dev*cc::Vector3::Random();
    quat_noise[i] = cc::AngularPosition(eul_noise[i].x(), eul_noise[i].y(), eul_noise[i].z());
  }
}

void quat_to_vec(
  const std::vector<cc::AngularPosition>& quat, 
  std::vector<std::vector<double> >& vecs)
{
  for(size_t i = 0; i < 4; ++i)
    vecs.push_back(std::vector<double>(quat.size()));
  for(size_t i = 0; i < quat.size(); ++i)
  {
    vecs[0][i] = quat[i].w();
    vecs[1][i] = quat[i].x();
    vecs[2][i] = quat[i].y();
    vecs[3][i] = quat[i].z();
  }
}

void plot_quad(
  const std::vector<double>& time_sig, 
  const std::vector<cc::AngularPosition>& quat_in,
  const std::vector<cc::AngularPosition>& quat_noise, 
  const std::vector<cc::AngularPosition>& quat_out)
{
  std::vector<std::vector<double> > vecs_in;
  quat_to_vec(quat_in, vecs_in);

  std::vector<std::vector<double> > vecs_noise;
  quat_to_vec(quat_noise, vecs_noise);

  std::vector<std::vector<double> > vecs_out;
  quat_to_vec(quat_out, vecs_out);

  std::vector<std::string> names = {"w", "x", "y", "z"};
  for(size_t i = 0; i < 4; ++i)
    cc_test::plot(time_sig, vecs_in[i], "in_"+names[i], "quat_filter", "--", true, false, 1);
  for(size_t i = 0; i < 4; ++i)
    cc_test::plot(time_sig, vecs_noise[i], "noise_"+names[i], "quat_filter", "-.", true, false, 1);
  for(size_t i = 0; i < 4; ++i)
    cc_test::plot(time_sig, vecs_out[i], "out_"+names[i], "quat_filter", "-", true, false, 1);
  cc_test::plt::show();
}

int main()
{
  double f_sample = 200.;
  double f_signal = 2.;
  double f_noise = 6*f_signal;
  double n = 2.0 * 200;

  std::vector<double> time_sig; 
  std::vector<cc::Vector3> eul;
  std::vector<cc::AngularPosition> quat;
  build_trajectory_signal(time_sig, eul, quat, f_sample, f_signal, f_noise, n);

  std::vector<cc::Vector3> eul_noise;
  std::vector<cc::AngularPosition> quat_noise;
  add_noise(eul, eul_noise, quat_noise, 0.1);
  
  double f_cutoff = 4.0;
  int order = 11;
  cc::QuaternionFilter filter(
    cc::ScalarFIRFilter::HanningFRI(f_sample, f_cutoff, order));
  
  std::vector<cc::AngularPosition> quat_out(quat_noise.size());
  for(size_t i = 0; i < quat_noise.size(); ++i)
    quat_out[i] = filter.update(quat_noise[i]);

  plot_quad(time_sig, quat, quat_noise, quat_out);
  return 0;
}