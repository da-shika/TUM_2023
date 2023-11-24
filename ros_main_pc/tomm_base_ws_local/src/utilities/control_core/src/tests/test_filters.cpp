#include <control_core/algorithms.h>

#include <control_core/test_utilities/plot.h>


void build_sigmal_signal(
  std::vector<double>& time_sig, std::vector<double>& test_sig, 
  double f_sample, int n)
{
  std::vector<double> zeros(n/2,0);
  std::vector<double> ones(n/2,1);

  test_sig.insert(test_sig.end(), zeros.begin(), zeros.end());
  test_sig.insert(test_sig.end(), ones.begin(), ones.end());

  time_sig.resize(test_sig.size());
  for(size_t i = 0; i < test_sig.size(); ++i)
    time_sig[i] = i/f_sample;
}

void build_sinusoidal_signal(
  std::vector<double>& time_sig, std::vector<double>& test_sig, 
  double f_sample, double f1, double f2, int n)
{
  time_sig.resize(n); test_sig.resize(n);
  for(int i = 0; i < n; ++i) 
  {
    time_sig[i] = i/f_sample;
    test_sig[i] = 2.0*std::cos(2*M_PI*f1*time_sig[i]) 
      + 4.0*std::sin(2*M_PI*f2*time_sig[i] + 0.2);
  }
}

std::vector<double> filter_signal(control_core::IScalarAlgorithm<cc::Scalar>& filter, const std::vector<double>& sig)
{
  std::vector<double> out(sig.size());
  for(int i = 0; i < sig.size(); ++i)
    out[i] = filter.update(sig[i]);
  return out;
}

void test_butter_worth_filter()
{
  // ---------------------------------------------------------------------------
  // Signal
  // ---------------------------------------------------------------------------
  double f_sample = 200;
  std::vector<double> time_sig, test_sig; 
  //build_sigmal_signal(time_sig, test_sig, f_sample, 600);
  build_sinusoidal_signal(time_sig, test_sig, f_sample, 0.5, 10, 1000);

  // ---------------------------------------------------------------------------
  // filter
  // ---------------------------------------------------------------------------
  double f_cutoff = 5;
  std::vector<std::shared_ptr<cc::ScalarButterWorthFilter> > filters = {
    std::make_shared<cc::ScalarButterWorthFilter>(cc::ScalarButterWorthFilter::LowPassSecondOrder(f_sample, f_cutoff)),
    std::make_shared<cc::ScalarButterWorthFilter>(cc::ScalarButterWorthFilter::HighPassSecondOrder(f_sample, f_cutoff))
  };
  std::vector<std::string> names = {"lp", "hp"};

  for(size_t i = 0; i < filters.size(); ++i)
  {
    auto res = filter_signal(*filters[i], test_sig);
    cc_test::plot(time_sig, res, names[i], "butter_worth_filter", "--", true, false, 1);
  }
  cc_test::plot(time_sig, test_sig, "input", "butter_worth_filter", "--", true, true, 1);
}

void test_fri_filter()
{
  // ---------------------------------------------------------------------------
  // Signal
  // ---------------------------------------------------------------------------
  double f_sample = 200;
  std::vector<double> time_sig, test_sig; 
  //build_sigmal_signal(time_sig, test_sig, f_sample, 600);
  build_sinusoidal_signal(time_sig, test_sig, f_sample, 0.5, 10, 1000);

  // ---------------------------------------------------------------------------
  // filter
  // ---------------------------------------------------------------------------
  double f_cutoff = 5;
  int order = 51;
  std::vector<std::shared_ptr<cc::ScalarFIRFilter> > filters = {
    std::make_shared<cc::ScalarFIRFilter>(cc::ScalarFIRFilter::HanningFRI(f_sample, f_cutoff, order)),
    std::make_shared<cc::ScalarFIRFilter>(cc::ScalarFIRFilter::BarlettFRI(f_sample, f_cutoff, order)),
    std::make_shared<cc::ScalarFIRFilter>(cc::ScalarFIRFilter::BlackmanFRI(f_sample, f_cutoff, order)),
    std::make_shared<cc::ScalarFIRFilter>(cc::ScalarFIRFilter::RectFRI(f_sample, f_cutoff, order)),
    std::make_shared<cc::ScalarFIRFilter>(cc::ScalarFIRFilter::HammingFRI(f_sample, f_cutoff, order))
  };
  std::vector<std::string> names = {"Barlett", "Blackman", "Rect", "Hamming", "Hanning"};

  for(size_t i = 0; i < filters.size(); ++i)
  {
    auto res = filter_signal(*filters[i], test_sig);
    cc_test::plot(time_sig, res, names[i], "fir", "-", true, false, 1);
  }
  cc_test::plot(time_sig, test_sig, "input", "fir", "--", true, true, 1);
}


int main()
{
  test_butter_worth_filter();
  //test_fri_filter();

  return 0;
}
