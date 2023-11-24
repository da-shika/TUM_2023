#include <control_core/stats/distributions.h>
#include <iostream>

// test normal distribution
int main()
{  
  std::cout << "test distribution" << std::endl;

  double mu = 1.0;
  double std = 1.0;
  double x_min = mu - 3*std;
  double x_max = mu + 3*std;

  std::vector<double> xs;
  for(double x = x_min; x < x_max; x+=0.1)
    xs.push_back(x);

  for(auto x : xs)
    std::cout << "x=" << x << 
      " pdf(x)=" << cc::normal_pdf(mu, std, x) << 
      " cdf(x)=" << cc::normal_cdf(mu, std, x) << std::endl;

  return 0;
}