#ifndef UUV_GZ_SIM_GAUSS_MARKOV_PROCESS_HH_
#define UUV_GZ_SIM_GAUSS_MARKOV_PROCESS_HH_

#include <random>
#include <string>

namespace uuv_gz_sim
{

class GaussMarkovProcess
{
public:
  GaussMarkovProcess();

  void Reset();

  bool SetModel(double _mean, double _min, double _max,
                double _mu = 0.0, double _noise = 0.0);

  bool SetMean(double _mean);

  double Update(double _time);

  void Print() const;

public:
  double var{0.0};
  double mean{0.0};
  double min{0.0};
  double max{0.0};
  double mu{0.0};
  double noiseAmp{0.0};
  double lastUpdate{0.0};

private:
  std::mt19937 rng;
  std::uniform_real_distribution<double> dist;
};

} // namespace uuv_gz_sim

#endif
