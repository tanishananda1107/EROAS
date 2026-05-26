#ifndef UUV_GZ_SIM_GAUSS_MARKOV_PROCESS_HH_
#define UUV_GZ_SIM_GAUSS_MARKOV_PROCESS_HH_

#include <random>
#include <string>

namespace uuv_gz_sim
{

class GaussMarkovProcess
{
public:
  GaussMarkovProcess() { this->Reset(); }

  void Reset()
  {
    this->var = this->mean;
    this->lastUpdate = 0.0;
  }

  bool SetModel(double _mean, double _min, double _max,
                double _mu = 0.0, double _noise = 0.0)
  {
    if (_min > _max)
      return false;

    mean = _mean;
    min = _min;
    max = _max;
    mu = _mu;
    noiseAmp = _noise;

    this->var = mean;
    return true;
  }

  bool SetMean(double _mean)
  {
    if (_mean < min || _mean > max)
      return false;

    mean = _mean;
    return true;
  }

  double Update(double _time)
  {
    double dt = _time - lastUpdate;
    lastUpdate = _time;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> dist(0.0, noiseAmp);

    double noise = dist(gen);

    var += mu * (mean - var) * dt + noise;

    if (var > max) var = max;
    if (var < min) var = min;

    return var;
  }

  void Print() {}

public:
  double var;
  double mean;
  double min;
  double max;
  double mu;
  double noiseAmp;
  double lastUpdate;
};

} // namespace uuv_gz_sim

#endif
