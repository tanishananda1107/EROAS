#include "GaussMarkovProcess.hh"
#include <iostream>
#include <cmath>
#include <chrono>

namespace uuv_gz_sim
{

GaussMarkovProcess::GaussMarkovProcess()
: rng(std::random_device{}()),
  dist(-0.5, 0.5)
{
  Reset();
}

void GaussMarkovProcess::Reset()
{
  var = mean;
}

bool GaussMarkovProcess::SetMean(double _mean)
{
  if (_mean < min || _mean > max)
    return false;

  mean = _mean;
  Reset();
  return true;
}

bool GaussMarkovProcess::SetModel(double _mean, double _min, double _max,
                                  double _mu, double _noise)
{
  if (_min >= _max || _mean < _min || _mean > _max)
    return false;
  if (_mu < 0 || _mu > 1)
    return false;
  if (_noise < 0)
    return false;

  mean = _mean;
  min = _min;
  max = _max;
  mu = _mu;
  noiseAmp = _noise;

  Reset();
  return true;
}

double GaussMarkovProcess::Update(double _time)
{
  double step = _time - lastUpdate;

  double random = dist(rng);

  var = (1.0 - step * mu) * var + noiseAmp * random;

  if (var > max) var = max;
  if (var < min) var = min;

  lastUpdate = _time;
  return var;
}

void GaussMarkovProcess::Print() const
{
  std::cout
    << "Mean: " << mean << "\n"
    << "Min: " << min << "\n"
    << "Max: " << max << "\n"
    << "Mu: " << mu << "\n"
    << "Noise: " << noiseAmp << std::endl;
}

} // namespace uuv_gz_sim
