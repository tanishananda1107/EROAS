// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// GZ-SIM 8 (Gazebo Harmonic) port
// Changes from ROS1/Gazebo Classic:
//   - Namespace: gazebo::  →  gz::sim::
//   - SDF_VERSION macro (from gazebo) replaced by SDF_VERSION from sdformat13
//     which is still available via <sdf/sdf.hh>; if not defined locally,
//     fall back to "1.9" (sdformat13 default).
//   - sdf::SDF / sdf::ElementPtr API is unchanged (sdformat13 keeps compat).
//   - ConversionFunctionFactory and ConversionFunction live in gz::sim:: now.
//   - <cmath> std::abs for double math.

#include <string>
#include <memory>
#include <sstream>
#include <vector>
#include <cmath>

#include <gtest/gtest.h>

// sdformat13
#include <sdf/sdf.hh>

// Plugin header (update your include guard / namespace accordingly)
#include <uuv_gazebo_plugins/ThrusterConversionFcn.hh>

// SDF_VERSION may be defined by the sdformat CMake targets; provide a fallback.
#ifndef SDF_VERSION
#define SDF_VERSION "1.9"
#endif

/// \brief Helper: parse an SDF snippet and return the ConversionFunction.
static std::shared_ptr<gz::sim::ConversionFunction>
ConversionFromString(const std::string& description)
{
  std::stringstream stream;
  stream << "<sdf version='" << SDF_VERSION << "'>\n"
         << "<model name='test_model'>\n"
         << "<plugin name='test_plugin' filename='test_file.so'>\n"
         << description << "\n"
         << "</plugin>\n"
         << "</model>\n"
         << "</sdf>\n";

  sdf::SDF sdfParsed;
  sdfParsed.SetFromString(stream.str());

  sdf::ElementPtr conversion =
      sdfParsed.Root()
               ->GetElement("model")
               ->GetElement("plugin")
               ->GetElement("conversion");

  std::shared_ptr<gz::sim::ConversionFunction> func;
  func.reset(
      gz::sim::ConversionFunctionFactory::GetInstance()
              .CreateConversionFunction(conversion));

  return func;
}

//////////////////////////////////////////////////
TEST(ThrusterConversionFcn, Basic)
{
  const std::string description =
      "<conversion>\n"
      "  <type>Basic</type>\n"
      "  <rotorConstant>0.0049</rotorConstant>\n"
      "</conversion>";

  auto func = ConversionFromString(description);

  ASSERT_NE(func, nullptr);
  EXPECT_EQ(func->GetType(), "Basic");

  EXPECT_EQ(func->convert(0.0),  0.0);
  EXPECT_EQ(func->convert(50.),  50.0 * 50.0 * 0.0049);
  EXPECT_EQ(func->convert(-50.), -50.0 * 50.0 * 0.0049);
}

//////////////////////////////////////////////////
TEST(ThrusterConversionFcn, Bessa)
{
  const double cl    = 0.001;
  const double cr    = 0.002;
  const double dl    = -50.0;
  const double dr    =  25.0;
  const double delta = 1e-6;

  std::ostringstream stream;
  stream << "<conversion>\n"
         << "  <type>Bessa</type>\n"
         << "  <rotorConstantL>" << cl << "</rotorConstantL>\n"
         << "  <rotorConstantR>" << cr << "</rotorConstantR>\n"
         << "  <deltaL>"         << dl << "</deltaL>\n"
         << "  <deltaR>"         << dr << "</deltaR>\n"
         << "</conversion>";

  auto func = ConversionFromString(stream.str());

  ASSERT_NE(func, nullptr);
  EXPECT_EQ(func->GetType(), "Bessa");

  // Dead-zone and its boundaries.
  EXPECT_EQ(0.0, func->convert(0.0));
  EXPECT_EQ(0.0, func->convert( std::sqrt(dr)  - delta));
  EXPECT_EQ(0.0, func->convert(-std::sqrt(-dl) + delta));

  // Values outside the dead-zone.
  const double cmdl = -50.0;
  const double cmdr =  50.0;
  EXPECT_EQ(cl * (cmdl * std::abs(cmdl) - dl), func->convert(cmdl));
  EXPECT_EQ(cr * (cmdr * std::abs(cmdr) - dr), func->convert(cmdr));
}

//////////////////////////////////////////////////
TEST(ThrusterConversionFcn, LinearInterp)
{
  const std::vector<double> input  = {-5.0, 0.0, 2.0, 5.0};
  const std::vector<double> output = {-100.0, -10.0, 20.0, 120.0};
  const std::vector<double> alpha  = {0.1, 0.5, 0.9};

  std::ostringstream stream;
  stream << "<conversion>\n"
         << "  <type>LinearInterp</type>\n"
         << "  <inputValues>";
  for (double d : input)  stream << d << " ";
  stream << "</inputValues>\n"
         << "  <outputValues>";
  for (double d : output) stream << d << " ";
  stream << "</outputValues>\n"
         << "</conversion>";

  auto func = ConversionFromString(stream.str());

  ASSERT_NE(func, nullptr);
  EXPECT_EQ(func->GetType(), "LinearInterp");

  // Exact sample points must be reproduced perfectly.
  for (std::size_t i = 0; i < input.size(); ++i)
    EXPECT_EQ(output[i], func->convert(input[i]));

  // Outside defined range: clamp to nearest boundary value.
  EXPECT_EQ(output.front(), func->convert(input.front() - 0.5));
  EXPECT_EQ(output.back(),  func->convert(input.back()  + 0.5));

  // Between samples: verify linear interpolation.
  for (std::size_t i = 0; i < input.size() - 1; ++i)
  {
    const double a   = alpha[i];
    const double in  = a * input[i]  + (1.0 - a) * input[i + 1];
    const double out = a * output[i] + (1.0 - a) * output[i + 1];
    EXPECT_NEAR(out, func->convert(in), 1e-7);
  }
}

//////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
