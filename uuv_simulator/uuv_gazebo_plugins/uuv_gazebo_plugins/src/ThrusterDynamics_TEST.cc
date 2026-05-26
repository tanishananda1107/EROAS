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
//   - SDF_VERSION fallback guard (same as in ConversionFcn test).
//   - DynamicsFactory and Dynamics live in gz::sim:: now.
//   - sdf::SDF / sdf::ElementPtr API unchanged (sdformat13).
//   - NULL  →  nullptr throughout.
//   - EXPECT_TRUE(x != NULL)  →  ASSERT_NE(x, nullptr)
//     (ASSERT stops the test immediately on failure, avoiding UB if ptr is
//      null and subsequent EXPECT_EQ calls dereference it).

#include <string>
#include <memory>
#include <sstream>

#include <gtest/gtest.h>

// sdformat13
#include <sdf/sdf.hh>

// Plugin header (update namespace in the header to gz::sim as well)
#include <uuv_gazebo_plugins/Dynamics.hh>

#ifndef SDF_VERSION
#define SDF_VERSION "1.9"
#endif

/// \brief Helper: parse an SDF snippet and return a Dynamics object.
static std::shared_ptr<gz::sim::Dynamics>
DynamicsFromString(const std::string& description)
{
  std::ostringstream stream;
  stream << "<sdf version='" << SDF_VERSION << "'>\n"
         << "<model name='test_model'>\n"
         << "<plugin name='test_plugin' filename='test_file.so'>\n"
         << description << "\n"
         << "</plugin>\n"
         << "</model>\n"
         << "</sdf>\n";

  sdf::SDF sdfParsed;
  sdfParsed.SetFromString(stream.str());

  sdf::ElementPtr dynSdf =
      sdfParsed.Root()
               ->GetElement("model")
               ->GetElement("plugin")
               ->GetElement("dynamics");

  std::shared_ptr<gz::sim::Dynamics> dyn;
  dyn.reset(
      gz::sim::DynamicsFactory::GetInstance()
              .CreateDynamics(dynSdf));

  return dyn;
}

//////////////////////////////////////////////////
TEST(ThrusterDynamics, ZeroOrder)
{
  const std::string description =
      "<dynamics>\n"
      "  <type>ZeroOrder</type>\n"
      "</dynamics>";

  auto dyn = DynamicsFromString(description);

  ASSERT_NE(dyn, nullptr);
  EXPECT_EQ(dyn->GetType(), "ZeroOrder");

  EXPECT_EQ(10.0, dyn->update(10.0, 0.0));
  EXPECT_EQ(20.0, dyn->update(20.0, 0.2));
}

//////////////////////////////////////////////////
TEST(ThrusterDynamics, FirstOrder)
{
  const std::string description =
      "<dynamics>\n"
      "  <type>FirstOrder</type>\n"
      "  <timeConstant>0.5</timeConstant>\n"
      "</dynamics>";

  auto dyn = DynamicsFromString(description);

  ASSERT_NE(dyn, nullptr);
  EXPECT_EQ(dyn->GetType(), "FirstOrder");

  // At t=0 the output must be zero.
  EXPECT_EQ(0.0, dyn->update(0.0, 0.0));

  // After one time-constant (dt=0.5 s, τ=0.5 s), step-response ≈ 1 − e⁻¹.
  EXPECT_NEAR(1.0 - 0.36787944, dyn->update(1.0, 0.5), 1e-5);
}

//////////////////////////////////////////////////
TEST(ThrusterDynamics, Yoerger)
{
  const std::string description =
      "<dynamics>\n"
      "  <type>Yoerger</type>\n"
      "  <alpha>0.5</alpha>\n"
      "  <beta>0.5</beta>\n"
      "</dynamics>";

  auto dyn = DynamicsFromString(description);

  ASSERT_NE(dyn, nullptr);
  EXPECT_EQ(dyn->GetType(), "Yoerger");

  // Zero input → zero output.
  EXPECT_EQ(0.0, dyn->update(0.0, 0.0));

  // TODO: add a quantitative test of the Yoerger model transient response.
}

//////////////////////////////////////////////////
TEST(ThrusterDynamics, Bessa)
{
  const std::string description =
      "<dynamics>\n"
      "  <type>Bessa</type>\n"
      "  <Jmsp>0.5</Jmsp>\n"
      "  <Kv1>0.5</Kv1>\n"
      "  <Kv2>0.5</Kv2>\n"
      "  <Kt>0.5</Kt>\n"
      "  <Rm>0.5</Rm>\n"
      "</dynamics>";

  auto dyn = DynamicsFromString(description);

  ASSERT_NE(dyn, nullptr);
  EXPECT_EQ(dyn->GetType(), "Bessa");

  // Zero input → zero output.
  EXPECT_EQ(0.0, dyn->update(0.0, 0.0));

  // TODO: add a quantitative test of the Bessa motor model.
}

//////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
