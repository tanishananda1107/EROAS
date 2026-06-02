/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "selection_buffer/MaterialSwitcher.hh"

#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
namespace gazebo
{
namespace rendering
{
//////////////////////////////////////////////////
MaterialSwitcher::MaterialSwitcher()
{
  this->Reset();
}

//////////////////////////////////////////////////
MaterialSwitcher::~MaterialSwitcher()
{
}

//////////////////////////////////////////////////
gz::rendering::MaterialPtr MaterialSwitcher::GetSelectionMaterial(
    gz::rendering::ScenePtr _scene, gz::rendering::VisualPtr _visual)
{
  if (!_scene || !_visual)
  {
    return gz::rendering::MaterialPtr();
  }

  // If the visual is the same as the last processed one, reuse its material
  if (this->lastEntity == _visual->Name())
  {
    std::string matName = "selection_mat_" + std::to_string(this->currentColor.AsRGBA());
    gz::rendering::MaterialPtr mat = _scene->Material(matName);
    if (mat)
    {
      mat->SetAmbient(this->currentColor);
      mat->SetDiffuse(this->currentColor);
      return mat;
    }
  }

  // Iterate to next unique visual tracking color channel 
  this->GetNextColor();

  std::string matName = "selection_mat_" + std::to_string(this->currentColor.AsRGBA());
  gz::rendering::MaterialPtr newMat = _scene->CreateMaterial(matName);

  if (newMat)
  {
    newMat->SetAmbient(this->currentColor);
    newMat->SetDiffuse(this->currentColor);

    // Map unique color to visual entity identifier name
    this->lastEntity = _visual->Name();
    this->colorDict[this->currentColor.AsRGBA()] = this->lastEntity;
    return newMat;
  }

  return gz::rendering::MaterialPtr();
}

//////////////////////////////////////////////////
const std::string &MaterialSwitcher::GetEntityName(
    const gz::math::Color &_color) const
{
  auto iter = this->colorDict.find(_color.AsRGBA());

  if (iter != this->colorDict.end())
    return iter->second;

  return this->emptyString;
}

//////////////////////////////////////////////////
void MaterialSwitcher::GetNextColor()
{
  auto color = this->currentColor.AsARGB();
  color++;
  this->currentColor.SetFromARGB(color);
}

//////////////////////////////////////////////////
void MaterialSwitcher::Reset()
{
  this->currentColor = gz::math::Color(0.0f, 0.0f, 0.1f);
  this->lastEntity.clear();
  this->colorDict.clear();
}
}
}
