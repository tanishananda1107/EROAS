/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Updated for Gazebo Harmonic / gz-sim8
 */

#include "MaterialSwitcher.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
MaterialSwitcher::MaterialSwitcher()
{
  this->currentColor =
      gz::math::Color(0.0f, 0.0f, 0.1f, 1.0f);
}

/////////////////////////////////////////////////
MaterialSwitcher::~MaterialSwitcher()
{
  this->Reset();
}

/////////////////////////////////////////////////
void MaterialSwitcher::Reset()
{
  this->currentColor =
      gz::math::Color(0.0f, 0.0f, 0.1f, 1.0f);

  this->colorDict.clear();
}

/////////////////////////////////////////////////
void MaterialSwitcher::GetNextColor()
{
  uint32_t color =
      this->currentColor.AsARGB();

  ++color;

  this->currentColor.SetFromARGB(color);
}

/////////////////////////////////////////////////
gz::math::Color MaterialSwitcher::NextColor()
{
  this->GetNextColor();
  return this->currentColor;
}

/////////////////////////////////////////////////
const std::string &
MaterialSwitcher::GetEntityName(
    const gz::math::Color &_color) const
{
  auto iter =
      this->colorDict.find(_color.AsRGBA());

  if (iter != this->colorDict.end())
    return iter->second;

  return this->emptyString;
}

/////////////////////////////////////////////////
void MaterialSwitcher::RegisterVisual(
    const gz::rendering::VisualPtr &_visual)
{
  if (!_visual)
    return;

  this->GetNextColor();

  gz::rendering::MaterialPtr material =
      _visual->Material();

  if (!material)
  {
    auto scene = _visual->Scene();

    if (!scene)
      return;

    material = scene->CreateMaterial();
  }

  material->SetAmbient(this->currentColor);
  material->SetDiffuse(this->currentColor);
  material->SetEmissive(this->currentColor);

  material->SetTransparency(0.0);

  _visual->SetMaterial(material);

  this->colorDict[this->currentColor.AsRGBA()] =
      _visual->Name();
}

/////////////////////////////////////////////////
bool MaterialSwitcher::HasColor(
    const gz::math::Color &_color) const
{
  return this->colorDict.find(_color.AsRGBA()) !=
         this->colorDict.end();
}

/////////////////////////////////////////////////
std::vector<std::string>
MaterialSwitcher::RegisteredVisuals() const
{
  std::vector<std::string> names;

  for (const auto &item : this->colorDict)
  {
    names.push_back(item.second);
  }

  return names;
}

/////////////////////////////////////////////////
size_t MaterialSwitcher::VisualCount() const
{
  return this->colorDict.size();
}
