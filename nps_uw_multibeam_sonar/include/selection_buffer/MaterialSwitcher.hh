/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#ifndef GZ_RENDERING_SELECTIONBUFFER_MATERIALSWITCHER_HH_
#define GZ_RENDERING_SELECTIONBUFFER_MATERIALSWITCHER_HH_

#include <map>
#include <string>
#include <memory>
#include <gz/math/Color.hh>
#include <gz/rendering/config.hh>
#include <gz/rendering/Export.hh>

#include <gz/rendering/Material.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
namespace gazebo
{
  namespace rendering
  {
    class SelectionBuffer;

    /// \brief MaterialSwitcher rewritten for Gazebo Harmonic.
    /// Note: Direct backend listeners are managed via the underlying render engine
    /// abstraction layer rather than exposing raw Ogre types directly.
    class GZ_RENDERING_VISIBLE MaterialSwitcher
    {
      /// \brief Constructor
      public: MaterialSwitcher();

      /// \brief Destructor
      public: virtual ~MaterialSwitcher();

      /// \brief Get the entity name associated with a specific color identifier
      /// \param[in] _color The entity's unique identifier color.
      /// \return The name of the hit entity, or empty string if not found.
      public: const std::string &GetEntityName(
              const gz::math::Color &_color) const;
      /// \brief Get a unique selection material for a visual.
      public: gz::rendering::MaterialPtr GetSelectionMaterial(
      gz::rendering::ScenePtr _scene,
      gz::rendering::VisualPtr _visual);
      /// \brief Reset the color value tracker/incrementor for a new selection pass
      public: void Reset();

      private: typedef std::map<unsigned int, std::string> ColorMap;
      private: typedef ColorMap::const_iterator ColorMapConstIter;
      
      private: std::string emptyString;
      private: gz::math::Color currentColor;
      private: std::string lastEntity;
      private: MaterialSwitcher::ColorMap colorDict;

      /// \brief Increments to generate the next unique mathematical color code
      private: void GetNextColor();

      public: friend class SelectionBuffer;
    };
  }
}
#endif
