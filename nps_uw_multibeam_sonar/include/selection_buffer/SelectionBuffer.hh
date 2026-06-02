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

#ifndef GZ_RENDERING_SELECTION_BUFFER_SELECTIONBUFFER_HH_
#define GZ_RENDERING_SELECTION_BUFFER_SELECTIONBUFFER_HH_

#include <memory>
#include <string>
#include <gz/rendering/RenderTypes.hh>
#include <gz/rendering/Export.hh>

namespace gazebo
{
  namespace rendering
  {
    // Forward declaration of modern private data structures
    struct SelectionBufferPrivate;

    /// \brief SelectionBuffer class modernized for Gazebo Harmonic.
    /// This handles offscreen render-to-texture data picking for custom sensors
    /// such as Sonar or Ray-based systems.
    class GZ_RENDERING_VISIBLE SelectionBuffer
    {
      /// \brief Constructor
      /// \param[in] _camera Smart pointer to the modern rendering camera sensor.
      public: explicit SelectionBuffer(gz::rendering::CameraPtr _camera);

      /// \brief Destructor
      public: virtual ~SelectionBuffer();

      /// \brief Handle on mouse/ray selection click maps to a 3D Visual entity.
      /// \param[in] _x X coordinate in pixels.
      /// \param[in] _y Y coordinate in pixels.
      /// \return Returns the modern abstract Visual wrapper at the coordinate.
      public: gz::rendering::VisualPtr OnSelectionClick(int _x, int _y);

      /// \brief Call this to update the underlying selection pass buffer contents
      public: void Update();

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<SelectionBufferPrivate> dataPtr;
    };
  }
}
#endif
