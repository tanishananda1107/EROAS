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

#include <gz/common/Console.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Scene.hh>
#include <selection_buffer/SelectionBuffer.hh>
#include <gz/rendering/Visual.hh>

using namespace gazebo;
using namespace rendering;

namespace gazebo
{
  namespace rendering
  {
    struct SelectionBufferPrivate
    {
      /// \brief Pointer to the modern Gazebo Harmonic Camera
      gz::rendering::CameraPtr camera = nullptr;

    };
  }
}

/////////////////////////////////////////////////
SelectionBuffer::SelectionBuffer(gz::rendering::CameraPtr _camera)
: dataPtr(new SelectionBufferPrivate)
{
  if (!_camera)
  {
    gzerr << "Camera pointer passed to SelectionBuffer is null.\n";
    return;
  }

  this->dataPtr->camera = _camera;
  
  gz::rendering::ScenePtr scene = this->dataPtr->camera->Scene();

  if (!scene)
  {
   gzerr << "Failed to extract Scene pointer from Camera.\n";
   return;
}}

/////////////////////////////////////////////////
SelectionBuffer::~SelectionBuffer()
{
  // Smart pointers automatically handle cleanup
}

/////////////////////////////////////////////////
gz::rendering::VisualPtr SelectionBuffer::OnSelectionClick(int _x, int _y)
{
  if (!this->dataPtr->camera)
  {
    gzerr << "Camera unavailable for selection click mapping.\n";
    return nullptr;
  }

  // Boundary check against the render window dimensions
  unsigned int targetWidth = this->dataPtr->camera->ImageWidth();
  unsigned int targetHeight = this->dataPtr->camera->ImageHeight();

  if (_x < 0 || _y < 0 || _x >= static_cast<int>(targetWidth)
      || _y >= static_cast<int>(targetHeight))
  {
    return nullptr;
  }

  // Gazebo Harmonic simplifies selection via Scene::VisualAt
  // This abstracts away the old 1x1 RTT matrix multiplication hack
  gz::rendering::ScenePtr scene = this->dataPtr->camera->Scene();
  if (!scene)
    return nullptr;

  // Query the visual directly at the screen coordinate coordinates mapped from the camera
  gz::rendering::VisualPtr visuallySelected = 
      this->dataPtr->camera->VisualAt(gz::math::Vector2i(_x, _y));

  return visuallySelected;
}
