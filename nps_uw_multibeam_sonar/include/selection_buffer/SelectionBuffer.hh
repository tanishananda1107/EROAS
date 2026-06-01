<<<<<<< HEAD
#ifndef SELECTIONBUFFER_HH_
#define SELECTIONBUFFER_HH_
=======

/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Ported to Gazebo Harmonic (gz-sim 8 / gz-rendering 8).
 * The classic Gazebo selection buffer used Ogre internals directly.
 * In Gazebo Harmonic, gz-rendering exposes a renderer-agnostic ray-query
 * API (gz::rendering::RayQuery) that replaces the entire Ogre-based
 * MaterialSwitcher / SelectionBuffer / SelectionRenderListener stack.
 * No Ogre headers are included here.
*/

#ifndef GZ_RENDERING_SELECTION_BUFFER_HH_
#define GZ_RENDERING_SELECTION_BUFFER_HH_
>>>>>>> bde8874 (Remove unused directories from navigator_auv)

#include <memory>
#include <string>

<<<<<<< HEAD
#include <gz/math/Vector2i.hh>

#include <gz/rendering/Camera.hh>
#include <gz/rendering/RayQuery.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>

namespace gazebo
{
namespace rendering
{

class SelectionBuffer
{
  public: SelectionBuffer(
      const gz::rendering::CameraPtr &_camera,
      const gz::rendering::ScenePtr &_scene);

  public: virtual ~SelectionBuffer();

  public: gz::rendering::VisualPtr OnSelectionClick(
      int _x,
      int _y);

  public: std::string SelectedVisualName(
      int _x,
      int _y);

  private: gz::rendering::CameraPtr camera;

  private: gz::rendering::ScenePtr scene;

  private: gz::rendering::RayQueryPtr rayQuery;
};

}
}

#endif
=======
// gz-rendering public API only — NO Ogre headers
#include <gz/rendering/Camera.hh>
#include <gz/rendering/RayQuery.hh>
#include <gz/rendering/Scene.hh>
#include <gz/math/Vector2.hh>

namespace gz
{
  namespace rendering
  {
    /// \brief Renderer-agnostic entity picker.
    ///
    /// Replaces the classic gazebo::rendering::SelectionBuffer +
    /// MaterialSwitcher + SelectionRenderListener stack which relied on
    /// internal Ogre 1.x APIs.
    ///
    /// Uses gz::rendering::RayQuery — the public gz-rendering picking
    /// interface that works with any backend (ogre2, optix, …).
    class SelectionBuffer
    {
      /// \brief Constructor.
      /// \param[in] _camera  The camera through which picking is performed.
      /// \param[in] _scene   The gz-rendering scene.
      public: SelectionBuffer(gz::rendering::CameraPtr _camera,
                              gz::rendering::ScenePtr  _scene);

      /// \brief Destructor.
      public: ~SelectionBuffer();

      /// \brief Return the name of the visual at viewport pixel (_x, _y).
      ///
      /// Casts a ray from the camera through the given pixel and returns
      /// the name of the first gz::rendering::Visual that is hit, or an
      /// empty string if nothing was hit.
      ///
      /// \param[in] _x  Pixel X in viewport space.
      /// \param[in] _y  Pixel Y in viewport space.
      /// \return Name of the intersected visual, or "".
      public: std::string OnSelectionClick(int _x, int _y);

      /// \brief Update internal state (call once per frame if needed).
      ///        No-op in the ray-query implementation; kept for API compat.
      public: void Update();

      private: gz::rendering::CameraPtr   camera_;
      private: gz::rendering::ScenePtr    scene_;
      private: gz::rendering::RayQueryPtr rayQuery_;
    };

  }  // namespace rendering
}  // namespace gz

#endif  // GZ_RENDERING_SELECTION_BUFFER_HH_
HEREDOC
>>>>>>> bde8874 (Remove unused directories from navigator_auv)
