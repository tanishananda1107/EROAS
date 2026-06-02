#pragma once

#include <memory>
#include <string>
#include <gz/rendering/RenderTypes.hh>
#include <gz/math/Vector2.hh>

namespace custom_sonar
{
  class SonarBufferHandler
  {
    public: SonarBufferHandler() = default;
    public: virtual ~SonarBufferHandler() = default;

    /// \brief Replaces the entire RenderListener target update pattern.
    /// \param[in] _camera The modern abstract rendering camera pointer.
    /// \param[in] _x Pixel X position.
    /// \param[in] _y Pixel Y position.
    /// \return The visual entity found at those coordinates.
    public: gz::rendering::VisualPtr GetSelectedVisual(gz::rendering::CameraPtr _camera, int _x, int _y)
    {
      if (!_camera)
        return nullptr;

      // Ensure target coordinates are within rendering view boundaries
      if (_x < 0 || _y < 0 || 
          _x >= static_cast<int>(_camera->ImageWidth()) || 
          _y >= static_cast<int>(_camera->ImageHeight()))
      {
        return nullptr;
      }

      // Gazebo Harmonic automatically invokes its internal material selection 
      // mechanisms natively when calling this function. No listeners required!
      return _camera->VisualAt(gz::math::Vector2i(_x, _y));
    }
  };
}
