#include <gz/rendering/Camera.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/math/Vector2.hh>

// Example method inside your custom Sonar Plugin
gz::rendering::VisualPtr IdentifySonarTarget(gz::rendering::CameraPtr _camera, int _screenX, int _y)
{
  if (!_camera)
    return nullptr;

  // 1. Check boundaries against current rendering viewport dimensions
  unsigned int width = _camera->ImageWidth();
  unsigned int height = _camera->ImageHeight();

  if (_screenX < 0 || _y < 0 || _screenX >= static_cast<int>(width) || _y >= static_cast<int>(height))
  {
    return nullptr;
  }

  // 2. Fetch the modern visual pointer directly using the native visual selection pass
  // This automatically handles what the old SelectionRenderListener used to do!
  gz::rendering::VisualPtr pickedVisual = _camera->VisualAt(gz::math::Vector2i(_screenX, _y));

  if (pickedVisual)
  {
    // You can now extract the object name, IDs, or geometry features for your sonar calculations
    // gzmsg << "Sonar hit object: " << pickedVisual->Name() << "\n";
    return pickedVisual;
  }

  return nullptr;
}
