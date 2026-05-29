#include <string>
#include <memory>

#include <gz/rendering/Scene.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/SelectionBuffer.hh>
#include <gz/rendering/Visual.hh>

#include <gz/common/Console.hh>

class SelectionBuffer
{
public:
  SelectionBuffer(gz::rendering::ScenePtr _scene,
                  const std::string &_cameraName)
  {
    this->scene = _scene;

    this->camera = _scene->CameraByName(_cameraName);

    if (!this->camera)
    {
      gzerr << "Camera not found: " << _cameraName << "\n";
      return;
    }

    this->selBuffer =
      _scene->CreateSelectionBuffer(_cameraName + "_sel", this->camera);
  }

  ~SelectionBuffer() = default;

  /// Click-based picking (replaces ALL OGRE pixel logic)
  gz::rendering::VisualPtr OnSelectionClick(int _x, int _y)
  {
    if (!this->selBuffer)
      return nullptr;

    return this->selBuffer->Select(_x, _y);
  }

  /// Optional helper: get entity name
  std::string GetEntityName(int _x, int _y)
  {
    auto visual = this->OnSelectionClick(_x, _y);
    if (!visual)
      return "";

    return visual->Name();
  }

private:
  gz::rendering::ScenePtr scene;
  gz::rendering::CameraPtr camera;
  gz::rendering::SelectionBufferPtr selBuffer;
};
