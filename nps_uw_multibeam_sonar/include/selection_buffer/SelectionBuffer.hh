#ifndef SELECTIONBUFFER_HH_
#define SELECTIONBUFFER_HH_

#include <memory>
#include <string>

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
