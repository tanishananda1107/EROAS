#include "SelectionBuffer.hh"

#include <gz/rendering/RenderingIface.hh>

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
SelectionBuffer::SelectionBuffer(
    const gz::rendering::CameraPtr &_camera,
    const gz::rendering::ScenePtr &_scene)
    : camera(_camera),
      scene(_scene)
{
  if (this->scene)
  {
    this->rayQuery =
        this->scene->CreateRayQuery();
  }
}

/////////////////////////////////////////////////
SelectionBuffer::~SelectionBuffer()
{
  if (this->scene && this->rayQuery)
  {
    this->scene->DestroyRayQuery(
        this->rayQuery);
  }
}

/////////////////////////////////////////////////
gz::rendering::VisualPtr
SelectionBuffer::OnSelectionClick(
    int _x,
    int _y)
{
  if (!this->camera ||
      !this->scene ||
      !this->rayQuery)
  {
    return nullptr;
  }

  const unsigned int width =
      this->camera->ImageWidth();

  const unsigned int height =
      this->camera->ImageHeight();

  double nx =
      static_cast<double>(_x) /
      static_cast<double>(width);

  double ny =
      static_cast<double>(_y) /
      static_cast<double>(height);

  gz::math::Vector3d origin;
  gz::math::Vector3d direction;

  this->camera->Project(
      gz::math::Vector2d(nx, ny),
      origin,
      direction);

  this->rayQuery->SetOrigin(origin);
  this->rayQuery->SetDirection(direction);

  auto result =
      this->rayQuery->ClosestPoint();

  if (!result)
    return nullptr;

  auto node =
      this->scene->NodeById(
          result->objectId);

  if (!node)
    return nullptr;

  return std::dynamic_pointer_cast<
      gz::rendering::Visual>(node);
}

/////////////////////////////////////////////////
std::string SelectionBuffer::SelectedVisualName(
    int _x,
    int _y)
{
  auto visual =
      this->OnSelectionClick(_x, _y);

  if (!visual)
    return "";

  return visual->Name();
}
