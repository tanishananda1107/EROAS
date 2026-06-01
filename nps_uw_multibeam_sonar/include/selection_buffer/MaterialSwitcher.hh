#ifndef MATERIAL_SWITCHER_HH_
#define MATERIAL_SWITCHER_HH_

#include <string>
#include <unordered_map>
#include <vector>

#include <gz/math/Color.hh>

#include <gz/rendering/Visual.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/Scene.hh>

namespace gazebo
{
namespace rendering
{

class MaterialSwitcher
{
  public: MaterialSwitcher();

  public: virtual ~MaterialSwitcher();

  public: void Reset();

  public: gz::math::Color NextColor();

  public: const std::string &
      GetEntityName(
          const gz::math::Color &_color) const;

  public: void RegisterVisual(
      const gz::rendering::VisualPtr &_visual);

  public: bool HasColor(
      const gz::math::Color &_color) const;

  public: std::vector<std::string>
      RegisteredVisuals() const;

  public: size_t VisualCount() const;

  private: void GetNextColor();

  private: gz::math::Color currentColor;

  private: std::unordered_map<uint32_t,
      std::string> colorDict;

  private: std::string emptyString;
};

}
}

#endif
