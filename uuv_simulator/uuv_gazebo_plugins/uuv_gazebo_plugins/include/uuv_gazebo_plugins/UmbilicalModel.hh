#ifndef UUV_GZ_SIM_UMBILICAL_MODEL_HH_
#define UUV_GZ_SIM_UMBILICAL_MODEL_HH_

#include <string>
#include <map>
#include <memory>

#include <sdf/sdf.hh>

#include <gz/math/Vector3.hh>

#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/EntityComponentManager.hh>

namespace uuv_gz_plugins
{

/// Base class for all tether / umbilical models (gz-sim8 compatible)
class UmbilicalModel
{
public:
  UmbilicalModel() = default;
  virtual ~UmbilicalModel() = default;

  /// Initialize model after creation
  virtual void Init() {}

  /// Update physics every simulation step
  virtual void OnUpdate(
      const double _dt,
      const gz::math::Vector3d &_flow) = 0;

  /// Attach model from ECS (gz-sim architecture)
  virtual void SetModel(
      const gz::sim::Model & _model,
      const gz::sim::EntityComponentManager & _ecm)
  {
    this->model = _model;
    this->ecm = &_ecm;
  }

protected:
  gz::sim::Model model;
  const gz::sim::EntityComponentManager *ecm{nullptr};

  gz::sim::Link connector;
};


/// Factory function type (gz-sim8 safe: no model copying)
using UmbilicalModelCreator =
  UmbilicalModel* (*)(sdf::ElementPtr, const gz::sim::Model &);


/// Factory for tether models
class UmbilicalModelFactory
{
public:
  UmbilicalModel* CreateUmbilicalModel(
      sdf::ElementPtr _sdf,
      const gz::sim::Model & _model)
  {
    std::string type = _sdf->Get<std::string>("type", "berg").first;

    auto it = creators_.find(type);
    if (it == creators_.end())
      return nullptr;

    return (it->second)(_sdf, _model);
  }

  static UmbilicalModelFactory &GetInstance()
  {
    static UmbilicalModelFactory inst;
    return inst;
  }

  bool RegisterCreator(const std::string &_id,
                       UmbilicalModelCreator _creator)
  {
    creators_[_id] = _creator;
    return true;
  }

private:
  UmbilicalModelFactory() = default;

  std::map<std::string, UmbilicalModelCreator> creators_;
};


/// Macros
#define REGISTER_UMBILICALMODEL(type) \
  static const bool registeredWithFactory

#define REGISTER_UMBILICALMODEL_CREATOR(type, creator) \
  const bool type::registeredWithFactory = \
    UmbilicalModelFactory::GetInstance().RegisterCreator( \
      type::IDENTIFIER, creator);


/// Berg implementation
class UmbilicalModelBerg : public UmbilicalModel
{
protected:
  UmbilicalModelBerg(sdf::ElementPtr _sdf,
                     const gz::sim::Model & _model);

public:
  static UmbilicalModel* create(sdf::ElementPtr _sdf,
                                const gz::sim::Model & _model);

  void OnUpdate(const double _dt,
                const gz::math::Vector3d &_flow) override;

private:
  REGISTER_UMBILICALMODEL(UmbilicalModelBerg);

  static const std::string IDENTIFIER;

  double diameter{0.0};
  double rho{1000.0};
};

} // namespace uuv_gz_plugins

#endif
