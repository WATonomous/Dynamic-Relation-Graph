#include <lanelet2_traffic_rules/GenericTrafficRules.h>
#include <lanelet2_traffic_rules/GermanTrafficRules.h>

namespace lanelet {
namespace traffic_rules {

class WatoVehicle : public GenericTrafficRules {
 public:
  using GenericTrafficRules::GenericTrafficRules;

  const CountrySpeedLimits& countrySpeedLimits() const override {
    return speedLimits_;
  }
  Optional<SpeedLimitInformation> speedLimit(
      const RegulatoryElementConstPtrs& regelems) const override {
    return {};
  };

  // function assumes all associated TrafficSign regulatory elements result in
  // canPass failing
  Optional<bool> canPass(
      const RegulatoryElementConstPtrs& regElems) const override {
    auto associatedTrafficSignRegElems =
        utils::transformSharedPtr<const TrafficSign>(regElems);
    if (!associatedTrafficSignRegElems.empty()) {
      return false;
    }
    return {};
  }

  Optional<bool> canPass(const std::string& type,
                         const std::string& /*location*/) const override {
    if (type == "blocked_by_occupancy") {
      return false;
    }
    return GenericTrafficRules::canPass(type, "");
  }

  bool canPass(const ConstLanelet& lanelet) const override {
    if (lanelet.attributeOr("blocked_by_occupancy", false) == true) {
      return false;
    }
    return GenericTrafficRules::canPass(lanelet);
  }

 private:
  CountrySpeedLimits speedLimits_{germanSpeedLimits()};
};
}  // namespace traffic_rules
}  // namespace lanelet
