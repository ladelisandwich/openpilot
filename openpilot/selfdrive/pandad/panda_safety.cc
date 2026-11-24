#include "selfdrive/pandad/pandad.h"
#include "selfdrive/pandad/preap_pedal_wire.h"
#include "openpilot/cereal/messaging/messaging.h"
#include "common/swaglog.h"

void PandaSafety::configureSafetyMode(bool is_onroad) {
  // A Card restart/config change or native reset withdraws before any restage.
  if (is_onroad && safety_configured_ && preap_pedal_words_ &&
      (read_preap_pedal_capsule(params_.getParamPath("TeslaPreAPPedalStartup")) != preap_pedal_capsule_ ||
       params_.get("CarParams") != preap_pedal_cp_bytes_ || pandas_.size() != 1 ||
       !pandas_[0]->matches_preap_pedal(*preap_pedal_words_))) {
    for (auto *panda : pandas_) panda->set_safety_model(cereal::CarParams::SafetyModel::NO_OUTPUT, 0U);
    safety_configured_ = false;
    preap_pedal_words_.reset();
  }
  if (is_onroad && !safety_configured_) {
    updateMultiplexingMode();

    auto car_params = fetchCarParams();
    if (!car_params.empty()) {
      LOGW("got %lu bytes CarParams", car_params.size());
      safety_configured_ = setSafetyMode(car_params);
    }
  } else if (!is_onroad) {
    initialized_ = false;
    safety_configured_ = false;
    log_once_ = false;
    preap_pedal_words_.reset();
    preap_pedal_capsule_.clear();
    preap_pedal_cp_bytes_.clear();
  }
}

void PandaSafety::updateMultiplexingMode() {
  // Initialize to ELM327 without OBD multiplexing for initial fingerprinting
  if (!initialized_) {
    prev_obd_multiplexing_ = false;
    for (auto *panda : pandas_) panda->set_safety_model(cereal::CarParams::SafetyModel::ELM327, 1U);
    initialized_ = true;
  }

  // Switch between multiplexing modes based on the OBD multiplexing request
  bool obd_multiplexing_requested = params_.getBool("ObdMultiplexingEnabled");
  if (obd_multiplexing_requested != prev_obd_multiplexing_) {
    for (size_t i = 0; i < pandas_.size(); ++i) {
      const uint16_t safety_param = (i > 0 || !obd_multiplexing_requested) ? 1U : 0U;
      pandas_[i]->set_safety_model(cereal::CarParams::SafetyModel::ELM327, safety_param);
    }
    prev_obd_multiplexing_ = obd_multiplexing_requested;
    params_.putBool("ObdMultiplexingChanged", true);
  }
}

std::string PandaSafety::fetchCarParams() {
  if (!params_.getBool("FirmwareQueryDone")) {
    return {};
  }

  if (!log_once_) {
    LOGW("Finished FW query, Waiting for params to set safety model");
    log_once_ = true;
  }

  if (!params_.getBool("ControlsReady")) {
    return {};
  }
  return params_.get("CarParams");
}

bool PandaSafety::setSafetyMode(const std::string &params_string) {
  AlignedBuffer aligned_buf;
  capnp::FlatArrayMessageReader cmsg(aligned_buf.align(params_string.data(), params_string.size()));
  cereal::CarParams::Reader car_params = cmsg.getRoot<cereal::CarParams>();

  auto safety_configs = car_params.getSafetyConfigs();
  uint16_t alternative_experience = car_params.getAlternativeExperience();
  const bool pedal = car_params.getBrand() == "tesla" && car_params.getCarFingerprint() == "TESLA_MODEL_S_PREAP" &&
    car_params.getOpenpilotLongitudinalControl();
  std::optional<PreapPedalWords> pedal_words;
  std::string pedal_capsule;
  if (pedal) {
    pedal_capsule = read_preap_pedal_capsule(params_.getParamPath("TeslaPreAPPedalStartup"));
    pedal_words = preap_pedal_words(pedal_capsule, params_string, car_params);
    if (pandas_.size() != 1 || !pedal_words) {
      for (auto *panda : pandas_) panda->set_safety_model(cereal::CarParams::SafetyModel::NO_OUTPUT, 0U);
      return false;
    }
  }


  alternative_experience_ = alternative_experience;
  safety_config_count_ = safety_configs.size();
  safety_slots_.clear();
  for (size_t i = 0; i < pandas_.size(); ++i) {
    auto model = cereal::CarParams::SafetyModel::SILENT;
    uint16_t param = 0U;
    if (i < safety_configs.size()) {
      model = safety_configs[i].getSafetyModel();
      param = safety_configs[i].getSafetyParam();
    }
    safety_slots_.emplace_back(model, param);
    LOGW("Panda %zu: setting safety model: %d, param: %d, alternative experience: %d", i, (int)model, param, alternative_experience);
    pandas_[i]->set_alternative_experience(alternative_experience);
    if (pedal && !pandas_[i]->stage_preap_pedal(*pedal_words)) {
      pandas_[i]->set_safety_model(cereal::CarParams::SafetyModel::NO_OUTPUT, 0U);
      return false;
    }
    pandas_[i]->set_safety_model(model, param);
    if (pedal && !pandas_[i]->matches_preap_pedal(*pedal_words)) {
      pandas_[i]->set_safety_model(cereal::CarParams::SafetyModel::NO_OUTPUT, 0U);
      return false;
    }
  }
  preap_pedal_words_ = pedal_words;
  preap_pedal_capsule_ = pedal ? pedal_capsule : std::string();
  preap_pedal_cp_bytes_ = pedal ? params_string : std::string();
  return true;
}

Panda *PandaSafety::aolOwner() {
  if (pandas_.size() == 1) return pandas_[0];
  if (pandas_.size() != 2 || !safety_configured_ || safety_config_count_ != pandas_.size() ||
      safety_slots_.size() != pandas_.size()) return nullptr;
  if (pandas_[0]->hw_serial().empty() || pandas_[1]->hw_serial().empty() ||
      pandas_[0]->hw_serial() == pandas_[1]->hw_serial()) return nullptr;
  Panda *owner = nullptr;
  for (size_t i = 0; i < pandas_.size(); ++i) {
    auto *panda = pandas_[i];
    if (!panda->connected() || !panda->comms_healthy()) return nullptr;
    const auto health = panda->get_state();
    if (!health || health->alternative_experience_pkt != alternative_experience_) return nullptr;
    const auto [model, param] = safety_slots_[i];
    const bool passive = model == cereal::CarParams::SafetyModel::NO_OUTPUT || model == cereal::CarParams::SafetyModel::SILENT;
    if (passive) {
      if (param != 0U || (health->safety_mode_pkt != (uint8_t)cereal::CarParams::SafetyModel::NO_OUTPUT &&
           health->safety_mode_pkt != (uint8_t)cereal::CarParams::SafetyModel::SILENT) ||
          health->safety_param_pkt != 0U || (health->flags_pkt & HEALTH_FLAG_CONTROLS_ALLOWED) != 0U) return nullptr;
    } else {
      if (owner != nullptr || health->safety_mode_pkt != (uint8_t)model || health->safety_param_pkt != param) return nullptr;
      owner = panda;
    }
  }
  return owner;
}

bool PandaSafety::aolStatusMatches(Panda *owner, const aol_safety_health_t &status) const {
  if (pandas_.size() == 1) return owner == pandas_[0];
  if (pandas_.size() != 2 || safety_config_count_ != pandas_.size() || safety_slots_.size() != pandas_.size()) return false;
  for (size_t i = 0; i < pandas_.size(); ++i) {
    if (pandas_[i] == owner) {
      return status.safety_mode == (uint8_t)safety_slots_[i].first && status.safety_param == safety_slots_[i].second;
    }
  }
  return false;
}
