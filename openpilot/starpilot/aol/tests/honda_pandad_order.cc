// Private test transport around unchanged production negotiation/armed-wire code.
#include "openpilot/selfdrive/pandad/aol_protocol.h"
#include "openpilot/selfdrive/pandad/aol_wire.h"
#include "openpilot/starpilot/car/honda/aol_policy.h"
#include <iostream>
#include <sstream>
#include <vector>

int main() {
  constexpr AolSafetyProfile profiles[] = {HONDA_STOCK_AOL_PROFILE, HONDA_NIDEC_AOL_PROFILE};
  AolAxisNegotiator owner({profiles, std::size(profiles)});
  AolWritePlan plan;
  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream in(line);
    std::string operation;
    unsigned request, permission, mode, word, capable;
    in >> operation >> request >> permission >> mode >> word >> capable;
    aol_safety_health_t status{AOL_SAFETY_PROTOCOL_MAGIC, AOL_SAFETY_PROTOCOL_VERSION,
      static_cast<uint8_t>(request), static_cast<uint8_t>(permission), static_cast<uint8_t>(mode),
      static_cast<uint16_t>(word), static_cast<uint8_t>(capable)};
    if (operation == "PREP") {
      AolAxisInput axis;
      uint64_t now, car_state_ns, intent_message_ns;
      std::string raw;
      bool onroad;
      in >> now >> axis.alive_valid >> axis.qualified >> axis.session >> axis.observed_ns >> axis.valid_until_ns
         >> axis.message_ns >> axis.desired_lateral >> axis.desired_longitudinal >> onroad
         >> car_state_ns >> intent_message_ns >> raw;
      std::vector<capnp::byte> bytes;
      if (raw != "-") {
        if (raw.size() % 2U != 0U) return 2;
        for (size_t pos = 0; pos < raw.size(); pos += 2U) {
          bytes.push_back(static_cast<capnp::byte>(std::stoul(raw.substr(pos, 2U), nullptr, 16)));
        }
      }
      axis.retain_lateral_arm = onroad && aol_armed_intent({bytes.data(), bytes.size()}, intent_message_ns, car_state_ns, now);
      plan = owner.prepare(aol_runtime_enabled(false, status, {profiles, std::size(profiles)}) ?
                           std::optional(status) : std::nullopt, axis, now, static_cast<uint8_t>(mode));
      std::cout << plan.capable << ' ' << unsigned(plan.request_mask) << ' ' << plan.retain_lateral_arm << std::endl;
    } else if (operation == "DONE") {
      bool write_ok;
      in >> write_ok;
      const auto outcome = owner.complete(plan, write_ok, status);
      std::cout << outcome.compatible << ' ' << (outcome.session.empty() ? "-" : outcome.session) << std::endl;
    } else {
      return 3;
    }
    if (in.fail()) return 4;
  }
}
