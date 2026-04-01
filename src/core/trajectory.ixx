module;

#include <cstddef>
#include <vector>

export module mininav.core.trajectory;

import mininav.core.types;

export namespace mininav {

class Trajectory {
public:
  void append(const StateRecord &record) { record_.push_back(record); }

  [[nodiscard]] const std::vector<StateRecord> &records() const noexcept {
    return record_;
  }

  [[nodiscard]] std::size_t size() const noexcept { return record_.size(); }

  [[nodiscard]] bool empty() const noexcept { return record_.empty(); }

private:
  std::vector<StateRecord> record_{};
};
} // namespace mininav