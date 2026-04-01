export module mininav.core.command_source;

import mininav.core.types;

export namespace mininav {
class CommandSource {
public:
  virtual ~CommandSource() = default;
  [[nodiscard]] virtual Twist2D command_at(double t) const noexcept = 0;
};

class StagedCommandSource final : public CommandSource {
public:
  [[nodiscard]] Twist2D command_at(double t) const noexcept override;
};
} // namespace mininav