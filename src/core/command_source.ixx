export module mininav.core.command_source;

import mininav.core.types;

export namespace mininav
{
    class CommandSource
    {
    public:
        CommandSource() = default;
        virtual ~CommandSource() = default;

        CommandSource(const CommandSource&) = delete;
        CommandSource& operator=(const CommandSource&) = delete;
        CommandSource(CommandSource&&) = delete;
        CommandSource& operator=(CommandSource&&) = delete;

        [[nodiscard]] virtual Twist2D command_at(double t) const noexcept = 0;
    };

    class StagedCommandSource final : public CommandSource
    {
    public:
        [[nodiscard]] Twist2D command_at(double t) const noexcept override;
    };
} // namespace mininav
