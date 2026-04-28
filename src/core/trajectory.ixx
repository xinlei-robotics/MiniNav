module;

#include <cstddef>
#include <vector>

export module mininav.core.trajectory;

import mininav.core.types;

export namespace mininav
{
    // ---------------------------------------------------------------------------
    // Trajectory: 一段时间内的状态记录序列
    // ---------------------------------------------------------------------------
    class Trajectory
    {
    public:
        void reserve(const std::size_t capacity) { records_.reserve(capacity); }

        void append(const StateRecord& record) { records_.push_back(record); }

        [[nodiscard]] const std::vector<StateRecord>& records() const noexcept
        {
            return records_;
        }

        [[nodiscard]] std::size_t size() const noexcept { return records_.size(); }

        [[nodiscard]] bool empty() const noexcept { return records_.empty(); }

    private:
        std::vector<StateRecord> records_{};
    };
} // namespace mininav
