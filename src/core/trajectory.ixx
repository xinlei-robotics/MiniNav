module;

#include <cstddef>
#include <vector>

export module mininav.core.trajectory;

export namespace mininav
{
    template <typename T>
    class Trajectory
    {
    public:
        using value_type = T;
        using container_type = std::vector<T>;

        void reserve(std::size_t capacity) { records_.reserve(capacity); }

        void append(const T& record) { records_.push_back(record); }
        void append(T&& record) { records_.push_back(std::move(record)); }

        void clear() noexcept { records_.clear(); }

        [[nodiscard]] const container_type& records() const noexcept { return records_; }

        [[nodiscard]] std::size_t size() const noexcept { return records_.size(); }
        [[nodiscard]] bool empty() const noexcept { return records_.empty(); }

    private:
        container_type records_{};
    };
} // namespace mininav
