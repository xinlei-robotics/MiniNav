module;

#include <array>
#include <cstdint>
#include <random>
#include <string_view>

export module mininav.core.random;

export namespace mininav
{
    namespace detail
    {
        // FNV-1a 64-bit hash.
        [[nodiscard]] constexpr std::uint64_t fnv1a_64(std::string_view s) noexcept
        {
            constexpr std::uint64_t kOffset = 0xcbf29ce484222325ULL;
            constexpr std::uint64_t kPrime = 0x100000001b3ULL;
            std::uint64_t h = kOffset;
            for (char c : s)
            {
                h ^= static_cast<std::uint64_t>(static_cast<unsigned char>(c));
                h *= kPrime;
            }
            return h;
        }
    }

    class RngFactory
    {
    public:
        explicit RngFactory(const std::uint64_t master_seed) noexcept : master_seed_{master_seed}
        {
        }

        [[nodiscard]] std::mt19937 make_engine(const std::string_view tag) const
        {
            const std::uint64_t tag_hash{detail::fnv1a_64(tag)};
            const std::array entropy{
                static_cast<std::uint32_t>(master_seed_ & 0xffffffffu),
                static_cast<std::uint32_t>((master_seed_ >> 32) & 0xffffffffu),
                static_cast<std::uint32_t>(tag_hash & 0xffffffffu),
                static_cast<std::uint32_t>((tag_hash >> 32) & 0xffffffffu),
            };

            std::seed_seq seq(entropy.begin(), entropy.end());
            return std::mt19937{seq};
        }

        [[nodiscard]] std::uint64_t master_seed() const noexcept { return master_seed_; }

    private:
        std::uint64_t master_seed_;
    };
}
