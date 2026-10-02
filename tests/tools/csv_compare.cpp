module;

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <istream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

module mininav.tools.csv_compare;

namespace mininav::tools
{
    namespace
    {
        [[nodiscard]] std::vector<std::string> read_lines(std::istream& in)
        {
            std::vector<std::string> lines;
            std::string line;
            while (std::getline(in, line))
            {
                lines.push_back(line);
            }
            return lines;
        }

        [[nodiscard]] std::vector<std::string_view> split_fields(const std::string_view line)
        {
            std::vector<std::string_view> fields;
            std::size_t begin = 0;
            while (true)
            {
                const std::size_t comma = line.find(',', begin);
                if (comma == std::string_view::npos)
                {
                    fields.push_back(line.substr(begin));
                    return fields;
                }
                fields.push_back(line.substr(begin, comma - begin));
                begin = comma + 1;
            }
        }

        [[nodiscard]] std::string_view trim(std::string_view s) noexcept
        {
            while (!s.empty() && (s.front() == ' ' || s.front() == '\t'))
            {
                s.remove_prefix(1);
            }
            while (!s.empty() && (s.back() == ' ' || s.back() == '\t'))
            {
                s.remove_suffix(1);
            }
            return s;
        }

        // "# key = value" → "key";不含 '=' 的注释行返回空。
        [[nodiscard]] std::string_view comment_key(const std::string_view line) noexcept
        {
            const std::string_view body = line.substr(1);
            const std::size_t eq = body.find('=');
            return eq == std::string_view::npos ? std::string_view{} : trim(body.substr(0, eq));
        }

        [[nodiscard]] bool is_integer(std::string_view s) noexcept
        {
            if (!s.empty() && (s.front() == '+' || s.front() == '-'))
            {
                s.remove_prefix(1);
            }
            return !s.empty() && std::ranges::all_of(s, [](const char c) { return c >= '0' && c <= '9'; });
        }

        // 整个字段都必须被消费,否则不算浮点("1.5abc" 按文本比较)。
        [[nodiscard]] std::optional<double> parse_double(std::string_view s) noexcept
        {
            if (!s.empty() && s.front() == '+')
            {
                s.remove_prefix(1); // from_chars 不接受前导 '+'
            }
            double value{};
            const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), value);
            if (ec != std::errc{} || ptr != s.data() + s.size())
            {
                return std::nullopt;
            }
            return value;
        }

        [[nodiscard]] std::string sci(const double v)
        {
            std::ostringstream os;
            os << std::scientific << std::setprecision(3) << v;
            return os.str();
        }

        class Recorder
        {
        public:
            Recorder(CsvCompareReport& report, const std::size_t max_reported) noexcept
                : report_{report}, max_reported_{max_reported}
            {
            }

            void add(const std::size_t line, std::string_view column, std::string_view expected,
                     std::string_view actual, std::string reason)
            {
                ++report_.mismatch_count;
                if (report_.mismatches.size() < max_reported_)
                {
                    report_.mismatches.push_back(CsvMismatch{
                        .line = line,
                        .column = std::string{column},
                        .expected = std::string{expected},
                        .actual = std::string{actual},
                        .reason = std::move(reason),
                    });
                }
            }

        private:
            CsvCompareReport& report_;
            std::size_t max_reported_;
        };

        // 单个数据字段;字符串相同的快速路径已在调用方处理。
        void compare_field(const std::size_t line, const std::string_view column,
                           const std::string_view e, const std::string_view a,
                           const CsvCompareOptions& opts, CsvCompareReport& report, Recorder& rec)
        {
            if (is_integer(e) && is_integer(a))
            {
                rec.add(line, column, e, a, "integer differs");
                return;
            }

            const std::optional<double> ev = parse_double(e);
            const std::optional<double> av = parse_double(a);
            if (!ev.has_value() || !av.has_value())
            {
                rec.add(line, column, e, a, "text differs");
                return;
            }
            if (std::isnan(*ev) || std::isnan(*av))
            {
                if (!(std::isnan(*ev) && std::isnan(*av)))
                {
                    rec.add(line, column, e, a, "NaN on one side only");
                }
                return;
            }
            if (*ev == *av) // 例如 "1" 对 "1.0",或同号无穷
            {
                return;
            }
            if (std::isinf(*ev) || std::isinf(*av))
            {
                rec.add(line, column, e, a, "infinity differs");
                return;
            }

            const double scaled = std::abs(*ev - *av) / std::max(1.0, std::abs(*ev));
            report.max_scaled_diff = std::max(report.max_scaled_diff, scaled);
            if (scaled > opts.rel_tol)
            {
                rec.add(line, column, e, a,
                        "scaled diff " + sci(scaled) + " exceeds tolerance " + sci(opts.rel_tol));
            }
        }
    }

    CsvCompareReport compare_csv(std::istream& expected, std::istream& actual,
                                 const CsvCompareOptions& opts)
    {
        CsvCompareReport report;
        Recorder rec{report, opts.max_reported};

        const std::vector<std::string> exp_lines = read_lines(expected);
        const std::vector<std::string> act_lines = read_lines(actual);
        if (exp_lines.size() != act_lines.size())
        {
            rec.add(0, {}, std::to_string(exp_lines.size()), std::to_string(act_lines.size()),
                    "line count differs");
        }

        const auto is_ignored = [&opts](const std::string_view key)
        {
            return !key.empty() && std::ranges::find(opts.ignored_comment_keys, key)
                != opts.ignored_comment_keys.end();
        };

        std::vector<std::string> columns; // 列名(取自 golden),仅用于报告
        bool header_seen = false;

        const std::size_t n = std::min(exp_lines.size(), act_lines.size());
        for (std::size_t i = 0; i < n; ++i)
        {
            const std::string_view e = exp_lines[i];
            const std::string_view a = act_lines[i];
            const std::size_t line = i + 1;
            ++report.lines_compared;

            if (e != a && e.ends_with('\r') != a.ends_with('\r'))
            {
                rec.add(line, {}, e, a,
                        "line endings differ (CRLF vs LF): check .gitattributes / core.autocrlf");
                continue;
            }

            const bool e_comment = e.starts_with('#');
            if (e_comment != a.starts_with('#'))
            {
                rec.add(line, {}, e, a, "comment line on one side only");
                continue;
            }

            // 1) 元数据注释行
            if (e_comment)
            {
                const std::string_view key = comment_key(e);
                if (e != a && !(is_ignored(key) && key == comment_key(a)))
                {
                    rec.add(line, {}, e, a, "metadata line differs");
                }
                continue;
            }

            // 2) 列名行
            if (!header_seen)
            {
                header_seen = true;
                for (const std::string_view name : split_fields(e))
                {
                    columns.emplace_back(name);
                }
                if (e != a)
                {
                    rec.add(line, {}, e, a, "column header differs");
                }
                continue;
            }

            // 3) 数据行
            if (e == a)
            {
                report.fields_compared += static_cast<std::size_t>(std::ranges::count(e, ',')) + 1;
                continue;
            }
            const std::vector<std::string_view> ef = split_fields(e);
            const std::vector<std::string_view> af = split_fields(a);
            if (ef.size() != af.size())
            {
                rec.add(line, {}, std::to_string(ef.size()), std::to_string(af.size()),
                        "field count differs");
                continue;
            }
            for (std::size_t c = 0; c < ef.size(); ++c)
            {
                ++report.fields_compared;
                if (ef[c] == af[c])
                {
                    continue;
                }
                const std::string column = c < columns.size() ? columns[c] : "#" + std::to_string(c + 1);
                compare_field(line, column, ef[c], af[c], opts, report, rec);
            }
        }
        return report;
    }
}
