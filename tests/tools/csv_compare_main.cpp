import mininav.tools.csv_compare;

#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <string_view>

// ===========================================================================
// csv_compare: golden 回归的比较端。
//
//   csv_compare <golden.csv> <actual.csv> [--rel-tol <x>]
//
// 退出码:0 = 一致;1 = 有差异(打印前若干条明细);2 = 用法 / 文件错误。
// 比较规则见 csv_compare.ixx 与 tests/golden/README.md。
// ===========================================================================

namespace
{
    constexpr int kExitMismatch = 1;
    constexpr int kExitUsage = 2;

    int usage(const char* argv0)
    {
        std::cerr << "usage: " << argv0 << " <golden.csv> <actual.csv> [--rel-tol <x>]\n";
        return kExitUsage;
    }
}

int main(int argc, char** argv)
{
    using namespace mininav::tools;

    if (argc != 3 && argc != 5)
    {
        return usage(argv[0]);
    }

    CsvCompareOptions opts{};
    if (argc == 5)
    {
        if (std::string_view{argv[3]} != "--rel-tol")
        {
            return usage(argv[0]);
        }
        try
        {
            opts.rel_tol = std::stod(argv[4]);
        }
        catch (const std::exception&)
        {
            return usage(argv[0]);
        }
    }

    const std::string golden_path{argv[1]};
    const std::string actual_path{argv[2]};
    std::ifstream golden{golden_path};
    std::ifstream actual{actual_path};
    if (!golden)
    {
        std::cerr << "csv_compare: cannot open golden file " << golden_path << '\n';
        return kExitUsage;
    }
    if (!actual)
    {
        std::cerr << "csv_compare: cannot open actual file " << actual_path << '\n';
        return kExitUsage;
    }

    const CsvCompareReport report = compare_csv(golden, actual, opts);

    if (report.ok())
    {
        std::cout << "csv_compare: OK  " << report.lines_compared << " lines, "
            << report.fields_compared << " fields, max scaled diff " << report.max_scaled_diff
            << " (tol " << opts.rel_tol << ")\n";
        return EXIT_SUCCESS;
    }

    std::cout << "csv_compare: MISMATCH  " << report.mismatch_count << " difference(s) between\n"
        << "  golden: " << golden_path << "\n"
        << "  actual: " << actual_path << '\n';
    for (const CsvMismatch& m : report.mismatches)
    {
        std::cout << "  line " << m.line;
        if (!m.column.empty())
        {
            std::cout << " [" << m.column << ']';
        }
        std::cout << ": " << m.reason << "\n    golden: " << m.expected
            << "\n    actual: " << m.actual << '\n';
    }
    if (report.mismatch_count > report.mismatches.size())
    {
        std::cout << "  ... " << report.mismatch_count - report.mismatches.size()
            << " more not shown\n";
    }
    std::cout << "If this change is intentional, regenerate with `cmake --build <build-dir> "
                 "--target update_golden` and state the reason in the PR (tests/golden/README.md).\n";
    return kExitMismatch;
}
