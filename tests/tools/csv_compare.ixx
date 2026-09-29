module;

#include <cstddef>
#include <istream>
#include <string>
#include <vector>

export module mininav.tools.csv_compare;

export namespace mininav::tools
{
    // ---------------------------------------------------------------------------
    // CsvCompareOptions: golden CSV 与 sim 实际产出的比较规则。
    //
    //   - '#' 注释行(元数据)逐字相同;ignored_comment_keys 里的 key(形如
    //     "# generated_at = ...")跳过 —— 它是墙钟时间,每次运行都不同。
    //   - 第一条非注释行是列名行,逐字相同。
    //   - 数据字段:字符串相同即相等;两边都是整数 → 必须精确相等;两边都能解析为
    //     浮点 → |a − b| ≤ rel_tol · max(1, |a|)(a 为 golden 值);其余按文本精确比较。
    //
    // 为什么浮点不逐字节:CSV 以 max_digits10 全精度写出,glibc 的 libm 会按 CPU 选择
    // 带 / 不带 FMA 的 sin / cos 实现,本机与 CI runner 的末位可能不同;而真正的行为
    // 变化(RNG 消耗顺序、逻辑改动)造成的差异远大于 1e-9。
    // ---------------------------------------------------------------------------
    struct CsvCompareOptions
    {
        double rel_tol{1e-9};
        std::vector<std::string> ignored_comment_keys{"generated_at"};
        std::size_t max_reported{10}; // 只保留前 N 条 mismatch 明细,总数照计
    };

    struct CsvMismatch
    {
        std::size_t line{0};    // 1-based;0 表示文件级(如行数不同)
        std::string column;     // 列名;注释行 / 结构性问题为空
        std::string expected;
        std::string actual;
        std::string reason;
    };

    struct CsvCompareReport
    {
        std::vector<CsvMismatch> mismatches;  // 至多 max_reported 条
        std::size_t mismatch_count{0};        // 全部 mismatch 数
        std::size_t lines_compared{0};
        std::size_t fields_compared{0};
        // 所有浮点字段中 |a − b| / max(1, |a|) 的最大值;同机运行应为 0。
        double max_scaled_diff{0.0};

        [[nodiscard]] bool ok() const noexcept { return mismatch_count == 0; }
    };

    [[nodiscard]] CsvCompareReport compare_csv(std::istream& expected, std::istream& actual,
                                               const CsvCompareOptions& opts = {});
}
