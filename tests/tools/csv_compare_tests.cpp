import mininav.tools.csv_compare;

#include <gtest/gtest.h>

#include <sstream>
#include <string>

namespace {

using mininav::tools::compare_csv;
using mininav::tools::CsvCompareOptions;
using mininav::tools::CsvCompareReport;

// 最小的 golden:元数据注释 + 列名行 + 整数 / 浮点 / 文本三类字段。
const std::string kGolden =
    "# MiniNav trajectory\n"
    "# seed = 42\n"
    "# generated_at = 2026-09-29T10:00:00Z\n"
    "t,enc_dl,truth_x,regime\n"
    "0.00000000000000000e+00,141,1.00000000000000000e+00,track\n"
    "1.00000000000000002e-02,97,2.75743840216313846e-02,rotate\n";

[[nodiscard]] CsvCompareReport compare(const std::string& golden, const std::string& actual,
                                       const CsvCompareOptions& opts = {}) {
  std::istringstream g{golden};
  std::istringstream a{actual};
  return compare_csv(g, a, opts);
}

// 把 kGolden 中第一处 from 替换为 to。
[[nodiscard]] std::string with(const std::string& from, const std::string& to) {
  std::string s = kGolden;
  s.replace(s.find(from), from.size(), to);
  return s;
}

} // namespace

// ---------------------------------------------------------------------------
// 应判相等的情形
// ---------------------------------------------------------------------------

TEST(CsvCompare, IdenticalFilesAreEqual) {
  const CsvCompareReport r = compare(kGolden, kGolden);
  EXPECT_TRUE(r.ok());
  EXPECT_EQ(r.lines_compared, 6U);
  EXPECT_EQ(r.fields_compared, 8U);
  EXPECT_EQ(r.max_scaled_diff, 0.0);
}

// generated_at 是墙钟时间,每次运行都不同,必须被忽略。
TEST(CsvCompare, GeneratedAtLineIsIgnored) {
  EXPECT_TRUE(compare(kGolden, with("2026-09-29T10:00:00Z", "2031-01-01T00:00:00Z")).ok());
}

// 末位差异(跨机器 libm 的 FMA / 非 FMA 实现)在 1e-9 相对容差内。
TEST(CsvCompare, FloatWithinRelativeToleranceIsEqual) {
  const CsvCompareReport r =
      compare(kGolden, with("2.75743840216313846e-02", "2.75743840216314846e-02"));  // ~3 ulp
  EXPECT_TRUE(r.ok());
  EXPECT_GT(r.max_scaled_diff, 0.0);
  EXPECT_LT(r.max_scaled_diff, 1e-15);
}

// 容差是 rel_tol · max(1, |golden|):大数按相对误差,零附近按绝对误差。
TEST(CsvCompare, ToleranceIsRelativeForLargeValuesAndAbsoluteNearZero) {
  const std::string golden = "a,b\n1e6,1e-12\n";
  EXPECT_TRUE(compare(golden, "a,b\n1000000.0005,1e-12\n").ok());  // 5e-10 相对
  EXPECT_TRUE(compare(golden, "a,b\n1e6,5e-10\n").ok());           // 5e-10 绝对
  EXPECT_FALSE(compare(golden, "a,b\n1000000.005,1e-12\n").ok());  // 5e-9 相对
  EXPECT_FALSE(compare(golden, "a,b\n1e6,5e-9\n").ok());           // 5e-9 绝对
}

TEST(CsvCompare, EquivalentNumericSpellingsAreEqual) {
  EXPECT_TRUE(compare("a,b\n1,nan\n", "a,b\n1.0,nan\n").ok());
}

// ---------------------------------------------------------------------------
// 必须报出的差异 —— 回归护栏只有在"能失败"时才有意义
// ---------------------------------------------------------------------------

TEST(CsvCompare, FloatBeyondToleranceIsReportedWithColumnName) {
  const CsvCompareReport r =
      compare(kGolden, with("2.75743840216313846e-02", "2.75753840216313846e-02"));  // 1e-6
  ASSERT_FALSE(r.ok());
  ASSERT_EQ(r.mismatches.size(), 1U);
  EXPECT_EQ(r.mismatches[0].line, 6U);
  EXPECT_EQ(r.mismatches[0].column, "truth_x");
  EXPECT_NE(r.mismatches[0].reason.find("exceeds tolerance"), std::string::npos);
}

// 编码器 tick 等整数列不走容差:差 1 也是行为变化。
TEST(CsvCompare, IntegerColumnsCompareExactly) {
  const CsvCompareReport r = compare(kGolden, with(",141,", ",142,"));
  ASSERT_FALSE(r.ok());
  EXPECT_EQ(r.mismatches[0].column, "enc_dl");
  EXPECT_EQ(r.mismatches[0].reason, "integer differs");
}

TEST(CsvCompare, TextFieldsCompareExactly) {
  const CsvCompareReport r = compare(kGolden, with("rotate", "approach"));
  ASSERT_FALSE(r.ok());
  EXPECT_EQ(r.mismatches[0].column, "regime");
  EXPECT_EQ(r.mismatches[0].reason, "text differs");
}

TEST(CsvCompare, MetadataLineMustMatchExactly) {
  const CsvCompareReport r = compare(kGolden, with("# seed = 42", "# seed = 43"));
  ASSERT_FALSE(r.ok());
  EXPECT_EQ(r.mismatches[0].line, 2U);
  EXPECT_EQ(r.mismatches[0].reason, "metadata line differs");
}

TEST(CsvCompare, ColumnHeaderMustMatchExactly) {
  const CsvCompareReport r = compare(kGolden, with("truth_x", "truth_y"));
  ASSERT_FALSE(r.ok());
  EXPECT_EQ(r.mismatches[0].reason, "column header differs");
}

TEST(CsvCompare, MissingRowIsReported) {
  const std::string truncated = kGolden.substr(0, kGolden.rfind("1.00000000000000002e-02"));
  const CsvCompareReport r = compare(kGolden, truncated);
  ASSERT_FALSE(r.ok());
  EXPECT_EQ(r.mismatches[0].reason, "line count differs");
}

TEST(CsvCompare, FieldCountMismatchIsReported) {
  const CsvCompareReport r = compare(kGolden, with(",track", ",track,extra"));
  ASSERT_FALSE(r.ok());
  EXPECT_EQ(r.mismatches[0].reason, "field count differs");
}

TEST(CsvCompare, NanOnOneSideIsReported) {
  const CsvCompareReport r = compare("a\n1.5\n", "a\nnan\n");
  ASSERT_FALSE(r.ok());
  EXPECT_EQ(r.mismatches[0].reason, "NaN on one side only");
}

// CRLF 工作区副本是 .gitattributes 要防的问题;万一发生,报告要直指原因。
TEST(CsvCompare, CrlfLineEndingIsDiagnosed) {
  std::string crlf;
  for (const char c : kGolden) {
    if (c == '\n') {
      crlf += '\r';
    }
    crlf += c;
  }
  const CsvCompareReport r = compare(crlf, kGolden);
  ASSERT_FALSE(r.ok());
  EXPECT_NE(r.mismatches[0].reason.find("CRLF"), std::string::npos);
}

TEST(CsvCompare, DetailsAreCappedButCountIsTotal) {
  CsvCompareOptions opts{};
  opts.max_reported = 1;
  std::string actual = with(",141,", ",142,");
  actual.replace(actual.find(",97,"), 4, ",98,");
  const CsvCompareReport r = compare(kGolden, actual, opts);
  EXPECT_EQ(r.mismatch_count, 2U);
  EXPECT_EQ(r.mismatches.size(), 1U);
}
