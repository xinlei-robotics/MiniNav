import mininav.core.types;
import mininav.core.csv_format;

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream ss{line};
  std::string field;
  while (std::getline(ss, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

}  // namespace

// 列名与数据列一一对应;Python 脚本按列名读取,错位会悄悄读错列。
TEST(CsvFormat, SimStateHeaderMatchesRowWidth) {
  const mininav::SimState state{};
  EXPECT_EQ(split(mininav::csv_header(state)).size(), 29u);
  EXPECT_EQ(split(mininav::csv_row(state)).size(), 29u);
}

// nav.csv:SimState 的全部列在前,导航诊断列在后(不改 SimState、不新增 SimStateV4)。
TEST(CsvFormat, NavStepExtendsSimStateColumns) {
  mininav::NavStep step{};
  step.nav.regime = "approach";
  step.nav.e_ctrl = 0.25;
  step.nav.clearance = 0.5;

  const std::string header = mininav::csv_header(step);
  const std::string sim_header = mininav::csv_header(mininav::SimState{});
  EXPECT_EQ(header.rfind(sim_header + ",", 0), 0u);

  const std::vector<std::string> names = split(header);
  const std::vector<std::string> values = split(mininav::csv_row(step));
  ASSERT_EQ(names.size(), 41u);
  ASSERT_EQ(values.size(), names.size());

  const auto column = [&](const std::string& name) {
    const auto it = std::ranges::find(names, name);
    return values[static_cast<std::size_t>(it - names.begin())];
  };
  EXPECT_EQ(column("regime"), "approach");
  EXPECT_DOUBLE_EQ(std::stod(column("e_ctrl")), 0.25);
  EXPECT_DOUBLE_EQ(std::stod(column("clearance")), 0.5);
}
