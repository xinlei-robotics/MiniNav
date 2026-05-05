#include <gtest/gtest.h>

#include <random>

import mininav.core.random;

TEST(RngFactory, SameSeedSameTagProducesSameSequence) {
    mininav::RngFactory f1{42};
    mininav::RngFactory f2{42};
    auto e1 = f1.make_engine("actuator_v");
    auto e2 = f2.make_engine("actuator_v");
    for (int i = 0; i < 100; ++i) {
        EXPECT_EQ(e1(), e2());
    }
}

TEST(RngFactory, DifferentTagsProduceDifferentSequences) {
    mininav::RngFactory f{42};
    auto e_a = f.make_engine("actuator_v");
    auto e_b = f.make_engine("encoder_slip_left");
    EXPECT_NE(e_a(), e_b());
}

TEST(RngFactory, DifferentMasterSeedsProduceDifferentSequences) {
    mininav::RngFactory f1{42};
    mininav::RngFactory f2{43};
    auto e1 = f1.make_engine("same_tag");
    auto e2 = f2.make_engine("same_tag");
    EXPECT_NE(e1(), e2());
}

TEST(RngFactory, AddingNewTagDoesNotPerturbExistingTags) {
    mininav::RngFactory f{42};
    auto e_existing_before = f.make_engine("actuator_v");
    std::vector<std::uint32_t> baseline;
    baseline.reserve(50);
    for (int i = 0; i < 50; ++i) baseline.push_back(e_existing_before());

    [[maybe_unused]] auto e_new = f.make_engine("future_imu_bias");

    auto e_existing_after = f.make_engine("actuator_v");
    for (int i = 0; i < 50; ++i) {
        EXPECT_EQ(baseline[static_cast<size_t>(i)], e_existing_after());
    }
}

TEST(Fnv1a, KnownVectors) {
    // FNV-1a 64-bit known test vectors (from the FNV reference).
    EXPECT_EQ(mininav::fnv1a_64(""), 0xcbf29ce484222325ULL);
    EXPECT_EQ(mininav::fnv1a_64("a"), 0xaf63dc4c8601ec8cULL);
}