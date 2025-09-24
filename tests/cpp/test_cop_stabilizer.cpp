#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "biped-stabilizer/cop_stabilizer.hpp"

TEST_CASE("FlexEstimatorTest - DefaultConstructor") {
  biped_stabilizer::CopStabilizer stabilizer;
  biped_stabilizer::CopStabilizerSettings default_settings;
  CHECK(default_settings == stabilizer.getSettings());
}
