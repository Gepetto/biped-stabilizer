#define BOOST_TEST_MODULE biped_stabilizer test boost
#include <boost/test/included/unit_test.hpp>
#include "biped-stabilizer/cop_stabilizer.hpp"


BOOST_AUTO_TEST_CASE(test_default_constructor) {
    biped_stabilizer::CopStabilizer stabilizer;
    biped_stabilizer::CopStabilizerSettings default_settings;
    BOOST_CHECK(true);
    BOOST_CHECK(default_settings ==
                      stabilizer.getSettings());
    }

BOOST_AUTO_TEST_CASE(test_dummy_2) { BOOST_CHECK(true); }
