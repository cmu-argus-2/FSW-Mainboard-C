/* Always passes. Tests that CI is running properly. */
#include <zephyr/ztest.h>

ZTEST_SUITE(empty, NULL, NULL, NULL, NULL, NULL);

ZTEST(empty, test_true)
{
	zassert_true(1);
}
