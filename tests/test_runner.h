#ifndef TEST_RUNNER_H
#define TEST_RUNNER_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

static int tests_run = 0;
static int tests_failed = 0;

#define TEST_START(name) printf("Running test: %s... ", name); tests_run++
#define TEST_PASS() printf("PASSED\n")
#define TEST_FAIL(msg) { printf("FAILED: %s at %d (%s)\n", __FILE__, __LINE__, msg); tests_failed++; }

#define ASSERT_TRUE(cond) if (!(cond)) { TEST_FAIL(#cond); return; }
#define ASSERT_INT_EQUAL(expected, actual) if ((expected) != (actual)) { printf("Expected %d, got %d. ", (int)expected, (int)actual); TEST_FAIL("Int mismatch"); return; }
#define ASSERT_FLOAT_EQUAL(expected, actual, delta) if (fabs((expected) - (actual)) > (delta)) { printf("Expected %f, got %f. ", (float)expected, (float)actual); TEST_FAIL("Float mismatch"); return; }

#endif
