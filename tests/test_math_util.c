#include "test_runner.h"
#include "math_util.h"

void test_clamp_i8(void) {
    TEST_START("test_clamp_i8");
    ASSERT_INT_EQUAL(10, _CLAMP_I8(15, 0, 10));
    ASSERT_INT_EQUAL(0, _CLAMP_I8(-5, 0, 10));
    ASSERT_INT_EQUAL(5, _CLAMP_I8(5, 0, 10));
    TEST_PASS();
}

void test_wrap_u16(void) {
    TEST_START("test_wrap_u16");
    ASSERT_INT_EQUAL(5, _WRAP_U16(15, 0, 10));
    ASSERT_INT_EQUAL(0, _WRAP_U16(20, 0, 10));
    ASSERT_INT_EQUAL(5, _WRAP_U16(5, 0, 10));
    TEST_PASS();
}

void test_fold_f(void) {
    TEST_START("test_fold_f");
    ASSERT_FLOAT_EQUAL(0.5f, _FOLD_F(0.5f, 1.0f), 0.001f);
    ASSERT_FLOAT_EQUAL(0.5f, _FOLD_F(1.5f, 1.0f), 0.001f);
    ASSERT_FLOAT_EQUAL(0.0f, _FOLD_F(2.0f, 1.0f), 0.001f);
    TEST_PASS();
}

int main(void) {
    test_clamp_i8();
    test_wrap_u16();
    test_fold_f();

    if (tests_failed > 0) {
        printf("\nFAILED: %d tests failed out of %d\n", tests_failed, tests_run);
        return 1;
    } else {
        printf("\nPASSED: %d tests passed\n", tests_run);
        return 0;
    }
}
