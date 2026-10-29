#ifndef TESTS_EQUIVALENCE_UTILS_H_
#define TESTS_EQUIVALENCE_UTILS_H_

#include <stdio.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <string>

namespace plaits {
namespace tests {

struct TestStats {
    double max_diff = 0.0;
    double sum_sq_diff = 0.0;
    double max_sig_orig = 0.0;
    double max_sig_opt = 0.0;
    size_t total_samples = 0;

    void Update(float orig, float opt) {
        double d_orig = fabs(orig);
        double d_opt = fabs(opt);
        if (d_orig > max_sig_orig) max_sig_orig = d_orig;
        if (d_opt > max_sig_opt) max_sig_opt = d_opt;

        double d = fabs(orig - opt);
        if (d > max_diff) max_diff = d;
        sum_sq_diff += d * d;
        total_samples++;
    }

    double RMS() const {
        return total_samples > 0 ? sqrt(sum_sq_diff / total_samples) : 0.0;
    }

    void PrintSummary() const {
        printf("----------------------------------------------------------------------\n");
        printf("Max Orig Sig: %10.6f, Max Opt Sig: %10.6f\n", max_sig_orig, max_sig_opt);
        printf("Overall Max Difference: %10.6f\n", max_diff);
        printf("Overall RMS Difference: %10.6f\n", RMS());
    }

    bool Success(double tolerance = 1e-3) const {
        if (max_diff < tolerance) {
            printf("Verification SUCCESS: Optimized output matches original baseline.\n");
            return true;
        } else {
            printf("Verification FAILED: Significant difference detected.\n");
            return false;
        }
    }
};

}  // namespace tests
}  // namespace plaits

#endif  // TESTS_EQUIVALENCE_UTILS_H_
