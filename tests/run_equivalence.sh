#!/bin/bash

# List of equivalence test binaries to run
TESTS=(
    "tests/string_equivalence"
    "tests/particle_equivalence"
    "tests/modal_equivalence"
    "tests/snare_equivalence"
    "tests/va_vcf_equivalence"
    "tests/noise_equivalence"
    "tests/hi_hat_equivalence"
    "tests/synthetic_snare_equivalence"
    "tests/diffuser_equivalence"
    "tests/ensemble_equivalence"
    "tests/string_machine_equivalence"
    "tests/pd_equivalence"
)

echo "======================================================================"
echo "RUNNING ALL PLAITS EQUIVALENCE TESTS"
echo "======================================================================"

RESULTS=()
FAILED=0

for test_bin in "${TESTS[@]}"; do
    if [ ! -f "$test_bin" ]; then
        echo "MISSING: $test_bin - Skipping (run 'make -f tests/Makefile.host' first)"
        RESULTS+=("MISSING: $test_bin | - | - | -")
        continue
    fi

    echo -n "Running $test_bin... "
    # Capture the output
    output=$($test_bin 2>&1)
    exit_code=$?
    
    # Extract Max and RMS errors using grep/awk
    max_err=$(echo "$output" | grep "Overall Max Difference" | awk -F: '{print $2}' | xargs)
    # Some older tests might use different wording or just print at the end
    if [ -z "$max_err" ]; then
        max_err=$(echo "$output" | tail -n 2 | grep "Difference" | head -n 1 | awk -F: '{print $2}' | xargs)
    fi
    [ -z "$max_err" ] && max_err="N/A"
    
    rms_err=$(echo "$output" | grep "Overall RMS Difference" | awk -F: '{print $2}' | xargs)
    if [ -z "$rms_err" ]; then
        rms_err=$(echo "$output" | tail -n 1 | grep "Difference" | awk -F: '{print $2}' | xargs)
    fi
    [ -z "$rms_err" ] && rms_err="N/A"

    if [ $exit_code -eq 0 ]; then
        echo "PASSED"
    else
        echo "FAILED"
        RESULTS+=("FAILED | $test_bin | $max_err | $rms_err")
        FAILED=$((FAILED + 1))
    fi
done

echo ""
echo "=========================================================================================="
echo "EQUIVALENCE TEST SUMMARY (Stats shown for failures only)"
echo "=========================================================================================="
printf "%-10s | %-35s | %-15s | %-15s\n" "STATUS" "TEST BINARY" "MAX ERROR" "RMS ERROR"
echo "------------------------------------------------------------------------------------------"
for res in "${RESULTS[@]}"; do
    IFS='|' read -r status binary max rms <<< "$res"
    printf "%-10s | %-35s | %-15s | %-15s\n" "$status" "$binary" "$max" "$rms"
done
echo "=========================================================================================="

if [ $FAILED -gt 0 ]; then
    echo "TOTAL FAILURES: $FAILED"
    exit 1
else
    echo "ALL TESTS PASSED!"
    exit 0
fi
