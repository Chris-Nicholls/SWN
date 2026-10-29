#!/bin/bash

EXECUTABLE="./tests/test_plaits"
SUMMARY="tests/engine_check_summary.txt"

echo "Plaits Engine Exhaustive Check - $(date)" > $SUMMARY
echo "----------------------------------------" >> $SUMMARY

if [ ! -f "$EXECUTABLE" ]; then
    echo "Executable $EXECUTABLE not found. Running make..."
    make -f tests/Makefile.host tests/test_plaits
fi

for i in {0..23}
do
    echo -n "Checking Engine $i... "
    # Run test and capture output. We use ASAN_OPTIONS to ensure it reports and exits.
    OUTPUT=$($EXECUTABLE $i 2>&1)
    EXIT_CODE=$?
    
    if [ $EXIT_CODE -eq 0 ]; then
        echo "OK"
        echo "Engine $i: PASS" >> $SUMMARY
    else
        echo "FAILED"
        echo "Engine $i: FAIL (Exit Code $EXIT_CODE)" >> $SUMMARY
        # Extract the ASan error if possible
        ERROR=$(echo "$OUTPUT" | grep -m 1 "ERROR: AddressSanitizer")
        LOC=$(echo "$OUTPUT" | grep -m 1 "    #0")
        echo "   $ERROR" >> $SUMMARY
        echo "   $LOC" >> $SUMMARY
    fi
done

echo "----------------------------------------" >> $SUMMARY
echo "Check completed. See $SUMMARY for details."
cat $SUMMARY
