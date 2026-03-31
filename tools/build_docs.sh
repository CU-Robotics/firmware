#!/bin/bash

echo "Building documentation..."

mkdir -p docs

rm -f docs/doxygen_warnings.txt

# Generate documentation using Doxygen
doxygen Doxyfile > /dev/null # This will generate generate documentation in docs/ and warnings in docs/doxygen_warnings.txt

if [ -s docs/doxygen_warnings.txt ]; then
    echo "----Doxygen warnings:----"
    cat docs/doxygen_warnings.txt
fi

echo "Documentation build complete. Check the 'docs' directory for output."