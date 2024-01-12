#!/usr/bin/env bash

set -ex

dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

proj_dir=$(dirname "$dir")
build_dir=$(pwd)

html_dest="$proj_dir/docs/Coverage"
dest="$build_dir/coverage"
mkdir -p "$dest"
mkdir -p "$html_dest"

rm -f "$dest/*.info"
rm -rf "$html_dest/*"

# Parse command line arguments

compiler="${1,,}"
version="${2%%.*}"

if [ ! -z "$version" ]; then version="-${version}"; fi

echo "Compiler: ${compiler}${version}"

# If the compiler is Clang, use a wrapper around llvm-cov that emulates gcov
# and use the right c++filt
if [ "${compiler}" == "clang" ]; then
    mkdir -p "/tmp/clang-cxxfilt-gcov"
    echo -e "#!/usr/bin/env sh\nexec llvm-cov${version} gcov \"\$@\"" \
        > "/tmp/clang-cxxfilt-gcov/llvm-cov"
    chmod +x "/tmp/clang-cxxfilt-gcov/llvm-cov"
    # Replace the default c++filt program with LLVM/Clang's version
    ln -sfn $(which llvm-cxxfilt${version}) /tmp/clang-cxxfilt-gcov/c++filt
    export PATH="/tmp/clang-cxxfilt-gcov:$PATH"
    gcov_bin="llvm-cov"
else
    gcov_bin="gcov${version}"
fi

branches=0

# Reset counters
lcov \
    --zerocounters \
    --directory "$build_dir"

# Initial capture
lcov \
    --capture --initial \
    --directory "$build_dir" \
    --include "$proj_dir"'/src/Filters/**' \
    --output-file "$dest"/coverage_base.info \
    --gcov-tool "$gcov_bin" \
    --rc lcov_branch_coverage=$branches

# Run tests
ctest

# Actual capture
lcov \
    --capture \
    --directory "$build_dir" \
    --include "$proj_dir"'/src/Filters/**' \
    --output-file "$dest"/coverage_test.info \
    --gcov-tool "$gcov_bin" \
    --rc lcov_branch_coverage=$branches

# Combine captures
lcov \
    --add-tracefile "$dest"/coverage_base.info \
    --add-tracefile "$dest"/coverage_test.info \
    --output-file "$dest"/coverage_total.info \
    --gcov-tool "$gcov_bin" \
    --rc lcov_branch_coverage=$branches

# Generate HTML coverage report
genhtml \
    --prefix "$proj_dir" \
    "$dest"/coverage_total.info \
    --output-directory="$html_dest" \
    --legend --title `cd "$proj_dir" && git rev-parse HEAD` \
    --rc lcov_branch_coverage=$branches \
    -s \
    --demangle-cpp

python3 "$proj_dir/scripts/coverage-badge.py"
