# Makefile v5

SHELL := /bin/bash

CXX ?= c++

HOST_CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror -pedantic -I.
COVERAGE_FLAGS := -fprofile-instr-generate -fcoverage-mapping

HOST_TEST_FILES := $(wildcard host_tests/*.cpp)
HOST_COMMON_SOURCES := $(wildcard *.cpp)

.PHONY: host-test coverage clean

host-test:
	@set -euo pipefail; \
	mkdir -p build/host; \
	for test_src in $(HOST_TEST_FILES); do \
		test_name="$$(basename "$$test_src" .cpp)"; \
		test_bin="build/host/$$test_name"; \
		$(CXX) $(HOST_CXXFLAGS) "$$test_src" $(HOST_COMMON_SOURCES) -o "$$test_bin"; \
		"$$test_bin"; \
	done

coverage:
	@set -euo pipefail; \
	mkdir -p build/host; \
	rm -f build/host/*.profraw build/host/tests.profdata; \
	bins=(); \
	for test_src in $(HOST_TEST_FILES); do \
		test_name="$$(basename "$$test_src" .cpp)"; \
		test_bin="build/host/$$test_name"; \
		$(CXX) $(HOST_CXXFLAGS) $(COVERAGE_FLAGS) "$$test_src" $(HOST_COMMON_SOURCES) -o "$$test_bin"; \
		LLVM_PROFILE_FILE="build/host/$${test_name}-%p.profraw" "$$test_bin"; \
		bins+=("$$test_bin"); \
	done; \
	llvm-profdata merge -sparse build/host/*.profraw -o build/host/tests.profdata; \
	cmd=(llvm-cov report "$${bins[0]}" -instr-profile=build/host/tests.profdata); \
	for ((i = 1; i < $${#bins[@]}; ++i)); do \
		cmd+=(-object "$${bins[$$i]}"); \
	done; \
	for src in $(HOST_COMMON_SOURCES); do \
		cmd+=("$$src"); \
	done; \
	"$${cmd[@]}"; \
	echo; \
	echo "Coverage detail example:"; \
	echo "  llvm-cov show $${bins[0]} -instr-profile=build/host/tests.profdata N2Controller.cpp"

clean:
	@rm -rf build

# Makefile v5