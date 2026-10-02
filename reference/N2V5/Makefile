# Makefile v6

SHELL := /bin/bash

CXX ?= c++

HOST_CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror -pedantic -I. -Isrc
COVERAGE_FLAGS := -fprofile-instr-generate -fcoverage-mapping

HOST_TEST_FILES := $(wildcard host_tests/*.cpp)
HOST_COMMON_SOURCES := $(filter-out src/SystemProfile_%.cpp,$(wildcard src/*.cpp))
COVERAGE_REPORT_SOURCES := src/TimedStateMachine.cpp src/O2Controller.cpp src/TowerController.cpp src/N2Controller.cpp

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
	for src in $(COVERAGE_REPORT_SOURCES); do \
		cmd+=("$$src"); \
	done; \
	"$${cmd[@]}"; \
	echo; \
	echo "Per-file coverage detail commands:"; \
	for src in $(COVERAGE_REPORT_SOURCES); do \
		echo "  llvm-cov show $${bins[0]} -instr-profile=build/host/tests.profdata $$src"; \
	done

clean:
	@rm -rf build

# Makefile v6