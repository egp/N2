# Makefile v3

SHELL := /bin/bash

CXX ?= c++

HOST_CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror -pedantic -I.

HOST_TEST_FILES := $(wildcard host_tests/*.cpp)

HOST_COMMON_SOURCES := TimedStateMachine.cpp TowerController.cpp N2Controller.cpp O2Handler.cpp

.PHONY: host-test clean

host-test:
	@set -euo pipefail; \
	mkdir -p build/host; \
	for test_src in $(HOST_TEST_FILES); do \
	test_name="$$(basename "$$test_src" .cpp)"; \
	test_bin="build/host/$$test_name"; \
	$(CXX) $(HOST_CXXFLAGS) "$$test_src" $(HOST_COMMON_SOURCES) -o "$$test_bin"; \
	"$$test_bin"; \
	done

clean:
	@rm -rf build
# Makefile v3