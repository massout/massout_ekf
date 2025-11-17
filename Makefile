# Depo ana dalı `main` olduğu için buradaki hızlı komutlar doğrudan güncel kodla çalışır.
CXX ?= g++
CXXFLAGS ?= -std=c++11 -Wall -Wextra -Iinclude -Isrc
LDFLAGS ?=

BUILD_DIR := build
TEST_DIR := $(BUILD_DIR)/tests
EXAMPLE_DIR := $(BUILD_DIR)/examples

TEST_SOURCES := tests/constant_velocity.cpp \
                tests/variable_acceleration.cpp \
                tests/multiple_filters.cpp \
                tests/value_access.cpp

TEST_BINS := $(patsubst tests/%.cpp,$(TEST_DIR)/%,$(TEST_SOURCES))

EXAMPLE_SOURCES := examples/basic_usage.cpp
EXAMPLE_BINS := $(patsubst examples/%.cpp,$(EXAMPLE_DIR)/%,$(EXAMPLE_SOURCES))

.PHONY: all clean tests examples

all: tests examples

$(TEST_DIR) $(EXAMPLE_DIR):
	@mkdir -p $@

$(TEST_DIR)/%: tests/%.cpp | $(TEST_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS)

$(EXAMPLE_DIR)/%: examples/%.cpp | $(EXAMPLE_DIR)
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS)

tests: $(TEST_BINS)

examples: $(EXAMPLE_BINS)

clean:
	rm -rf $(BUILD_DIR)
