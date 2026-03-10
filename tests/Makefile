# RepRapFirmware host-native test build

CXX := g++-15

ifeq ($(V),1)
	Q :=
else
	Q := @
endif

ROOT := ..
WORKSPACE := ../..
BUILD_DIR := build
TARGET := $(BUILD_DIR)/rrf_host_tests

CPPFLAGS := \
	-I$(CURDIR)/support/shims \
	-I$(ROOT) \
	-I$(ROOT)/src \
	-I$(WORKSPACE)/CANlib/src \
	-I$(WORKSPACE)/RRFLibraries/src \
	-I$(WORKSPACE)/RRFLibraries/src/General \
	-I$(WORKSPACE)/RRFLibraries/src/Math \
	-I$(CURDIR)/support

CXXFLAGS ?= -std=gnu++20 -O2 -Wall -Wextra -Wpedantic

TEST_SOURCES := \
	support/TestMain.cpp \
	unit/test_bitmap.cpp \
	unit/test_deviation.cpp \
	unit/test_fopdt.cpp \
	unit/test_isqrt.cpp

LIB_SOURCES := \
	$(ROOT)/src/Heating/FOPDT.cpp \
	$(WORKSPACE)/RRFLibraries/src/General/SafeVsnprintf.cpp \
	$(WORKSPACE)/RRFLibraries/src/General/StringFunctions.cpp \
	$(WORKSPACE)/RRFLibraries/src/General/StringRef.cpp \
	$(WORKSPACE)/RRFLibraries/src/General/Strnlen.cpp \
	$(WORKSPACE)/RRFLibraries/src/Math/Deviation.cpp \
	$(WORKSPACE)/RRFLibraries/src/Math/Isqrt.cpp

OBJECTS := $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(TEST_SOURCES)) \
	$(BUILD_DIR)/reprapfirmware/FOPDT.o \
	$(BUILD_DIR)/rrflibraries/SafeVsnprintf.o \
	$(BUILD_DIR)/rrflibraries/StringFunctions.o \
	$(BUILD_DIR)/rrflibraries/StringRef.o \
	$(BUILD_DIR)/rrflibraries/Strnlen.o \
	$(BUILD_DIR)/rrflibraries/Deviation.o \
	$(BUILD_DIR)/rrflibraries/Isqrt.o

.PHONY: all clean run

all: run

run: $(TARGET)
	$(Q)./$(TARGET)

$(TARGET): $(OBJECTS)
	$(Q)mkdir -p $(dir $@)
	$(Q)$(CXX) $(CXXFLAGS) $^ -o $@

$(BUILD_DIR)/support/%.o: support/%.cpp support/TestRunner.h
	$(Q)mkdir -p $(dir $@)
	$(Q)$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/unit/%.o: unit/%.cpp support/TestRunner.h
	$(Q)mkdir -p $(dir $@)
	$(Q)$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/rrflibraries/Isqrt.o: $(WORKSPACE)/RRFLibraries/src/Math/Isqrt.cpp
	$(Q)mkdir -p $(dir $@)
	$(Q)$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/rrflibraries/SafeVsnprintf.o: $(WORKSPACE)/RRFLibraries/src/General/SafeVsnprintf.cpp
	$(Q)mkdir -p $(dir $@)
	$(Q)$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/rrflibraries/StringFunctions.o: $(WORKSPACE)/RRFLibraries/src/General/StringFunctions.cpp
	$(Q)mkdir -p $(dir $@)
	$(Q)$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/rrflibraries/StringRef.o: $(WORKSPACE)/RRFLibraries/src/General/StringRef.cpp
	$(Q)mkdir -p $(dir $@)
	$(Q)$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/rrflibraries/Strnlen.o: $(WORKSPACE)/RRFLibraries/src/General/Strnlen.cpp
	$(Q)mkdir -p $(dir $@)
	$(Q)$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/rrflibraries/Deviation.o: $(WORKSPACE)/RRFLibraries/src/Math/Deviation.cpp
	$(Q)mkdir -p $(dir $@)
	$(Q)$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/reprapfirmware/FOPDT.o: $(ROOT)/src/Heating/FOPDT.cpp
	$(Q)mkdir -p $(dir $@)
	$(Q)$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

clean:
	$(Q)rm -rf $(BUILD_DIR)