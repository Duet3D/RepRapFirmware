# RepRapFirmware host-native test build

CXX ?= g++-15

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
	-I$(CURDIR)/support

# RRFLibraries/src/General must NOT be on the include path: it holds String.h, which on a
# case-insensitive filesystem (macOS) libc++'s <cstring> finds in place of the real <string.h>.
# Sources reach those headers through the General/ and Math/ prefixes instead.
CXXFLAGS ?= -std=gnu++20 -O2 -Wall -Wextra -Wpedantic

# unit/test_fopdt.cpp is excluded: it targets the pre-3.7.0-beta.3 FopDt API (CorrectPwmForVoltage,
# SetM301PidParameters, ArePidParametersOverridden, AppendM301Command, 3-argument EstimateRequiredPwm),
# all of which have since changed. It needs updating by whoever owns the heater model.
TEST_SOURCES := \
	support/TestMain.cpp \
	unit/test_bitmap.cpp \
	unit/test_deviation.cpp \
	unit/test_isqrt.cpp

LIB_SOURCES := \
	$(WORKSPACE)/RRFLibraries/src/General/SafeVsnprintf.cpp \
	$(WORKSPACE)/RRFLibraries/src/General/StringFunctions.cpp \
	$(WORKSPACE)/RRFLibraries/src/General/StringRef.cpp \
	$(WORKSPACE)/RRFLibraries/src/General/Strnlen.cpp \
	$(WORKSPACE)/RRFLibraries/src/Math/Deviation.cpp \
	$(WORKSPACE)/RRFLibraries/src/Math/Isqrt.cpp

OBJECTS := $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(TEST_SOURCES)) \
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