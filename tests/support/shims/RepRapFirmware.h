#ifndef RRF_HOST_TEST_REPRAPFIRMWARE_H
#define RRF_HOST_TEST_REPRAPFIRMWARE_H

#include <cstdarg>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "../../../../RRFLibraries/src/ecv_duet3d.h"
#include "../../../../RRFLibraries/src/General/SimpleMath.h"
#include "../../../../RRFLibraries/src/General/String.h"
#include "../../../../RRFLibraries/src/General/StringRef.h"

#define SUPPORT_OBJECT_MODEL 0
#define HAS_MASS_STORAGE 0
#define HAS_SBC_INTERFACE 0
#define SUPPORT_CAN_EXPANSION 0

#ifndef THROWS
#define THROWS(...) noexcept(false)
#endif

#ifndef INHERIT_OBJECT_MODEL
#define INHERIT_OBJECT_MODEL
#endif

#ifndef DECLARE_OBJECT_MODEL
#define DECLARE_OBJECT_MODEL
#endif

constexpr float NormalAmbientTemperature = 25.0;
constexpr unsigned int MaxFloatDigitsDisplayedAfterPoint = 3;

#define DEGREE_SYMBOL " "

#endif