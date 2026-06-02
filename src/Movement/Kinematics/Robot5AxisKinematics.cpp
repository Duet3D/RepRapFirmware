/*
 * Robot5AxisKinematics.cpp
 *
 *  Created on: 7 May 2026
 */

#include "Robot5AxisKinematics.h"

#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <GCodes/GCodes.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Movement/DDA.h>
#include <Movement/Move.h>

#include <ctype.h>
#include <math.h>
#include <string.h>

/**
 * @brief Helper macro used to expose Robot5Axis object-model entries.
 */
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(Robot5AxisKinematics, __VA_ARGS__)

constexpr ObjectModelTableEntry Robot5AxisKinematics::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. kinematics members
	{ "chain", 			OBJECT_MODEL_FUNC(self->chainDescription), 				ObjectModelEntryFlags::none },
	{ "homingMode", 		OBJECT_MODEL_FUNC((self->homingMode == HomingMode::homeCartesianAxes) ? "cartesian" : "individualDrives"), ObjectModelEntryFlags::none },
	{ "ik", 			OBJECT_MODEL_FUNC(self, 1), 					ObjectModelEntryFlags::none },
	{ "jointTypes", 		OBJECT_MODEL_FUNC(self->jointTypes), 				ObjectModelEntryFlags::none },
	{ "name", 			OBJECT_MODEL_FUNC(self->GetName(true)), 				ObjectModelEntryFlags::none },
	{ "solver", 			OBJECT_MODEL_FUNC(self->useScrewSolver ? "screw" : "matrix"), 		ObjectModelEntryFlags::none },

	// 1. ik members
	{ "damping", 			OBJECT_MODEL_FUNC(self->ikDamping, 3), 				ObjectModelEntryFlags::none },
	{ "failCount", 		OBJECT_MODEL_FUNC(self->ikFailCount), 				ObjectModelEntryFlags::none },
	{ "iterations", 		OBJECT_MODEL_FUNC((uint32_t)self->ikIterationsLimit), 		ObjectModelEntryFlags::none },
	{ "lastResidual", 		OBJECT_MODEL_FUNC(self->lastIkResidual, 3), 			ObjectModelEntryFlags::none },
	{ "rotationWeight", 	OBJECT_MODEL_FUNC(self->ikRotationWeight, 3), 			ObjectModelEntryFlags::none },
	{ "solveCount", 		OBJECT_MODEL_FUNC(self->ikSolveCount), 				ObjectModelEntryFlags::none },
	{ "tolerance", 		OBJECT_MODEL_FUNC(self->ikTolerance, 3), 				ObjectModelEntryFlags::none },
};

constexpr uint8_t Robot5AxisKinematics::objectModelTableDescriptor[] = { 2, 6, 7 };

DEFINE_GET_OBJECT_MODEL_TABLE_WITH_PARENT(Robot5AxisKinematics, ZLeadscrewKinematics)

namespace
{
	static constexpr size_t PoseDof = 6;
	#define KINDBG(...) do { if (reprap.Debug(Module::Kinematics)) { debugPrintf(__VA_ARGS__); } } while(false)

	/**
	 * @brief Initialize a 3x4 transform to identity rotation and zero translation.
	 * @param[out] m Transform matrix in row-major 3x4 form.
	 */
	static inline void SetIdentity(float m[12]) noexcept
	{
		m[0] = 1.0; m[1] = 0.0; m[2] = 0.0; m[3] = 0.0;
		m[4] = 0.0; m[5] = 1.0; m[6] = 0.0; m[7] = 0.0;
		m[8] = 0.0; m[9] = 0.0; m[10] = 1.0; m[11] = 0.0;
	}

	static inline void CopyTransform(const float src[12], float dst[12]) noexcept
	{
		memcpy(dst, src, sizeof(float) * 12);
	}

	/**
	 * @brief Compose two rigid transforms.
	 * @param[in] a Left transform.
	 * @param[in] b Right transform.
	 * @param[out] out Composed transform where @p out = @p a * @p b.
	 */
	static inline void Compose(const float a[12], const float b[12], float out[12]) noexcept
	{
		const float r00 = a[0]*b[0] + a[1]*b[4] + a[2]*b[8];
		const float r01 = a[0]*b[1] + a[1]*b[5] + a[2]*b[9];
		const float r02 = a[0]*b[2] + a[1]*b[6] + a[2]*b[10];
		const float px  = a[0]*b[3] + a[1]*b[7] + a[2]*b[11] + a[3];

		const float r10 = a[4]*b[0] + a[5]*b[4] + a[6]*b[8];
		const float r11 = a[4]*b[1] + a[5]*b[5] + a[6]*b[9];
		const float r12 = a[4]*b[2] + a[5]*b[6] + a[6]*b[10];
		const float py  = a[4]*b[3] + a[5]*b[7] + a[6]*b[11] + a[7];

		const float r20 = a[8]*b[0] + a[9]*b[4] + a[10]*b[8];
		const float r21 = a[8]*b[1] + a[9]*b[5] + a[10]*b[9];
		const float r22 = a[8]*b[2] + a[9]*b[6] + a[10]*b[10];
		const float pz  = a[8]*b[3] + a[9]*b[7] + a[10]*b[11] + a[11];

		out[0] = r00; out[1] = r01; out[2] = r02; out[3] = px;
		out[4] = r10; out[5] = r11; out[6] = r12; out[7] = py;
		out[8] = r20; out[9] = r21; out[10] = r22; out[11] = pz;
	}

	static inline void Cross(const float a[3], const float b[3], float out[3]) noexcept
	{
		out[0] = a[1]*b[2] - a[2]*b[1];
		out[1] = a[2]*b[0] - a[0]*b[2];
		out[2] = a[0]*b[1] - a[1]*b[0];
	}

	static inline float Dot(const float a[3], const float b[3]) noexcept
	{
		return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
	}

	static inline float Norm(const float v[3]) noexcept
	{
		return fastSqrtf(fsquare(v[0]) + fsquare(v[1]) + fsquare(v[2]));
	}

	/**
	 * @brief Build rotation matrix from intrinsic A/B/C Euler angles in degrees.
	 * @param[in] aDeg Rotation about A axis in degrees.
	 * @param[in] bDeg Rotation about B axis in degrees.
	 * @param[in] cDeg Rotation about C axis in degrees.
	 * @param[out] out Output transform rotation terms (translation untouched by caller convention).
	 */
	static inline void RotationFromABC(float aDeg, float bDeg, float cDeg, float out[12]) noexcept
	{
		const float a = aDeg * DegreesToRadians;
		const float b = bDeg * DegreesToRadians;
		const float c = cDeg * DegreesToRadians;
		const float ca = cosf(a), sa = sinf(a);
		const float cb = cosf(b), sb = sinf(b);
		const float cc = cosf(c), sc = sinf(c);

		// R = Rz(c) * Ry(b) * Rx(a)
		out[0] = cc * cb;
		out[1] = (cc * sb * sa) - (sc * ca);
		out[2] = (cc * sb * ca) + (sc * sa);

		out[4] = sc * cb;
		out[5] = (sc * sb * sa) + (cc * ca);
		out[6] = (sc * sb * ca) - (cc * sa);

		out[8] = -sb;
		out[9] = cb * sa;
		out[10] = cb * ca;
	}

	/**
	 * @brief Recover A/B/C Euler angles in degrees from a rotation matrix.
	 * @param[in] m Input transform.
	 * @param[out] aDeg Recovered A angle in degrees.
	 * @param[out] bDeg Recovered B angle in degrees.
	 * @param[out] cDeg Recovered C angle in degrees.
	 */
	static inline void ABCFromRotation(const float m[12], float& aDeg, float& bDeg, float& cDeg) noexcept
	{
		const float r20 = m[8];
		if (fabsf(r20) < 0.99999)
		{
			const float b = -asinf(r20);
			const float a = atan2f(m[9], m[10]);
			const float c = atan2f(m[4], m[0]);
			aDeg = a * RadiansToDegrees;
			bDeg = b * RadiansToDegrees;
			cDeg = c * RadiansToDegrees;
		}
		else
		{
			const float b = (r20 < 0.0) ? (Pi * 0.5f) : (-Pi * 0.5f);
			const float a = 0.0;
			const float c = atan2f(-m[1], m[5]);
			aDeg = a * RadiansToDegrees;
			bDeg = b * RadiansToDegrees;
			cDeg = c * RadiansToDegrees;
		}
	}

	/**
	 * @brief Compute screw exponential for a revolute joint.
	 * @param[in] omega Unit direction of rotation axis.
	 * @param[in] q A point on rotation axis.
	 * @param[in] thetaRad Joint rotation in radians.
	 * @param[out] out Joint transform.
	 */
	static inline void ExpRotJoint(const float omega[3], const float q[3], float thetaRad, float out[12]) noexcept
	{
		const float wx = omega[0], wy = omega[1], wz = omega[2];
		const float ct = cosf(thetaRad), st = sinf(thetaRad), omc = 1.0f - ct;

		out[0] = ct + wx*wx*omc;
		out[1] = wx*wy*omc - wz*st;
		out[2] = wx*wz*omc + wy*st;

		out[4] = wy*wx*omc + wz*st;
		out[5] = ct + wy*wy*omc;
		out[6] = wy*wz*omc - wx*st;

		out[8] = wz*wx*omc - wy*st;
		out[9] = wz*wy*omc + wx*st;
		out[10] = ct + wz*wz*omc;

		const float wxq[3] =
		{
			(wy * q[2]) - (wz * q[1]),
			(wz * q[0]) - (wx * q[2]),
			(wx * q[1]) - (wy * q[0])
		};
		const float wdotq = Dot(omega, q);
		const float iMinusR_wxq[3] =
		{
			(1.0f - out[0]) * wxq[0] - out[1] * wxq[1] - out[2] * wxq[2],
			-out[4] * wxq[0] + (1.0f - out[5]) * wxq[1] - out[6] * wxq[2],
			-out[8] * wxq[0] - out[9] * wxq[1] + (1.0f - out[10]) * wxq[2]
		};

		out[3] = iMinusR_wxq[0] + (wx * wdotq * thetaRad);
		out[7] = iMinusR_wxq[1] + (wy * wdotq * thetaRad);
		out[11] = iMinusR_wxq[2] + (wz * wdotq * thetaRad);
	}

	/**
	 * @brief Compute screw exponential for a prismatic joint.
	 * @param[in] omega Unit direction of linear motion.
	 * @param[in] dist Linear displacement.
	 * @param[out] out Joint transform.
	 */
	static inline void ExpPrismaticJoint(const float omega[3], float dist, float out[12]) noexcept
	{
		SetIdentity(out);
		out[3] = omega[0] * dist;
		out[7] = omega[1] * dist;
		out[11] = omega[2] * dist;
	}

	/**
	 * @brief Compute weighted 6D pose residual and its norm.
	 * @param[in] current Current end-effector pose.
	 * @param[in] target Target end-effector pose.
	 * @param[in] rotWeight Weight applied to rotational residual terms.
	 * @param[out] err Residual vector [dx, dy, dz, drx, dry, drz].
	 * @return Euclidean norm of @p err.
	 */
	static inline float PoseErrorVector(const float current[12], const float target[12], float rotWeight, float err[PoseDof]) noexcept
	{
		err[0] = target[3] - current[3];
		err[1] = target[7] - current[7];
		err[2] = target[11] - current[11];

		const float c1[3] = { current[0], current[4], current[8] };
		const float c2[3] = { current[1], current[5], current[9] };
		const float c3[3] = { current[2], current[6], current[10] };
		const float t1[3] = { target[0], target[4], target[8] };
		const float t2[3] = { target[1], target[5], target[9] };
		const float t3[3] = { target[2], target[6], target[10] };

		float cx[3], cy[3], cz[3];
		Cross(c1, t1, cx);
		Cross(c2, t2, cy);
		Cross(c3, t3, cz);

		err[3] = 0.5f * (cx[0] + cy[0] + cz[0]) * rotWeight;
		err[4] = 0.5f * (cx[1] + cy[1] + cz[1]) * rotWeight;
		err[5] = 0.5f * (cx[2] + cy[2] + cz[2]) * rotWeight;

		float norm2 = 0.0;
		for (size_t i = 0; i < PoseDof; ++i)
		{
			norm2 += fsquare(err[i]);
		}
		return fastSqrtf(norm2);
	}

	/**
	 * @brief Solve dense linear system A*x=b in-place using Gauss-Jordan elimination.
	 * @param[in,out] a Row-major NxN matrix A.
	 * @param[in,out] b Right-hand-side vector b, replaced by x on success.
	 * @param[in] n Matrix dimension.
	 * @return true if system is solved successfully, false if matrix is singular.
	 */
	static bool SolveLinear(float* a, float* b, size_t n) noexcept
	{
		for (size_t i = 0; i < n; ++i)
		{
			size_t pivot = i;
			float best = fabsf(a[i*n + i]);
			for (size_t r = i + 1; r < n; ++r)
			{
				const float v = fabsf(a[r*n + i]);
				if (v > best)
				{
					best = v;
					pivot = r;
				}
			}

			if (best < 1.0e-9)
			{
				return false;
			}

			if (pivot != i)
			{
				for (size_t c = i; c < n; ++c)
				{
					const float t = a[i*n + c];
					a[i*n + c] = a[pivot*n + c];
					a[pivot*n + c] = t;
				}
				const float tb = b[i];
				b[i] = b[pivot];
				b[pivot] = tb;
			}

			const float diag = a[i*n + i];
			for (size_t c = i; c < n; ++c)
			{
				a[i*n + c] /= diag;
			}
			b[i] /= diag;

			for (size_t r = 0; r < n; ++r)
			{
				if (r == i)
				{
					continue;
				}
				const float f = a[r*n + i];
				if (f == 0.0)
				{
					continue;
				}
				for (size_t c = i; c < n; ++c)
				{
					a[r*n + c] -= f * a[i*n + c];
				}
				b[r] -= f * b[i];
			}
		}
		return true;
	}

	static inline bool IsFinite(float v) noexcept
	{
		return isfinite(v);
	}
}

/**
 * @brief Construct K13 kinematics with identity matrix mapping and default IK parameters.
 */
Robot5AxisKinematics::Robot5AxisKinematics() noexcept
	: ZLeadscrewKinematics(KinematicsType::robot5axis),
	  useScrewSolver(false), hasLastJointSolution(false), modified(false),
	  ikIterationsLimit(40), ikDamping(0.05), ikTolerance(0.03), ikRotationWeight(50.0),
	  ikSolveCount(0), ikFailCount(0), lastIkResidual(0.0),
	  homingMode(HomingMode::homeIndividualDrives)
{
	inverseMatrix.Fill(0.0);
	forwardMatrix.Fill(0.0);
	SetIdentity(toolTransform.m);

	for (size_t i = 0; i < MaxAxes; ++i)
	{
		inverseMatrix(i, i) = 1.0;
		forwardMatrix(i, i) = 1.0;
		jointTypes[i] = 'P';
		chainLetters[i] = '\0';
		chainAxisMap[i] = -1;
		chainDescription[i] = '\0';
		hasHomePositions[i] = false;
		homePositions[i] = 0.0;
		hasScrew[i] = false;
		hasJointLimits[i] = false;
		jointHome[i] = 0.0;
		jointMin[i] = 0.0;
		jointMax[i] = 0.0;
		continuousRotation[i] = false;
		lastJointSolution[i] = 0.0;

		screwOmega[i][0] = 0.0;
		screwOmega[i][1] = 0.0;
		screwOmega[i][2] = 1.0;
		screwQ[i][0] = 0.0;
		screwQ[i][1] = 0.0;
		screwQ[i][2] = 0.0;
	}
	jointTypes[MaxAxes] = '\0';
	chainLetters[MaxAxes] = '\0';
	SafeStrncpy(chainDescription, "", sizeof(chainDescription));
	Recalc();
}

/**
 * @brief Get human-readable kinematics name.
 * @param[in] forStatusReport Select status/reporting-friendly name when true.
 * @return Kinematics name string.
 */
const char *_ecv_array Robot5AxisKinematics::GetName(bool forStatusReport) const noexcept
{
	return (forStatusReport) ? "robot5axis" : "Robot5Axis";
}

int Robot5AxisKinematics::FindVisibleAxisByLetter(char c, size_t numVisibleAxes) const noexcept
{
	const char axisLetter = toupper(c);
	const char *_ecv_array letters = reprap.GetGCodes().GetAxisLetters();
	const size_t maxVisible = min<size_t>(numVisibleAxes, MaxAxes);
	for (size_t axis = 0; axis < maxVisible; ++axis)
	{
		if (toupper(letters[axis]) == axisLetter)
		{
			return (int)axis;
		}
	}
	return -1;
}

int Robot5AxisKinematics::FindChainJointByLetter(char c) const noexcept
{
	const char u = toupper(c);
	for (size_t i = 0; i < MaxAxes && chainLetters[i] != 0; ++i)
	{
		if (toupper(chainLetters[i]) == u)
		{
			return (int)i;
		}
	}
	return -1;
}

bool Robot5AxisKinematics::IsPoseLetter(char c) const noexcept
{
	const char u = toupper(c);
	return u == 'X' || u == 'Y' || u == 'Z' || u == 'A' || u == 'B' || u == 'C';
}

/**
 * @brief Validate and rebuild mapping between chain letters and visible axis indices.
 * @param[in] numVisibleAxes Number of visible machine axes.
 * @param[in,out] reply Output buffer for configuration errors.
 * @param[in,out] error Set true on parse/validation failure.
 * @return true if mapping changed, otherwise false.
 */
bool Robot5AxisKinematics::UpdateChainMapping(size_t numVisibleAxes, const StringRef& reply, bool& error) noexcept
{
	bool changed = false;
	const char *_ecv_array letters = reprap.GetGCodes().GetAxisLetters();

	if (chainLetters[0] == 0)
	{
		size_t n = min<size_t>(numVisibleAxes, MaxAxes);
		for (size_t i = 0; i < n; ++i)
		{
			chainLetters[i] = letters[i];
			if (jointTypes[i] != 'P')
			{
				jointTypes[i] = 'P';
			}
		}
		if (chainLetters[n] != 0)
		{
			changed = true;
		}
		chainLetters[n] = 0;
	}

	for (size_t i = 0; i < MaxAxes; ++i)
	{
		const int8_t old = chainAxisMap[i];
		if (chainLetters[i] == 0)
		{
			chainAxisMap[i] = -1;
		}
		else
		{
			const int axis = FindVisibleAxisByLetter(chainLetters[i], numVisibleAxes);
			if (axis < 0)
			{
				reply.printf("Chain axis letter '%c' is not currently configured as a visible axis", chainLetters[i]);
				error = true;
				return changed;
			}
			chainAxisMap[i] = (int8_t)axis;
		}
		if (old != chainAxisMap[i])
		{
			changed = true;
		}
	}

	for (size_t i = 0; i < MaxAxes; ++i)
	{
		if (chainLetters[i] == 0)
		{
			break;
		}
		for (size_t j = i + 1; j < MaxAxes && chainLetters[j] != 0; ++j)
		{
			if (toupper(chainLetters[i]) == toupper(chainLetters[j]))
			{
				reply.printf("Duplicate axis letter '%c' in chain definition", chainLetters[i]);
				error = true;
				return changed;
			}
		}
	}

	return changed;
}

/**
 * @brief Parse the B parameter that defines chain letter order.
 * @param[in] s Chain-order string.
 * @param[in] numVisibleAxes Number of visible axes.
 * @param[in,out] reply Output buffer for validation errors.
 * @param[in,out] error Set true on parse failure.
 * @return true if chain definition changed.
 */
bool Robot5AxisKinematics::ParseChain(const char* s, size_t numVisibleAxes, const StringRef& reply, bool& error) noexcept
{
	char parsed[MaxAxes + 1];
	size_t n = 0;
	for (const char* p = s; *p != 0 && n < MaxAxes; ++p)
	{
		if (!isalpha(*p))
		{
			continue;
		}
		parsed[n++] = toupper(*p);
	}
	parsed[n] = 0;

	if (n == 0)
	{
		reply.copy("B parameter must include at least one axis letter");
		error = true;
		return false;
	}

	if (n > numVisibleAxes)
	{
		reply.printf("B parameter has %u axes but only %u visible axes are configured", n, numVisibleAxes);
		error = true;
		return false;
	}

	bool changed = strcmp(parsed, chainLetters) != 0;
	if (changed)
	{
		SafeStrncpy(chainLetters, parsed, sizeof(chainLetters));
		SafeStrncpy(chainDescription, parsed, sizeof(chainDescription));
	}
	return changed;
}

bool Robot5AxisKinematics::ParseJointTypes(const char* s, size_t numVisibleAxes, const StringRef& reply, bool& error) noexcept
{
	const size_t chainLen = strlen(chainLetters);
	const size_t len = strlen(s);
	if (len > max<size_t>(chainLen, numVisibleAxes))
	{
		reply.printf("P parameter has %u entries but only %u joints are available", len, max<size_t>(chainLen, numVisibleAxes));
		error = true;
		return false;
	}

	const size_t usedLen = (chainLen != 0) ? chainLen : min<size_t>(numVisibleAxes, MaxAxes);
	bool changed = false;
	for (size_t i = 0; i < usedLen; ++i)
	{
		const char jointType = (i < len) ? (char)toupper(s[i]) : 'P';
		if (jointType != 'R' && jointType != 'P')
		{
			reply.printf("Unsupported joint type '%c' at position %u in P parameter, use only R or P", jointType, i + 1);
			error = true;
			return false;
		}
		if (jointTypes[i] != jointType)
		{
			jointTypes[i] = jointType;
			changed = true;
		}
	}
	for (size_t i = usedLen; i < MaxAxes; ++i)
	{
		if (jointTypes[i] != 'P')
		{
			jointTypes[i] = 'P';
			changed = true;
		}
	}
	jointTypes[MaxAxes] = 0;
	return changed;
}

bool Robot5AxisKinematics::ParseContinuousAxes(const char* s, size_t numVisibleAxes, const StringRef& reply, bool& error) noexcept
{
	bool newContinuous[MaxAxes];
	for (size_t axis = 0; axis < MaxAxes; ++axis)
	{
		newContinuous[axis] = false;
	}

	for (const char* p = s; *p != 0; ++p)
	{
		if (*p == ' ' || *p == ',' || *p == ';')
		{
			continue;
		}
		if (!isalpha(*p))
		{
			reply.printf("Unsupported character '%c' in R parameter", *p);
			error = true;
			return false;
		}

		const int axis = FindVisibleAxisByLetter(*p, numVisibleAxes);
		if (axis < 0)
		{
			reply.printf("Unknown axis letter '%c' in R parameter", *p);
			error = true;
			return false;
		}

		const int joint = FindChainJointByLetter(*p);
		if (joint >= 0 && jointTypes[joint] != 'R')
		{
			reply.printf("Axis '%c' is not configured as a rotary joint in P parameter", *p);
			error = true;
			return false;
		}
		newContinuous[axis] = true;
	}

	bool changed = false;
	for (size_t axis = 0; axis < MaxAxes; ++axis)
	{
		if (continuousRotation[axis] != newContinuous[axis])
		{
			continuousRotation[axis] = newContinuous[axis];
			changed = true;
		}
	}
	return changed;
}

bool Robot5AxisKinematics::ParseHomeAssignments(const char* s, size_t numVisibleAxes, const StringRef& reply, bool& error) noexcept
{
	if (s[0] == 0)
	{
		bool changed = false;
		for (size_t axis = 0; axis < MaxAxes; ++axis)
		{
			if (hasHomePositions[axis])
			{
				hasHomePositions[axis] = false;
				changed = true;
			}
		}
		return changed;
	}

	bool changed = false;
	char buffer[160];
	SafeStrncpy(buffer, s, sizeof(buffer));

	char* savePtr = nullptr;
	for (char* token = strtok_r(buffer, ",", &savePtr); token != nullptr; token = strtok_r(nullptr, ",", &savePtr))
	{
		while (*token == ' ' || *token == '\t')
		{
			++token;
		}
		if (*token == 0)
		{
			continue;
		}

		if (!isalpha(*token))
		{
			reply.copy("A parameter entries must start with an axis letter");
			error = true;
			return false;
		}

		const int axis = FindVisibleAxisByLetter(*token, numVisibleAxes);
		if (axis < 0)
		{
			reply.printf("Unknown axis letter '%c' in A parameter", *token);
			error = true;
			return false;
		}

		char* separator = strchr(token + 1, ':');
		if (separator == nullptr)
		{
			separator = strchr(token + 1, '=');
		}
		if (separator == nullptr)
		{
			reply.copy("A parameter entries must be in the format <axis>:<home> or <axis>=<home>");
			error = true;
			return false;
		}

		const float newHomePos = SafeStrtof(separator + 1);
		if (!hasHomePositions[axis] || homePositions[axis] != newHomePos)
		{
			homePositions[axis] = newHomePos;
			hasHomePositions[axis] = true;
			changed = true;
		}
	}

	return changed;
}

/**
 * @brief Parse C parameter screw definitions (omega and q) for chain joints.
 * @param[in] s Screw-definition string.
 * @param[in] numVisibleAxes Number of visible axes.
 * @param[in,out] reply Output buffer for validation errors.
 * @param[in,out] error Set true on parse failure.
 * @return true if screw definitions changed.
 */
bool Robot5AxisKinematics::ParseScrewDefinitions(const char* s, size_t numVisibleAxes, const StringRef& reply, bool& error) noexcept
{
	bool changed = false;
	char buffer[220];
	SafeStrncpy(buffer, s, sizeof(buffer));

	char* savePtr = nullptr;
	for (char* token = strtok_r(buffer, ",", &savePtr); token != nullptr; token = strtok_r(nullptr, ",", &savePtr))
	{
		while (*token == ' ' || *token == '\t')
		{
			++token;
		}
		if (*token == 0)
		{
			continue;
		}

		if (!isalpha(*token))
		{
			reply.copy("C parameter entries must start with an axis letter");
			error = true;
			return false;
		}

		const int axis = FindVisibleAxisByLetter(*token, numVisibleAxes);
		if (axis < 0)
		{
			reply.printf("Unknown axis letter '%c' in C parameter", *token);
			error = true;
			return false;
		}

		const int joint = FindChainJointByLetter(*token);
		if (joint < 0)
		{
			reply.printf("Axis '%c' is not in the current chain definition", *token);
			error = true;
			return false;
		}

		char* separator = strchr(token + 1, '=');
		if (separator == nullptr)
		{
			separator = strchr(token + 1, ':');
		}
		if (separator == nullptr)
		{
			reply.copy("C parameter entries must be in the format <axis>=ox:oy:oz:qx:qy:qz");
			error = true;
			return false;
		}

		float values[6];
		size_t count = 0;
		char* valueSave = nullptr;
		for (char* v = strtok_r(separator + 1, ":", &valueSave); v != nullptr && count < 6; v = strtok_r(nullptr, ":", &valueSave))
		{
			values[count++] = SafeStrtof(v);
		}

		if (count != 6)
		{
			reply.printf("C parameter for axis '%c' must contain 6 values", *token);
			error = true;
			return false;
		}

		for (size_t i = 0; i < 3; ++i)
		{
			if (screwOmega[joint][i] != values[i])
			{
				screwOmega[joint][i] = values[i];
				changed = true;
			}
			if (screwQ[joint][i] != values[i + 3])
			{
				screwQ[joint][i] = values[i + 3];
				changed = true;
			}
		}
		if (!hasScrew[joint])
		{
			hasScrew[joint] = true;
			changed = true;
		}
	}

	return changed;
}

bool Robot5AxisKinematics::ParseJointLimits(const char* s, size_t numVisibleAxes, const StringRef& reply, bool& error) noexcept
{
	bool changed = false;
	char buffer[220];
	SafeStrncpy(buffer, s, sizeof(buffer));

	char* savePtr = nullptr;
	for (char* token = strtok_r(buffer, ",", &savePtr); token != nullptr; token = strtok_r(nullptr, ",", &savePtr))
	{
		while (*token == ' ' || *token == '\t')
		{
			++token;
		}
		if (*token == 0)
		{
			continue;
		}

		if (!isalpha(*token))
		{
			reply.copy("L parameter entries must start with an axis letter");
			error = true;
			return false;
		}

		const int axis = FindVisibleAxisByLetter(*token, numVisibleAxes);
		if (axis < 0)
		{
			reply.printf("Unknown axis letter '%c' in L parameter", *token);
			error = true;
			return false;
		}
		const int joint = FindChainJointByLetter(*token);
		if (joint < 0)
		{
			reply.printf("Axis '%c' is not in the current chain definition", *token);
			error = true;
			return false;
		}

		char* separator = strchr(token + 1, '=');
		if (separator == nullptr)
		{
			separator = strchr(token + 1, ':');
		}
		if (separator == nullptr)
		{
			reply.copy("L parameter entries must be in the format <axis>=min:max[:home]");
			error = true;
			return false;
		}

		float values[3];
		size_t count = 0;
		char* valueSave = nullptr;
		for (char* v = strtok_r(separator + 1, ":", &valueSave); v != nullptr && count < 3; v = strtok_r(nullptr, ":", &valueSave))
		{
			values[count++] = SafeStrtof(v);
		}
		if (count < 2)
		{
			reply.printf("L parameter for axis '%c' must contain at least min:max", *token);
			error = true;
			return false;
		}
		if (values[0] > values[1])
		{
			reply.printf("L parameter for axis '%c' has min > max", *token);
			error = true;
			return false;
		}

		if (!hasJointLimits[joint] || jointMin[joint] != values[0] || jointMax[joint] != values[1])
		{
			hasJointLimits[joint] = true;
			jointMin[joint] = values[0];
			jointMax[joint] = values[1];
			changed = true;
		}
		if (count >= 3 && jointHome[joint] != values[2])
		{
			jointHome[joint] = values[2];
			changed = true;
		}
	}

	return changed;
}

bool Robot5AxisKinematics::ParseToolTransform(const char* s, const StringRef& reply, bool& error) noexcept
{
	char buffer[220];
	SafeStrncpy(buffer, s, sizeof(buffer));
	float values[12];
	size_t count = 0;
	char* savePtr = nullptr;
	for (char* v = strtok_r(buffer, ":", &savePtr); v != nullptr && count < 12; v = strtok_r(nullptr, ":", &savePtr))
	{
		values[count++] = SafeStrtof(v);
	}
	if (count != 12)
	{
		reply.copy("M parameter must contain 12 values: r11:r12:r13:px:r21:r22:r23:py:r31:r32:r33:pz");
		error = true;
		return false;
	}

	bool changed = false;
	for (size_t i = 0; i < 12; ++i)
	{
		if (toolTransform.m[i] != values[i])
		{
			toolTransform.m[i] = values[i];
			changed = true;
		}
	}
	return changed;
}

/**
 * @brief Validate configured screw axes and normalize omega vectors.
 * @param[in] numVisibleAxes Number of visible axes.
 * @param[in,out] reply Output buffer for validation errors.
 * @param[in,out] error Set true when invalid screw geometry is detected.
 * @return true on valid geometry, false on failure.
 */
bool Robot5AxisKinematics::ValidateAndNormaliseScrews(size_t numVisibleAxes, const StringRef& reply, bool& error) noexcept
{
	const size_t chainLen = strlen(chainLetters);
	for (size_t i = 0; i < chainLen; ++i)
	{
		const int axis = chainAxisMap[i];
		if (axis < 0 || axis >= (int)numVisibleAxes)
		{
			reply.printf("Chain joint %u has no valid axis mapping", i + 1);
			error = true;
			return false;
		}

		if (!hasScrew[i])
		{
			// default axis-aligned screw for convenience
			screwOmega[i][0] = 0.0;
			screwOmega[i][1] = 0.0;
			screwOmega[i][2] = 1.0;
			screwQ[i][0] = 0.0;
			screwQ[i][1] = 0.0;
			screwQ[i][2] = 0.0;
			hasScrew[i] = true;
		}

		const float n = Norm(screwOmega[i]);
		if (n < 1.0e-6)
		{
			reply.printf("Joint '%c' has invalid zero-length screw axis", chainLetters[i]);
			error = true;
			return false;
		}
		screwOmega[i][0] /= n;
		screwOmega[i][1] /= n;
		screwOmega[i][2] /= n;
	}
	return true;
}

void Robot5AxisKinematics::SeedJointVectorFromMachine(const float machinePos[], size_t numVisibleAxes, float jointsOut[]) const noexcept
{
	const size_t chainLen = strlen(chainLetters);
	for (size_t i = 0; i < chainLen; ++i)
	{
		const int axis = chainAxisMap[i];
		if (axis >= 0 && axis < (int)numVisibleAxes)
		{
			jointsOut[i] = machinePos[axis];
		}
		else
		{
			jointsOut[i] = jointHome[i];
		}
	}
}

/**
 * @brief Build IK target pose from machine coordinates.
 * @param[in] machinePos Current machine-space coordinates.
 * @param[in] numVisibleAxes Number of visible axes.
 * @param[out] target Target pose composed from XYZ and ABC values.
 */
void Robot5AxisKinematics::BuildTargetPose(const float machinePos[], size_t numVisibleAxes, ChainTransform& target) const noexcept
{
	SetIdentity(target.m);
	float rotOnly[12];
	SetIdentity(rotOnly);

	const int xAxis = FindVisibleAxisByLetter('X', numVisibleAxes);
	const int yAxis = FindVisibleAxisByLetter('Y', numVisibleAxes);
	const int zAxis = FindVisibleAxisByLetter('Z', numVisibleAxes);
	const int aAxis = FindVisibleAxisByLetter('A', numVisibleAxes);
	const int bAxis = FindVisibleAxisByLetter('B', numVisibleAxes);
	const int cAxis = FindVisibleAxisByLetter('C', numVisibleAxes);

	const float x = (xAxis >= 0) ? machinePos[xAxis] : 0.0;
	const float y = (yAxis >= 0) ? machinePos[yAxis] : 0.0;
	const float z = (zAxis >= 0) ? machinePos[zAxis] : 0.0;
	const float a = (aAxis >= 0) ? machinePos[aAxis] : 0.0;
	const float b = (bAxis >= 0) ? machinePos[bAxis] : 0.0;
	const float c = (cAxis >= 0) ? machinePos[cAxis] : 0.0;

	RotationFromABC(a, b, c, rotOnly);
	for (size_t i = 0; i < 11; ++i)
	{
		target.m[i] = rotOnly[i];
	}
	target.m[3] = x;
	target.m[7] = y;
	target.m[11] = z;
}

/**
 * @brief Compute forward kinematics for current chain joint values.
 * @param[in] joints Joint vector in chain order.
 * @param[out] out End-effector transform including tool transform.
 */
void Robot5AxisKinematics::ForwardKinematics(const float joints[], ChainTransform& out) const noexcept
{
	SetIdentity(out.m);
	const size_t chainLen = strlen(chainLetters);
	for (size_t i = 0; i < chainLen; ++i)
	{
		float jt[12];
		const float qv = joints[i] - jointHome[i];
		if (jointTypes[i] == 'R')
		{
			ExpRotJoint(screwOmega[i], screwQ[i], qv * DegreesToRadians, jt);
		}
		else
		{
			ExpPrismaticJoint(screwOmega[i], qv, jt);
		}
		float composed[12];
		Compose(out.m, jt, composed);
		CopyTransform(composed, out.m);
	}
	float finalPose[12];
	Compose(out.m, toolTransform.m, finalPose);
	CopyTransform(finalPose, out.m);
}

/**
 * @brief Solve inverse kinematics for target machine pose.
 * @param[in] machinePos Target machine-space coordinates.
 * @param[in] numVisibleAxes Number of visible axes.
 * @param[in,out] jointsInOut Initial joint seed, replaced with solved joints on success.
 * @return true if solver converges within configured limits, otherwise false.
 */
bool Robot5AxisKinematics::SolveInverseKinematics(const float machinePos[], size_t numVisibleAxes, float jointsInOut[]) const noexcept
{
	struct IkScratch
	{
		float joints[MaxAxes];
		uint8_t activeJointIdx[MaxAxes];
		float error[PoseDof];
		float jac[PoseDof * MaxAxes];
		float normal[MaxAxes * MaxAxes];
		float rhs[MaxAxes];
		float perturbed[MaxAxes];
	};

	// Keep IK work buffers off task stacks. Kinematics solving is not re-entrant in normal use.
	static IkScratch scratch;

	ChainTransform target;
	BuildTargetPose(machinePos, numVisibleAxes, target);

	const size_t chainLen = strlen(chainLetters);
	if (chainLen == 0)
	{
		return false;
	}

	for (size_t i = 0; i < chainLen; ++i)
	{
		scratch.joints[i] = jointsInOut[i];
	}

	// Keep non-pose chain letters driven directly from axis coordinates.
	for (size_t i = 0; i < chainLen; ++i)
	{
		if (!IsPoseLetter(chainLetters[i]))
		{
			const int axis = chainAxisMap[i];
			if (axis >= 0 && axis < (int)numVisibleAxes)
			{
				scratch.joints[i] = machinePos[axis];
			}
		}
	}

	size_t activeCount = 0;
	for (size_t i = 0; i < chainLen; ++i)
	{
		if (IsPoseLetter(chainLetters[i]))
		{
			scratch.activeJointIdx[activeCount++] = (uint8_t)i;
		}
	}
	if (activeCount == 0)
	{
		activeCount = chainLen;
		for (size_t i = 0; i < chainLen; ++i)
		{
			scratch.activeJointIdx[i] = (uint8_t)i;
		}
	}

	for (uint16_t it = 0; it < ikIterationsLimit; ++it)
	{
		ChainTransform current;
		ForwardKinematics(scratch.joints, current);
		const float residual = PoseErrorVector(current.m, target.m, ikRotationWeight, scratch.error);
		lastIkResidual = residual;
		if (residual <= ikTolerance)
		{
			for (size_t i = 0; i < chainLen; ++i)
			{
				jointsInOut[i] = scratch.joints[i];
			}
			return true;
		}

		for (size_t k = 0; k < PoseDof * MaxAxes; ++k)
		{
			scratch.jac[k] = 0.0;
		}

		for (size_t a = 0; a < activeCount; ++a)
		{
			const size_t j = scratch.activeJointIdx[a];
			for (size_t i = 0; i < chainLen; ++i)
			{
				scratch.perturbed[i] = scratch.joints[i];
			}

			const float delta = (jointTypes[j] == 'R') ? 0.05f : 0.02f;
			scratch.perturbed[j] += delta;
			ChainTransform pertPose;
			ForwardKinematics(scratch.perturbed, pertPose);
			float err2[PoseDof];
			(void)PoseErrorVector(pertPose.m, target.m, ikRotationWeight, err2);
			for (size_t r = 0; r < PoseDof; ++r)
			{
				scratch.jac[r * MaxAxes + a] = (err2[r] - scratch.error[r]) / delta;
			}
		}

		for (size_t i = 0; i < activeCount; ++i)
		{
			scratch.rhs[i] = 0.0;
			for (size_t j = 0; j < activeCount; ++j)
			{
				scratch.normal[i * activeCount + j] = 0.0;
			}
		}

		for (size_t i = 0; i < activeCount; ++i)
		{
			for (size_t r = 0; r < PoseDof; ++r)
			{
				scratch.rhs[i] += scratch.jac[r * MaxAxes + i] * scratch.error[r];
			}
			for (size_t j = 0; j < activeCount; ++j)
			{
				float v = 0.0;
				for (size_t r = 0; r < PoseDof; ++r)
				{
					v += scratch.jac[r * MaxAxes + i] * scratch.jac[r * MaxAxes + j];
				}
				if (i == j)
				{
					v += fsquare(ikDamping);
				}
				scratch.normal[i * activeCount + j] = v;
			}
		}

		if (!SolveLinear(scratch.normal, scratch.rhs, activeCount))
		{
			KINDBG("K13 IK linear solve failed (active=%u, residual=%.4f)\n", (unsigned int)activeCount, (double)lastIkResidual);
			return false;
		}

		for (size_t i = 0; i < activeCount; ++i)
		{
			const size_t joint = scratch.activeJointIdx[i];
			scratch.joints[joint] += scratch.rhs[i];
			if (hasJointLimits[joint])
			{
				scratch.joints[joint] = constrain<float>(scratch.joints[joint], jointMin[joint], jointMax[joint]);
			}
			if (!IsFinite(scratch.joints[joint]))
			{
				KINDBG("K13 IK non-finite joint value at joint %u\n", (unsigned int)joint);
				return false;
			}
		}
	}

	KINDBG("K13 IK did not converge (iter=%u, residual=%.4f, tol=%.4f)\n",
		(unsigned int)ikIterationsLimit, (double)lastIkResidual, (double)ikTolerance);

	return false;
}

void Robot5AxisKinematics::ApplyMatrixMap(const float axisPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], MovementError& rslt) const noexcept
{
	for (size_t motor = 0; motor < numTotalAxes; ++motor)
	{
		const size_t axisLimit = min<size_t>(numVisibleAxes, lastAxis[motor] + 1);
		size_t axis = firstAxis[motor];
		if (axis < axisLimit)
		{
			float movement = inverseMatrix(axis, motor) * axisPos[axis];
			++axis;
			while (axis < axisLimit)
			{
				movement += inverseMatrix(axis, motor) * axisPos[axis];
				++axis;
			}
			RoundToInt32(rslt, movement * stepsPerMm[motor], motorPos[motor]);
		}
	}
}

void Robot5AxisKinematics::ApplyInverseMatrixMap(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float axisPos[]) const noexcept
{
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		float position = 0.0;
		const size_t motorLimit = min<size_t>(numTotalAxes, lastMotor[axis] + 1);
		for (size_t motor = firstMotor[axis]; motor < motorLimit; ++motor)
		{
			const float factor = forwardMatrix(motor, axis);
			if (factor != 0.0)
			{
				position += factor * (float)motorPos[motor] / stepsPerMm[motor];
			}
		}
		axisPos[axis] = position;
	}
}

void Robot5AxisKinematics::ExtractPoseToAxes(const ChainTransform& pose, float axisPos[], size_t numVisibleAxes) const noexcept
{
	int xAxis = FindVisibleAxisByLetter('X', numVisibleAxes);
	int yAxis = FindVisibleAxisByLetter('Y', numVisibleAxes);
	int zAxis = FindVisibleAxisByLetter('Z', numVisibleAxes);
	int aAxis = FindVisibleAxisByLetter('A', numVisibleAxes);
	int bAxis = FindVisibleAxisByLetter('B', numVisibleAxes);
	int cAxis = FindVisibleAxisByLetter('C', numVisibleAxes);

	if (xAxis >= 0) { axisPos[xAxis] = pose.m[3]; }
	if (yAxis >= 0) { axisPos[yAxis] = pose.m[7]; }
	if (zAxis >= 0) { axisPos[zAxis] = pose.m[11]; }

	float a, b, c;
	ABCFromRotation(pose.m, a, b, c);
	if (aAxis >= 0) { axisPos[aAxis] = a; }
	if (bAxis >= 0) { axisPos[bAxis] = b; }
	if (cAxis >= 0) { axisPos[cAxis] = c; }
}

/**
 * @brief Handle M669 configuration for K13 matrix and screw-solver parameters.
 * @param[in] mCode G-code number to process.
 * @param[in,out] gb G-code parser buffer.
 * @param[in,out] reply Output buffer for status/error text.
 * @param[in,out] error Set true when configuration fails.
 * @return true when this call consumed configuration parameters.
 */
bool Robot5AxisKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException)
{
	if (mCode != 669)
	{
		return ZLeadscrewKinematics::Configure(mCode, gb, reply, error);
	}

	bool seen = gb.Seen('K');
	const size_t numVisibleAxes = min<size_t>(reprap.GetGCodes().GetVisibleAxes(), MaxAxes);
	const size_t numTotalAxes = min<size_t>(reprap.GetGCodes().GetTotalAxes(), MaxAxes);

	if (UpdateChainMapping(numVisibleAxes, reply, error))
	{
		seen = true;
	}

	for (size_t axis = 0; !error && axis < numVisibleAxes; ++axis)
	{
		if (gb.Seen(reprap.GetGCodes().GetAxisLetters()[axis]))
		{
			float motorFactors[MaxAxes];
			size_t numMotors = numTotalAxes;
			gb.GetFloatArray(motorFactors, numMotors, false);
			for (size_t motor = 0; motor < numTotalAxes; ++motor)
			{
				if (inverseMatrix(axis, motor) != motorFactors[motor])
				{
					inverseMatrix(axis, motor) = motorFactors[motor];
					modified = true;
					seen = true;
				}
			}
			for (size_t motor = numTotalAxes; motor < MaxAxes; ++motor)
			{
				if (inverseMatrix(axis, motor) != 0.0)
				{
					inverseMatrix(axis, motor) = 0.0;
					modified = true;
					seen = true;
				}
			}
		}
	}

	String<220> paramValue;
	bool dummySeen = false;
	if (!error && gb.TryGetQuotedString('B', paramValue.GetRef(), dummySeen, true))
	{
		if (ParseChain(paramValue.c_str(), numVisibleAxes, reply, error))
		{
			seen = true;
		}
	}
	if (!error && gb.TryGetQuotedString('P', paramValue.GetRef(), dummySeen, false))
	{
		if (ParseJointTypes(paramValue.c_str(), numVisibleAxes, reply, error))
		{
			seen = true;
		}
	}
	if (!error && gb.TryGetQuotedString('C', paramValue.GetRef(), dummySeen, false))
	{
		if (ParseScrewDefinitions(paramValue.c_str(), numVisibleAxes, reply, error))
		{
			seen = true;
		}
	}
	if (!error && gb.TryGetQuotedString('L', paramValue.GetRef(), dummySeen, false))
	{
		if (ParseJointLimits(paramValue.c_str(), numVisibleAxes, reply, error))
		{
			seen = true;
		}
	}
	if (!error && gb.TryGetQuotedString('M', paramValue.GetRef(), dummySeen, false))
	{
		if (ParseToolTransform(paramValue.c_str(), reply, error))
		{
			seen = true;
		}
	}
	if (!error && gb.TryGetQuotedString('R', paramValue.GetRef(), dummySeen, true))
	{
		if (ParseContinuousAxes(paramValue.c_str(), numVisibleAxes, reply, error))
		{
			seen = true;
		}
	}
	if (!error && gb.TryGetQuotedString('A', paramValue.GetRef(), dummySeen, true))
	{
		if (ParseHomeAssignments(paramValue.c_str(), numVisibleAxes, reply, error))
		{
			seen = true;
		}
	}

	if (!error && gb.Seen('H'))
	{
		const HomingMode newMode = (gb.GetLimitedUIValue('H', 2) == 0) ? HomingMode::homeCartesianAxes : HomingMode::homeIndividualDrives;
		if (newMode != homingMode)
		{
			homingMode = newMode;
			seen = true;
		}
	}

	if (!error && gb.Seen('U'))
	{
		const bool useSolver = gb.GetLimitedUIValue('U', 2) != 0;
		if (useSolver != useScrewSolver)
		{
			useScrewSolver = useSolver;
			seen = true;
		}
	}
	if (!error)
	{
		uint32_t newIter = ikIterationsLimit;
		if (gb.TryGetLimitedUIValue('I', newIter, seen, 101))
		{
			ikIterationsLimit = (uint16_t)max<uint32_t>(newIter, 3);
		}
		gb.TryGetFValue('D', ikDamping, seen);
		gb.TryGetFValue('E', ikTolerance, seen);
		gb.TryGetFValue('W', ikRotationWeight, seen);
	}

	if (!error && TryConfigureSegmentation(gb))
	{
		seen = true;
	}

	if (!error)
	{
		(void)UpdateChainMapping(numVisibleAxes, reply, error);
	}
	if (!error && ValidateAndNormaliseScrews(numVisibleAxes, reply, error))
	{
		seen = true;
	}

	if (seen && !error)
	{
		Recalc();
		hasLastJointSolution = false;
		KINDBG("K13 configure: solver=%s chain=%s joints=%s H=%s I=%u D=%.4f E=%.4f W=%.2f\n",
			useScrewSolver ? "screw" : "matrix",
			chainDescription,
			jointTypes,
			(homingMode == HomingMode::homeCartesianAxes) ? "cartesian" : "individualDrives",
			(unsigned int)ikIterationsLimit,
			(double)ikDamping,
			(double)ikTolerance,
			(double)ikRotationWeight);
	}
	else if (!seen && !error)
	{
		Kinematics::Configure(mCode, gb, reply, error);
		reply.catf(", %smatrix, solver %s, joints %s", (modified) ? "modified " : "", (useScrewSolver) ? "screw" : "matrix", jointTypes);
		if (chainDescription[0] != 0)
		{
			reply.catf(", chain %s", chainDescription);
		}
		reply.catf(", homing %s", (homingMode == HomingMode::homeCartesianAxes) ? "cartesian" : "individualDrives");
		reply.catf(", IK I%u D%.3f E%.3f W%.1f", ikIterationsLimit, (double)ikDamping, (double)ikTolerance, (double)ikRotationWeight);
	}

	return seen;
}

/**
 * @brief Recompute matrix inverse and axis/drive coupling lookup state.
 */
void Robot5AxisKinematics::Recalc() noexcept
{
	// This matrix is large (MaxAxes x 2*MaxAxes), keep it out of the MAIN task stack.
	// Recalc is only called from configuration paths, so shared scratch storage is acceptable.
	static FixedMatrix<float, MaxAxes, 2 * MaxAxes> tempMatrix;
	for (size_t i = 0; i < MaxAxes; ++i)
	{
		for (size_t j = 0; j < MaxAxes; ++j)
		{
			tempMatrix(i, j) = inverseMatrix(i, j);
			tempMatrix(i, j + MaxAxes) = (i == j) ? 1.0 : 0.0;
		}
	}

	const bool ok = tempMatrix.GaussJordan(MaxAxes, 2 * MaxAxes);
	if (ok)
	{
		for (size_t i = 0; i < MaxAxes; ++i)
		{
			for (size_t j = 0; j < MaxAxes; ++j)
			{
				forwardMatrix(i, j) = tempMatrix(i, j + MaxAxes);
			}
		}
	}
	else
	{
		forwardMatrix.Fill(0.0);
		reprap.GetPlatform().Message(ErrorMessage, "Invalid robot kinematics matrix\n");
	}

	for (size_t i = 0; i < MaxAxes; ++i)
	{
		firstMotor[i] = firstAxis[i] = MaxAxes;
		lastMotor[i] = lastAxis[i] = 0;
		controllingDrivers[i].Clear();
	}

	for (size_t axis = 0; axis < MaxAxes; ++axis)
	{
		for (size_t motor = 0; motor < MaxAxes; ++motor)
		{
			if (inverseMatrix(axis, motor) != 0.0)
			{
				if (axis < firstAxis[motor])
				{
					firstAxis[motor] = axis;
				}
				if (axis > lastAxis[motor])
				{
					lastAxis[motor] = axis;
				}
				controllingDrivers[axis].SetBit(motor);
			}

			if (forwardMatrix(motor, axis) != 0.0)
			{
				if (motor < firstMotor[axis])
				{
					firstMotor[axis] = motor;
				}
				if (motor > lastMotor[axis])
				{
					lastMotor[axis] = motor;
				}
				controllingDrivers[axis].SetBit(motor);
			}
		}
	}
}

bool Robot5AxisKinematics::HasSharedMotor(size_t axis) const noexcept
{
	return controllingDrivers[axis] != LogicalDrivesBitmap::MakeFromBits(axis);
}

/**
 * @brief Convert machine-space coordinates into motor-step endpoints.
 * @param[in] machinePos Machine-space coordinates.
 * @param[in] stepsPerMm Steps-per-unit for each drive.
 * @param[in] numVisibleAxes Number of visible machine axes.
 * @param[in] numTotalAxes Total number of logical drives.
 * @param[out] motorPos Output motor-step endpoints.
 * @param[in] isCoordinated True when called for coordinated motion.
 * @return Conversion result and reachability status.
 */
MovementError Robot5AxisKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes,
														int32_t motorPos[], bool isCoordinated) const noexcept
{
	MovementError rslt = MovementError::ok;
	float axisPos[MaxAxes];
	memcpyf(axisPos, machinePos, MaxAxes);

	if (useScrewSolver)
	{
		float joints[MaxAxes];
		if (hasLastJointSolution)
		{
			for (size_t i = 0; i < MaxAxes; ++i)
			{
				joints[i] = lastJointSolution[i];
			}
		}
		else
		{
			SeedJointVectorFromMachine(machinePos, numVisibleAxes, joints);
		}

		const bool solved = SolveInverseKinematics(machinePos, numVisibleAxes, joints);
		++ikSolveCount;
		if (!solved)
		{
			++ikFailCount;
			const int xAxis = FindVisibleAxisByLetter('X', numVisibleAxes);
			const int yAxis = FindVisibleAxisByLetter('Y', numVisibleAxes);
			const int zAxis = FindVisibleAxisByLetter('Z', numVisibleAxes);
			const int aAxis = FindVisibleAxisByLetter('A', numVisibleAxes);
			const int bAxis = FindVisibleAxisByLetter('B', numVisibleAxes);
			const int cAxis = FindVisibleAxisByLetter('C', numVisibleAxes);
			KINDBG("K13 IK fail #%lu/%lu target X%.3f Y%.3f Z%.3f A%.3f B%.3f C%.3f residual=%.4f\n",
				(unsigned long)ikFailCount,
				(unsigned long)ikSolveCount,
				(double)((xAxis >= 0) ? machinePos[xAxis] : 0.0f),
				(double)((yAxis >= 0) ? machinePos[yAxis] : 0.0f),
				(double)((zAxis >= 0) ? machinePos[zAxis] : 0.0f),
				(double)((aAxis >= 0) ? machinePos[aAxis] : 0.0f),
				(double)((bAxis >= 0) ? machinePos[bAxis] : 0.0f),
				(double)((cAxis >= 0) ? machinePos[cAxis] : 0.0f),
				(double)lastIkResidual);
			return MovementError::unreachable_position;
		}

		for (size_t i = 0; i < MaxAxes; ++i)
		{
			lastJointSolution[i] = joints[i];
		}
		hasLastJointSolution = true;

		const size_t chainLen = strlen(chainLetters);
		for (size_t i = 0; i < chainLen; ++i)
		{
			const int axis = chainAxisMap[i];
			if (axis >= 0 && axis < (int)numVisibleAxes)
			{
				axisPos[axis] = joints[i];
			}
		}

		if (reprap.Debug(Module::Kinematics))
		{
			KINDBG("K13 IK ok #%lu residual=%.4f firstJoints %.4f %.4f %.4f %.4f %.4f %.4f\n",
				(unsigned long)ikSolveCount,
				(double)lastIkResidual,
				(double)joints[0],
				(double)joints[1],
				(double)joints[2],
				(double)joints[3],
				(double)joints[4],
				(double)joints[5]);
		}
	}

	ApplyMatrixMap(axisPos, stepsPerMm, numVisibleAxes, numTotalAxes, motorPos, rslt);
	return rslt;
}

/**
 * @brief Convert motor-step positions back into machine-space coordinates.
 * @param[in] motorPos Motor-step positions.
 * @param[in] stepsPerMm Steps-per-unit for each drive.
 * @param[in] numVisibleAxes Number of visible machine axes.
 * @param[in] numTotalAxes Total number of logical drives.
 * @param[out] machinePos Reconstructed machine-space coordinates.
 */
void Robot5AxisKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept
{
	ApplyInverseMatrixMap(motorPos, stepsPerMm, numVisibleAxes, numTotalAxes, machinePos);

	if (useScrewSolver)
	{
		float joints[MaxAxes];
		const size_t chainLen = strlen(chainLetters);
		for (size_t i = 0; i < chainLen; ++i)
		{
			const int axis = chainAxisMap[i];
			joints[i] = (axis >= 0 && axis < (int)numVisibleAxes) ? machinePos[axis] : 0.0;
		}

		ChainTransform pose;
		ForwardKinematics(joints, pose);
		ExtractPoseToAxes(pose, machinePos, numVisibleAxes);
	}
}

float Robot5AxisKinematics::GetEndstopPosition(size_t drive, bool highEnd) noexcept
{
	if (GetHomingMode() == HomingMode::homeIndividualDrives && drive < MaxAxes && hasHomePositions[drive])
	{
		return homePositions[drive];
	}
	return ZLeadscrewKinematics::GetEndstopPosition(drive, highEnd);
}

AxesBitmap Robot5AxisKinematics::AxesAssumedHomed(AxesBitmap g92Axes) const noexcept
{
	return (GetHomingMode() == HomingMode::homeCartesianAxes) ? g92Axes : AxesBitmap();
}

AxesBitmap Robot5AxisKinematics::MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept
{
	if (!disallowMovesBeforeHoming)
	{
		return AxesBitmap();
	}

	if (GetHomingMode() == HomingMode::homeCartesianAxes)
	{
		return axesMoving;
	}

	AxesBitmap requiredAxes = axesMoving;
	for (size_t axis = 0; axis < MaxAxes; ++axis)
	{
		if (axesMoving.IsBitSet(axis))
		{
			for (size_t otherAxis = 0; otherAxis < MaxAxes; ++otherAxis)
			{
				if (otherAxis != axis && controllingDrivers[axis].Intersects(controllingDrivers[otherAxis]))
				{
					requiredAxes.SetBit(otherAxis);
				}
			}
		}
	}
	return requiredAxes;
}

/**
 * @brief Apply axis limits and optional intermediate reachability checks.
 * @param[in,out] finalCoords Requested final coordinates, adjusted when clamped.
 * @param[in] initialCoords Initial move coordinates for interpolation checks.
 * @param[in] numVisibleAxes Number of visible machine axes.
 * @param[in] axesToLimit Axes to clamp against machine limits.
 * @param[in] isCoordinated True for coordinated moves.
 * @param[in] applyM208Limits True to apply configured axis limit constraints.
 * @return Limit status including intermediate-unreachable conditions.
 */
LimitPositionResult Robot5AxisKinematics::LimitPosition(float finalCoords[], const float *_ecv_array _ecv_null initialCoords,
														 size_t numVisibleAxes, AxesBitmap axesToLimit, bool isCoordinated, bool applyM208Limits) const noexcept
{
	const bool limited = (applyM208Limits && LimitPositionFromAxis(finalCoords, 0, numVisibleAxes, axesToLimit));
	if (!useScrewSolver)
	{
		return (limited) ? LimitPositionResult::adjusted : LimitPositionResult::ok;
	}

	float joints[MaxAxes];
	SeedJointVectorFromMachine(finalCoords, numVisibleAxes, joints);
	if (!SolveInverseKinematics(finalCoords, numVisibleAxes, joints))
	{
		return (limited) ? LimitPositionResult::adjustedAndIntermediateUnreachable : LimitPositionResult::intermediateUnreachable;
	}

	if (isCoordinated && initialCoords != nullptr)
	{
		float testCoords[MaxAxes];
		for (int i = 1; i <= 3; ++i)
		{
			const float t = (float)i * 0.25f;
			for (size_t axis = 0; axis < numVisibleAxes; ++axis)
			{
				testCoords[axis] = initialCoords[axis] + (finalCoords[axis] - initialCoords[axis]) * t;
			}
			SeedJointVectorFromMachine(testCoords, numVisibleAxes, joints);
			if (!SolveInverseKinematics(testCoords, numVisibleAxes, joints))
			{
				return (limited) ? LimitPositionResult::adjustedAndIntermediateUnreachable : LimitPositionResult::intermediateUnreachable;
			}
		}
	}

	return (limited) ? LimitPositionResult::adjusted : LimitPositionResult::ok;
}

/**
 * @brief Enforce speed/acceleration limits for coupled kinematics mappings.
 * @param[in,out] dda Move currently being planned.
 * @param[in] normalisedDirectionVector Unit move direction in machine-space.
 * @param[in] numVisibleAxes Number of visible machine axes.
 * @param[in] continuousRotationShortcut Unused shortcut flag from base interface.
 */
void Robot5AxisKinematics::LimitSpeedAndAcceleration(DDA& dda, const float *_ecv_array normalisedDirectionVector, size_t numVisibleAxes, bool continuousRotationShortcut) const noexcept
{
	float motorMovements[MaxAxes];
	for (float& mm : motorMovements)
	{
		mm = 0.0;
	}

	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		if (HasSharedMotor(axis))
		{
			const float dv = normalisedDirectionVector[axis];
			if (dv != 0.0)
			{
				for (size_t motor = 0; motor < MaxAxes; ++motor)
				{
					const float factor = inverseMatrix(axis, motor);
					if (factor != 0.0)
					{
						motorMovements[motor] += factor * dv;
					}
				}
			}
		}
	}

	for (size_t motor = 0; motor < MaxAxes; ++motor)
	{
		const float mm = fabsf(motorMovements[motor]);
		if (mm != 0.0)
		{
			dda.LimitSpeedAndAcceleration(reprap.GetMove().MaxFeedrate(motor)/mm, reprap.GetMove().NormalAcceleration(motor)/mm);
		}
	}
}

bool Robot5AxisKinematics::IsContinuousRotationAxis(size_t axis) const noexcept
{
	return (axis < MaxAxes && continuousRotation[axis]) || Kinematics::IsContinuousRotationAxis(axis);
}

LogicalDrivesBitmap Robot5AxisKinematics::GetControllingDrives(size_t axis, bool forHoming) const noexcept
{
	return (axis < MaxAxes) ? controllingDrivers[axis] : LogicalDrivesBitmap::MakeFromBits(axis);
}
