#include "../../src/Heating/FOPDT.h"

#include "../support/TestRunner.h"

TEST_CASE(FopDtRejectsInvalidParameters)
{
	FopDt model;
	String<512> reply;

	EXPECT_TRUE(!model.SetParameters(0.01f, 1.0f, 0.0f, 1.2f, 5.0f, 0.5f, 24.0f, true, false, reply.GetRef()));
	EXPECT_TRUE(std::string(reply.c_str()).find("estimated temperature rise too small") != std::string::npos);
	EXPECT_TRUE(!model.IsEnabled());
}

TEST_CASE(FopDtAcceptsValidParametersAndComputesHeatingModel)
{
	FopDt model;
	String<512> reply;

	EXPECT_TRUE(model.SetParameters(2.4f, 0.4f, 0.2f, 1.2f, 4.0f, 0.8f, 24.0f, true, false, reply.GetRef()));
	EXPECT_TRUE(model.IsEnabled());
	EXPECT_NEAR(model.GetMaxPwm(), 0.8, 1e-6);
	EXPECT_NEAR(model.EstimateRequiredPwm(100.0f, 0.5f), model.GetNetHeatingRate(100.0f, 0.5f, 0.0f) / -model.GetHeatingRate(), 1e-5);
	EXPECT_NEAR(model.GetNetHeatingRate(60.0f, 0.0f, 0.5f), model.GetHeatingRate() * 0.5f - model.EstimateRequiredPwm(60.0f, 0.0f) * model.GetHeatingRate(), 1e-5);
	EXPECT_TRUE(reply.IsEmpty());
}

TEST_CASE(FopDtCorrectsPwmForVoltageWithinLimits)
{
	FopDt model;
	String<512> reply;
	EXPECT_TRUE(model.SetParameters(2.0f, 0.5f, 0.0f, 1.35f, 5.0f, 0.9f, 24.0f, true, false, reply.GetRef()));

	const float corrected = model.CorrectPwmForVoltage(0.4f, 20.0f);
	EXPECT_NEAR(corrected, 0.4 * (24.0 / 20.0) * (24.0 / 20.0), 1e-5);
	EXPECT_EQ(model.CorrectPwmForVoltage(1.0f, 20.0f), 0.9f);
}

TEST_CASE(FopDtCalculatesPidParametersAndOverridesThem)
{
	FopDt model;
	String<512> reply;
	EXPECT_TRUE(model.SetParameters(2.4f, 0.4f, 0.2f, 1.2f, 4.0f, 0.8f, 24.0f, true, false, reply.GetRef()));

	model.CalcPidConstants(220.0f);
	const PidParameters loadParams = model.GetPidParameters(true);
	const PidParameters setpointParams = model.GetPidParameters(false);
	EXPECT_TRUE(loadParams.kP > 0.0f);
	EXPECT_TRUE(loadParams.recipTi > 0.0f);
	EXPECT_TRUE(loadParams.tD > 0.0f);
	EXPECT_TRUE(setpointParams.kP > 0.0f);

	const M301PidParameters overrideParams{255.0f, 10.0f, 50.0f};
	model.SetM301PidParameters(overrideParams);
	EXPECT_TRUE(model.ArePidParametersOverridden());
	const M301PidParameters roundTripped = model.GetM301PidParameters(false);
	EXPECT_NEAR(roundTripped.kP, overrideParams.kP, 1e-4);
	EXPECT_NEAR(roundTripped.kI, overrideParams.kI, 1e-4);
	EXPECT_NEAR(roundTripped.kD, overrideParams.kD, 1e-4);
}

TEST_CASE(FopDtAppendsModelCommands)
{
	FopDt model;
	String<512> reply;
	EXPECT_TRUE(model.SetParameters(2.4f, 0.4f, 0.2f, 1.2f, 4.0f, 0.8f, 24.0f, true, true, reply.GetRef()));
	EXPECT_TRUE(model.IsInverted());

	String<512> command;
	model.AppendM307Command(1, command.GetRef(), true);
	EXPECT_TRUE(std::string(command.c_str()).find("M307 H1") != std::string::npos);
	EXPECT_TRUE(std::string(command.c_str()).find("V24.0") != std::string::npos);

	model.SetM301PidParameters({200.0f, 5.0f, 25.0f});
	String<512> pidCommand;
	model.AppendM301Command(1, pidCommand.GetRef());
	EXPECT_TRUE(std::string(pidCommand.c_str()).find("M301 H1") != std::string::npos);
}