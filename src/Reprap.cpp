#include "RepRapFirmware.h"

// RepRap member functions.

// Do nothing more in the constructor; put what you want in RepRap:Init()

RepRap::RepRap() : toolList(nullptr), currentTool(nullptr), lastToolWarningTime(0.0), activeExtruders(0),
	activeToolHeaters(0), ticksInSpinState(0), spinningModule(noModule), debug(0), stopped(false),
	active(false), resetting(false), processingConfig(true), beepFrequency(0), beepDuration(0)
{
	OutputBuffer::Init();
	platform = new Platform();
	network = new Network(platform);
	webserver = new Webserver(platform, network);
	gCodes = new GCodes(platform, webserver);
	move = new Move(platform, gCodes);
	heat = new Heat(platform);

#if SUPPORT_ROLAND
	roland = new Roland(platform);
#endif

	printMonitor = new PrintMonitor(platform, gCodes);

	SetPassword(DEFAULT_PASSWORD);
	SetName(DEFAULT_NAME);
	message[0] = 0;
}

void RepRap::Init()
{
	// All of the following init functions must execute reasonably quickly before the watchdog times us out
	platform->Init();
	gCodes->Init();
	network->Init();
	webserver->Init();
	move->Init();
	heat->Init();
#if SUPPORT_ROLAND
	roland->Init();
#endif
	printMonitor->Init();
	Platform::EnableWatchdog();		// do this after all init calls are made
	active = true;					// must do this before we start the network, else the watchdog may time out

	platform->MessageF(HOST_MESSAGE, "%s Version %s dated %s\n", NAME, VERSION, DATE);

	// Run the configuration file
	const char *configFile = platform->GetConfigFile();
	platform->Message(HOST_MESSAGE, "\nExecuting ");
	if (platform->GetMassStorage()->FileExists(platform->GetSysDir(), configFile))
	{
		platform->MessageF(HOST_MESSAGE, "%s...", platform->GetConfigFile());
	}
	else
	{
		platform->MessageF(HOST_MESSAGE, "%s (no configuration file found)...", platform->GetDefaultFile());
		configFile = platform->GetDefaultFile();
	}

	if (gCodes->DoFileMacro(configFile, false))
	{
		while (gCodes->DoingFileMacro())
		{
			// GCodes::Spin will read the macro and ensure DoFileMacro returns true when it's done
			Spin();
		}
		platform->Message(HOST_MESSAGE, "Done!\n");
	}
	else
	{
		platform->Message(HOST_MESSAGE, "Error, not found\n");
	}
	processingConfig = false;

	// Enable network (unless it's disabled)
#ifdef DUET_NG
	network->Activate();			// Need to do this here, as the configuration GCodes may set IP address etc.
	if (network->IsEnabled())
	{
		platform->Message(HOST_MESSAGE, "Starting network...\n");
	}
	else
	{
		platform->Message(HOST_MESSAGE, "Network disabled.\n");
	}
#else
	if (network->IsEnabled())
	{
		// Need to do this here, as the configuration GCodes may set IP address etc.
		platform->Message(HOST_MESSAGE, "Starting network...\n");
		network->Enable();
	}
	else
	{
		platform->Message(HOST_MESSAGE, "Network disabled.\n");
	}
#endif

	platform->MessageF(HOST_MESSAGE, "%s is up and running.\n", NAME);
	fastLoop = FLT_MAX;
	slowLoop = 0.0;
	lastTime = platform->Time();
}

void RepRap::Exit()
{
	active = false;
	heat->Exit();
	move->Exit();
	gCodes->Exit();
	webserver->Exit();
	network->Exit();
	platform->Message(GENERIC_MESSAGE, "RepRap class exited.\n");
	platform->Exit();
}

void RepRap::Spin()
{
	if(!active)
		return;

	spinningModule = modulePlatform;
	ticksInSpinState = 0;
	platform->Spin();

	spinningModule = moduleNetwork;
	ticksInSpinState = 0;
	network->Spin();

	spinningModule = moduleWebserver;
	ticksInSpinState = 0;
	webserver->Spin();

	spinningModule = moduleGcodes;
	ticksInSpinState = 0;
	gCodes->Spin();

	spinningModule = moduleMove;
	ticksInSpinState = 0;
	move->Spin();

	spinningModule = moduleHeat;
	ticksInSpinState = 0;
	heat->Spin();

#if SUPPORT_ROLAND
	spinningModule = moduleRoland;
	ticksInSpinState = 0;
	roland->Spin();
#endif

	spinningModule = modulePrintMonitor;
	ticksInSpinState = 0;
	printMonitor->Spin();

	spinningModule = noModule;
	ticksInSpinState = 0;

	// Check if we need to display a cold extrusion warning

	for (Tool *t = toolList; t != nullptr; t = t->Next())
	{
		if (t->DisplayColdExtrudeWarning() && ToolWarningsAllowed())
		{
			platform->MessageF(GENERIC_MESSAGE, "Warning: Tool %d was not driven because its heater temperatures were not high enough or it has a heater fault\n", t->myNumber);
		}
	}

	// Keep track of the loop time

	float t = platform->Time();
	float dt = t - lastTime;
	if(dt < fastLoop)
	{
		fastLoop = dt;
	}
	if(dt > slowLoop)
	{
		slowLoop = dt;
	}
	lastTime = t;
}

void RepRap::Timing()
{
	platform->MessageF(GENERIC_MESSAGE, "Slowest main loop (seconds): %f; fastest: %f\n", slowLoop, fastLoop);
	fastLoop = FLT_MAX;
	slowLoop = 0.0;
}

void RepRap::Diagnostics()
{
	platform->Message(GENERIC_MESSAGE, "Diagnostics\n");
	OutputBuffer::Diagnostics();
	platform->Diagnostics();				// this includes a call to our Timing() function
	move->Diagnostics();
	heat->Diagnostics();
	gCodes->Diagnostics();
	network->Diagnostics();
	webserver->Diagnostics();
}

// Turn off the heaters, disable the motors, and
// deactivate the Heat and Move classes.  Leave everything else
// working.

void RepRap::EmergencyStop()
{
	stopped = true;

	// Do not turn off ATX power here. If the nozzles are still hot, don't risk melting any surrounding parts...
	//platform->SetAtxPower(false);

	Tool* tool = toolList;
	while (tool != nullptr)
	{
		tool->Standby();
		tool = tool->Next();
	}

	heat->Exit();
	for(size_t heater = 0; heater < HEATERS; heater++)
	{
		platform->SetHeater(heater, 0.0);
	}

	// We do this twice, to avoid an interrupt switching a drive back on. move->Exit() should prevent interrupts doing this.
	for(int i = 0; i < 2; i++)
	{
		move->Exit();
		for(size_t drive = 0; drive < DRIVES; drive++)
		{
			platform->SetMotorCurrent(drive, 0.0);
			platform->DisableDrive(drive);
		}
	}
}

void RepRap::SetDebug(Module m, bool enable)
{
	if (enable)
	{
		debug |= (1 << m);
	}
	else
	{
		debug &= ~(1 << m);
	}
	PrintDebug();
}

void RepRap::SetDebug(bool enable)
{
	debug = (enable) ? 0xFFFF : 0;
}

void RepRap::PrintDebug()
{
	if (debug != 0)
	{
		platform->Message(GENERIC_MESSAGE, "Debugging enabled for modules:");
		for (size_t i = 0; i < numModules; i++)
		{
			if ((debug & (1 << i)) != 0)
			{
				platform->MessageF(GENERIC_MESSAGE, " %s(%u)", moduleName[i], i);
			}
		}
		platform->Message(GENERIC_MESSAGE, "\nDebugging disabled for modules:");
		for (size_t i = 0; i < numModules; i++)
		{
			if ((debug & (1 << i)) == 0)
			{
				platform->MessageF(GENERIC_MESSAGE, " %s(%u)", moduleName[i], i);
			}
		}
		platform->Message(GENERIC_MESSAGE, "\n");
	}
	else
	{
		platform->Message(GENERIC_MESSAGE, "Debugging disabled\n");
	}
}

// Add a tool.
// Prior to calling this, delete any existing tool with the same number
void RepRap::AddTool(Tool* tool)
{
	Tool** t = &toolList;
	while(*t != nullptr && (*t)->Number() < tool->Number())
	{
		t = &((*t)->next);
	}
	tool->next = *t;
	*t = tool;
	tool->UpdateExtruderAndHeaterCount(activeExtruders, activeToolHeaters);
	platform->UpdateConfiguredHeaters();
}

void RepRap::DeleteTool(Tool* tool)
{
	// Must have a valid tool...
	if (tool == nullptr)
	{
		return;
	}

	// Deselect it if necessary
	if (GetCurrentTool() == tool)
	{
		SelectTool(-1);
	}

	// Switch off any associated heater
	for (size_t i = 0; i < tool->HeaterCount(); i++)
	{
		reprap.GetHeat()->SwitchOff(tool->Heater(i));
	}

	// Purge any references to this tool
	for (Tool **t = &toolList; *t != nullptr; t = &((*t)->next))
	{
		if (*t == tool)
		{
			*t = tool->next;
			break;
		}
	}

	// Delete it
	Tool::Delete(tool);

	// Update the number of active heaters and extruder drives
	activeExtruders = activeToolHeaters = 0;
	for (Tool *t = toolList; t != nullptr; t = t->Next())
	{
		t->UpdateExtruderAndHeaterCount(activeExtruders, activeToolHeaters);
	}
	platform->UpdateConfiguredHeaters();
}

void RepRap::SelectTool(int toolNumber)
{
	Tool* tool = toolList;

	while(tool != nullptr)
	{
		if (tool->Number() == toolNumber)
		{
			tool->Activate(currentTool);
			currentTool = tool;
			return;
		}
		tool = tool->Next();
	}

	// Selecting a non-existent tool is valid.  It sets them all to standby.
	if (currentTool != nullptr)
	{
		StandbyTool(currentTool->Number());
	}
	currentTool = nullptr;
}

void RepRap::PrintTool(int toolNumber, StringRef& reply) const
{
	Tool* tool = GetTool(toolNumber);
	if (tool != nullptr)
	{
		tool->Print(reply);
	}
	else
	{
		reply.copy("Error: Attempt to print details of non-existent tool.\n");
	}
}

void RepRap::StandbyTool(int toolNumber)
{
	Tool* tool = GetTool(toolNumber);
	if (tool != nullptr)
	{
		tool->Standby();
		if (currentTool == tool)
		{
			currentTool = nullptr;
		}
	}
	else
	{
		platform->MessageF(GENERIC_MESSAGE, "Error: Attempt to standby a non-existent tool: %d.\n", toolNumber);
	}
}

Tool* RepRap::GetTool(int toolNumber) const
{
	Tool* tool = toolList;
	while(tool != nullptr)
	{
		if (tool->Number() == toolNumber)
		{
			return tool;
		}
		tool = tool->Next();
	}
	return nullptr; // Not an error
}

Tool* RepRap::GetOnlyTool() const
{
	return (toolList != nullptr && toolList->Next() == nullptr) ? toolList : nullptr;
}

void RepRap::SetToolVariables(int toolNumber, const float* standbyTemperatures, const float* activeTemperatures)
{
	Tool* tool = GetTool(toolNumber);
	if (tool != nullptr)
	{
		tool->SetVariables(standbyTemperatures, activeTemperatures);
	}
	else
	{
		platform->MessageF(GENERIC_MESSAGE, "Error: Attempt to set variables for a non-existent tool: %d.\n", toolNumber);
	}
}

// chrishamm 02-10-2015: I don't think it's a good idea to write tool warning message after every
// short move, so only print them in a reasonable interval.
bool RepRap::ToolWarningsAllowed()
{
	const float now = platform->Time();
	if (now - lastToolWarningTime > MINIMUM_TOOL_WARNING_INTERVAL)
	{
		lastToolWarningTime = platform->Time();
		return true;
	}
	return false;
}

bool RepRap::IsHeaterAssignedToTool(int8_t heater) const
{
	for(Tool *tool = toolList; tool != nullptr; tool = tool->Next())
	{
		for(size_t i = 0; i < tool->HeaterCount(); i++)
		{
			if (tool->Heater(i) == heater)
			{
				// It's already in use by some tool
				return true;
			}
		}
	}

	return false;
}

void RepRap::Tick()
{
	if (active)
	{
		Platform::KickWatchdog();
		if (!resetting)
		{
			platform->Tick();
			++ticksInSpinState;
			if (ticksInSpinState >= 20000)	// if we stall for 20 seconds, save diagnostic data and reset
			{
				resetting = true;
				for(size_t i = 0; i < HEATERS; i++)
				{
					platform->SetHeater(i, 0.0);
				}
				for(size_t i = 0; i < DRIVES; i++)
				{
					platform->DisableDrive(i);
					// We can't set motor currents to 0 here because that requires interrupts to be working, and we are in an ISR
				}

				platform->SoftwareReset((uint16_t)SoftwareResetReason::stuckInSpin);
			}
		}
	}
}

// Get the JSON status response for the web server (or later for the M105 command).
// Type 1 is the ordinary JSON status response.
// Type 2 is the same except that static parameters are also included.
// Type 3 is the same but instead of static parameters we report print estimation values.
OutputBuffer *RepRap::GetStatusResponse(uint8_t type, ResponseSource source)
{
	// Need something to write to...
	OutputBuffer *response;
	if (!OutputBuffer::Allocate(response))
	{
		// Should never happen
		return nullptr;
	}

	// Machine status
	char ch = GetStatusCharacter();
	response->printf("{\"status\":\"%c\",\"coords\":{", ch);

	/* Coordinates */
	{
		float liveCoordinates[DRIVES + 1];
#if SUPPORT_ROLAND
		if (roland->Active())
		{
			roland->GetCurrentRolandPosition(liveCoordinates);
		}
		else
#endif
		{
			move->LiveCoordinates(liveCoordinates);
		}

		if (currentTool != nullptr)
		{
			const float *offset = currentTool->GetOffset();
			for (size_t i = 0; i < AXES; ++i)
			{
				liveCoordinates[i] += offset[i];
			}
		}

		// Homed axes
		response->catf("\"axesHomed\":[%d,%d,%d]",
				(gCodes->GetAxisIsHomed(0)) ? 1 : 0,
				(gCodes->GetAxisIsHomed(1)) ? 1 : 0,
				(gCodes->GetAxisIsHomed(2)) ? 1 : 0);

		// Actual and theoretical extruder positions since power up, last G92 or last M23
		response->catf(",\"extr\":");		// announce actual extruder positions
		ch = '[';
		for (size_t extruder = 0; extruder < GetExtrudersInUse(); extruder++)
		{
			response->catf("%c%.1f", ch, liveCoordinates[AXES + extruder]);
			ch = ',';
		}
		if (ch == '[')
		{
			response->cat("[");
		}

		// XYZ positions
		response->cat("],\"xyz\":");
		if (!gCodes->AllAxesAreHomed() && move->IsDeltaMode())
		{
			// If in Delta mode, skip these coordinates if some axes are not homed
			response->cat("[0.00,0.00,0.00");
		}
		else
		{
			// On Cartesian printers, the live coordinates are (usually) valid
			ch = '[';
			for (size_t axis = 0; axis < AXES; axis++)
			{
				response->catf("%c%.2f", ch, liveCoordinates[axis]);
				ch = ',';
			}
		}
	}

	// Current tool number
	int toolNumber = (currentTool == nullptr) ? -1 : currentTool->Number();
	response->catf("]},\"currentTool\":%d", toolNumber);

	/* Output - only reported once */
	{
		bool sendBeep = (beepDuration != 0 && beepFrequency != 0);
		bool sendMessage = (message[0] != 0);
		bool sourceRight = (gCodes->HaveAux() && source == ResponseSource::AUX) || (!gCodes->HaveAux() && source == ResponseSource::HTTP);
		if ((sendBeep || message[0] != 0) && sourceRight)
		{
			response->cat(",\"output\":{");

			// Report beep values
			if (sendBeep)
			{
				response->catf("\"beepDuration\":%d,\"beepFrequency\":%d", beepDuration, beepFrequency);
				if (sendMessage)
				{
					response->cat(",");
				}

				beepFrequency = beepDuration = 0;
			}

			// Report message
			if (sendMessage)
			{
				response->cat("\"message\":");
				response->EncodeString(message, ARRAY_SIZE(message), false);
				message[0] = 0;
			}
			response->cat("}");
		}
	}

	/* Parameters */
	{
		// ATX power
		response->catf(",\"params\":{\"atxPower\":%d", platform->AtxPower() ? 1 : 0);

		// Cooling fan value
		response->cat(",\"fanPercent\":[");
		for(size_t i = 0; i < NUM_FANS; i++)
		{
			if (i == NUM_FANS - 1)
			{
				response->catf("%.2f", platform->GetFanValue(i) * 100.0);
			}
			else
			{
				response->catf("%.2f,", platform->GetFanValue(i) * 100.0);
			}
		}

		// Speed and Extrusion factors
		response->catf("],\"speedFactor\":%.2f,\"extrFactors\":", gCodes->GetSpeedFactor() * 100.0);
		ch = '[';
		for (size_t extruder = 0; extruder < GetExtrudersInUse(); extruder++)
		{
			response->catf("%c%.2f", ch, gCodes->GetExtrusionFactor(extruder) * 100.0);
			ch = ',';
		}
		response->cat((ch == '[') ? "[]}" : "]}");
	}

	// G-code reply sequence for webserver (seqence number for AUX is handled later)
	if (source == ResponseSource::HTTP)
	{
		response->catf(",\"seq\":%d", webserver->GetReplySeq());

		// There currently appears to be no need for this one, so skip it
		//response->catf(",\"buff\":%u", webserver->GetGCodeBufferSpace(WebSource::HTTP));
	}

	/* Sensors */
	{
		response->cat(",\"sensors\":{");

		// Probe
		int v0 = platform->ZProbe();
		int v1, v2;
		switch (platform->GetZProbeSecondaryValues(v1, v2))
		{
			case 1:
				response->catf("\"probeValue\":%d,\"probeSecondary\":[%d]", v0, v1);
				break;
			case 2:
				response->catf("\"probeValue\":%d,\"probeSecondary\":[%d,%d]", v0, v1, v2);
				break;
			default:
				response->catf("\"probeValue\":%d", v0);
				break;
		}

		// Fan RPM
		response->catf(",\"fanRPM\":%d}", static_cast<unsigned int>(platform->GetFanRPM()));
	}

	/* Temperatures */
	{
		response->cat(",\"temps\":{");

		/* Bed */
		const int8_t bedHeater = heat->GetBedHeater();
		if (bedHeater != -1)
		{
			response->catf("\"bed\":{\"current\":%.1f,\"active\":%.1f,\"state\":%d},",
					heat->GetTemperature(bedHeater), heat->GetActiveTemperature(bedHeater),
					heat->GetStatus(bedHeater));
		}

		/* Chamber */
		const int8_t chamberHeater = heat->GetChamberHeater();
		if (chamberHeater != -1)
		{
			response->catf("\"chamber\":{\"current\":%.1f,", heat->GetTemperature(chamberHeater));
			response->catf("\"active\":%.1f,", heat->GetActiveTemperature(chamberHeater));
			response->catf("\"state\":%d},", static_cast<int>(heat->GetStatus(chamberHeater)));
		}

		/* Heads */
		{
			response->cat("\"heads\":{\"current\":");

			// Current temperatures
			ch = '[';
			for (size_t heater = E0_HEATER; heater < GetToolHeatersInUse(); heater++)
			{
				response->catf("%c%.1f", ch, heat->GetTemperature(heater));
				ch = ',';
			}
			response->cat((ch == '[') ? "[]" : "]");

			// Active temperatures
			response->catf(",\"active\":");
			ch = '[';
			for (size_t heater = E0_HEATER; heater < GetToolHeatersInUse(); heater++)
			{
				response->catf("%c%.1f", ch, heat->GetActiveTemperature(heater));
				ch = ',';
			}
			response->cat((ch == '[') ? "[]" : "]");

			// Standby temperatures
			response->catf(",\"standby\":");
			ch = '[';
			for (size_t heater = E0_HEATER; heater < GetToolHeatersInUse(); heater++)
			{
				response->catf("%c%.1f", ch, heat->GetStandbyTemperature(heater));
				ch = ',';
			}
			response->cat((ch == '[') ? "[]" : "]");

			// Heater statuses (0=off, 1=standby, 2=active, 3=fault)
			response->cat(",\"state\":");
			ch = '[';
			for (size_t heater = E0_HEATER; heater < GetToolHeatersInUse(); heater++)
			{
				response->catf("%c%d", ch, static_cast<int>(heat->GetStatus(heater)));
				ch = ',';
			}
			response->cat((ch == '[') ? "[]" : "]");
		}
		response->cat("}}");
	}

	// Time since last reset
	response->catf(",\"time\":%.1f", platform->Time());

	/* Extended Status Response */
	if (type == 2)
	{
		// Cold Extrude/Retract
		response->catf(",\"coldExtrudeTemp\":%1.f", heat->ColdExtrude() ? 0 : HOT_ENOUGH_TO_EXTRUDE);
		response->catf(",\"coldRetractTemp\":%1.f", heat->ColdExtrude() ? 0 : HOT_ENOUGH_TO_RETRACT);

		// Endstops
		uint16_t endstops = 0;
		for(size_t drive = 0; drive < DRIVES; drive++)
		{
			EndStopHit stopped = platform->Stopped(drive);
			if (stopped == EndStopHit::highHit || stopped == EndStopHit::lowHit)
			{
				endstops |= (1 << drive);
			}
		}
		response->catf(",\"endstops\":%d", endstops);

		// Delta configuration
		response->catf(",\"geometry\":\"%s\"", move->GetGeometryString());

		// Machine name
		response->cat(",\"name\":");
		response->EncodeString(myName, ARRAY_SIZE(myName), false);

		/* Probe */
		{
			const ZProbeParameters probeParams = platform->GetZProbeParameters();

			// Trigger threshold
			response->catf(",\"probe\":{\"threshold\":%d", probeParams.adcValue);

			// Trigger height
			response->catf(",\"height\":%.2f", probeParams.height);

			// Type
			response->catf(",\"type\":%d}", platform->GetZProbeType());
		}

		/* Tool Mapping */
		{
			response->cat(",\"tools\":[");
			for(Tool *tool = toolList; tool != nullptr; tool = tool->Next())
			{
				// Heaters
				response->catf("{\"number\":%d,\"heaters\":[", tool->Number());
				for(size_t heater=0; heater<tool->HeaterCount(); heater++)
				{
					response->catf("%d", tool->Heater(heater));
					if (heater + 1 < tool->HeaterCount())
					{
						response->cat(",");
					}
				}

				// Extruder drives
				response->cat("],\"drives\":[");
				for(size_t drive=0; drive<tool->DriveCount(); drive++)
				{
					response->catf("%d", tool->Drive(drive));
					if (drive + 1 < tool->DriveCount())
					{
						response->cat(",");
					}
				}

				// Do we have any more tools?
				if (tool->Next() != nullptr)
				{
					response->cat("]},");
				}
				else
				{
					response->cat("]}");
				}
			}
			response->cat("]");
		}
	}
	else if (type == 3)
	{
		// Current Layer
		response->catf(",\"currentLayer\":%d", printMonitor->GetCurrentLayer());

		// Current Layer Time
		response->catf(",\"currentLayerTime\":%.1f", printMonitor->GetCurrentLayerTime());

		// Raw Extruder Positions
		response->cat(",\"extrRaw\":");
		ch = '[';
		for (size_t extruder = 0; extruder < GetExtrudersInUse(); extruder++)		// loop through extruders
		{
			response->catf("%c%.1f", ch, gCodes->GetRawExtruderTotalByDrive(extruder));
			ch = ',';
		}
		if (ch == '[')
		{
			response->cat("]");
		}

		// Fraction of file printed
		response->catf("],\"fractionPrinted\":%.1f", (printMonitor->IsPrinting()) ? (gCodes->FractionOfFilePrinted() * 100.0) : 0.0);

		// First Layer Duration
		response->catf(",\"firstLayerDuration\":%.1f", printMonitor->GetFirstLayerDuration());

		// First Layer Height
		// NB: This shouldn't be needed any more, but leave it here for the case that the file-based first-layer detection fails
		response->catf(",\"firstLayerHeight\":%.2f", printMonitor->GetFirstLayerHeight());

		// Print Duration
		response->catf(",\"printDuration\":%.1f", printMonitor->GetPrintDuration());

		// Warm-Up Time
		response->catf(",\"warmUpDuration\":%.1f", printMonitor->GetWarmUpDuration());

		/* Print Time Estimations */
		{
			// Based on file progress
			response->catf(",\"timesLeft\":{\"file\":%.1f", printMonitor->EstimateTimeLeft(fileBased));

			// Based on filament usage
			response->catf(",\"filament\":%.1f", printMonitor->EstimateTimeLeft(filamentBased));

			// Based on layers
			response->catf(",\"layer\":%.1f}", printMonitor->EstimateTimeLeft(layerBased));
		}
	}

	if (source == ResponseSource::AUX)
	{
		OutputBuffer *response = gCodes->GetAuxGCodeReply();
		if (response != nullptr)
		{
			// Send the response to the last command. Do this last
			response->catf(",\"seq\":%u,\"resp\":", gCodes->GetAuxSeq());			// send the response sequence number

			// Send the JSON response
			response->EncodeReply(response, true);									// also releases the OutputBuffer chain
		}
	}
	response->cat("}");

	return response;
}

OutputBuffer *RepRap::GetConfigResponse()
{
	// We need some resources to return a valid config response...
	OutputBuffer *response;
	if (!OutputBuffer::Allocate(response))
	{
		return nullptr;
	}

	// Axis minima
	response->copy("{\"axisMins\":");
	char ch = '[';
	for (size_t axis = 0; axis < AXES; axis++)
	{
		response->catf("%c%.2f", ch, platform->AxisMinimum(axis));
		ch = ',';
	}

	// Axis maxima
	response->cat("],\"axisMaxes\":");
	ch = '[';
	for (size_t axis = 0; axis < AXES; axis++)
	{
		response->catf("%c%.2f", ch, platform->AxisMaximum(axis));
		ch = ',';
	}

	// Accelerations
	response->cat("],\"accelerations\":");
	ch = '[';
	for (size_t drive = 0; drive < DRIVES; drive++)
	{
		response->catf("%c%.2f", ch, platform->Acceleration(drive));
		ch = ',';
	}

	// Motor currents
	response->cat("],\"currents\":");
	ch = '[';
	for (size_t drive = 0; drive < DRIVES; drive++)
	{
		response->catf("%c%.2f", ch, platform->MotorCurrent(drive));
		ch = ',';
	}

	// Firmware details
	response->catf("],\"firmwareElectronics\":\"%s\"", platform->GetElectronicsString());
	response->catf(",\"firmwareName\":\"%s\"", NAME);
	response->catf(",\"firmwareVersion\":\"%s\"", VERSION);
	response->catf(",\"firmwareDate\":\"%s\"", DATE);

	// Motor idle parameters
	response->catf(",\"idleCurrentFactor\":%.1f", platform->GetIdleCurrentFactor() * 100.0);
	response->catf(",\"idleTimeout\":%.1f", move->IdleTimeout());

	// Minimum feedrates
	response->cat(",\"minFeedrates\":");
	ch = '[';
	for (size_t drive = 0; drive < DRIVES; drive++)
	{
		response->catf("%c%.2f", ch, platform->ConfiguredInstantDv(drive));
		ch = ',';
	}

	// Maximum feedrates
	response->cat("],\"maxFeedrates\":");
	ch = '[';
	for (size_t drive = 0; drive < DRIVES; drive++)
	{
		response->catf("%c%.2f", ch, platform->MaxFeedrate(drive));
		ch = ',';
	}

	// Config file is no longer included, because we can use rr_configfile or M503 instead
	response->cat("]}");

	return response;
}

// Get the JSON status response for PanelDue or the old web server.
// Type 0 was the old-style webserver status response, but is no longer supported.
// Type 1 is the new-style webserver status response.
// Type 2 is the M105 S2 response, which is like the new-style status response but some fields are omitted.
// Type 3 is the M105 S3 response, which is like the M105 S2 response except that static values are also included.
// 'seq' is the response sequence number, if it is not -1 and we have a different sequence number then we send the gcode response
OutputBuffer *RepRap::GetLegacyStatusResponse(uint8_t type, int seq)
{
	// Need something to write to...
	OutputBuffer *response;
	if (!OutputBuffer::Allocate(response))
	{
		// Should never happen
		return nullptr;
	}

	// Send the status. Note that 'S' has always meant that the machine is halted in this version of the status response, so we use A for pAused.
	char ch = GetStatusCharacter();
	if (ch == 'S')			// if paused then send 'A'
	{
		ch = 'A';
	}
	else if (ch == 'H')		// if halted then send 'S'
	{
		ch = 'S';
	}
	response->printf("{\"status\":\"%c\",\"heaters\":", ch);

	// Send the heater actual temperatures
	const int8_t bedHeater = heat->GetBedHeater();
	if (bedHeater != -1)
	{
		ch = ',';
		response->catf("[%.1f", heat->GetTemperature(bedHeater));
	}
	else
	{
		ch = '[';
	}
	for (size_t heater = E0_HEATER; heater < GetToolHeatersInUse(); heater++)
	{
		response->catf("%c%.1f", ch, heat->GetTemperature(heater));
		ch = ',';
	}
	response->cat((ch == '[') ? "[]" : "]");

	// Send the heater active temperatures
	response->catf(",\"active\":");
	if (heat->GetBedHeater() != -1)
	{
		ch = ',';
		response->catf("[%.1f", heat->GetActiveTemperature(heat->GetBedHeater()));
	}
	else
	{
		ch = '[';
	}
	for (size_t heater = E0_HEATER; heater < GetToolHeatersInUse(); heater++)
	{
		response->catf("%c%.1f", ch, heat->GetActiveTemperature(heater));
		ch = ',';
	}
	response->cat((ch == '[') ? "[]" : "]");

	// Send the heater standby temperatures
	response->catf(",\"standby\":");
	if (bedHeater != -1)
	{
		ch = ',';
		response->catf("[%.1f", heat->GetStandbyTemperature(bedHeater));
	}
	else
	{
		ch = '[';
	}
	for (size_t heater = E0_HEATER; heater < GetToolHeatersInUse(); heater++)
	{
		response->catf("%c%.1f", ch, heat->GetStandbyTemperature(heater));
		ch = ',';
	}
	response->cat((ch == '[') ? "[]" : "]");

	// Send the heater statuses (0=off, 1=standby, 2=active)
	response->cat(",\"hstat\":");
	if (bedHeater != -1)
	{
		ch = ',';
		response->catf("[%d", static_cast<int>(heat->GetStatus(bedHeater)));
	}
	else
	{
		ch = '[';
	}
	for (size_t heater = E0_HEATER; heater < GetToolHeatersInUse(); heater++)
	{
		response->catf("%c%d", ch, static_cast<int>(heat->GetStatus(heater)));
		ch = ',';
	}
	response->cat((ch == '[') ? "[]" : "]");

	// Send XYZ positions
	float liveCoordinates[DRIVES];
	reprap.GetMove()->LiveCoordinates(liveCoordinates);
	const Tool* currentTool = reprap.GetCurrentTool();
	if (currentTool != nullptr)
	{
		const float *offset = currentTool->GetOffset();
		for (size_t i = 0; i < AXES; ++i)
		{
			liveCoordinates[i] += offset[i];
		}
	}
	response->catf(",\"pos\":");		// announce the XYZ position
	ch = '[';
	for (size_t drive = 0; drive < AXES; drive++)
	{
		response->catf("%c%.2f", ch, liveCoordinates[drive]);
		ch = ',';
	}

	// Send extruder total extrusion since power up, last G92 or last M23
	response->cat("],\"extr\":");		// announce the extruder positions
	ch = '[';
	for (size_t drive = 0; drive < reprap.GetExtrudersInUse(); drive++)		// loop through extruders
	{
		response->catf("%c%.1f", ch, gCodes->GetRawExtruderPosition(drive));
		ch = ',';
	}
	response->cat((ch == ']') ? "[]" : "]");

	// Send the speed and extruder override factors
	response->catf(",\"sfactor\":%.2f,\"efactor\":", gCodes->GetSpeedFactor() * 100.0);
	ch = '[';
	for (size_t i = 0; i < reprap.GetExtrudersInUse(); ++i)
	{
		response->catf("%c%.2f", ch, gCodes->GetExtrusionFactor(i) * 100.0);
		ch = ',';
	}
	response->cat((ch == '[') ? "[]" : "]");

	// Send the current tool number
	int toolNumber = (currentTool == nullptr) ? 0 : currentTool->Number();
	response->catf(",\"tool\":%d", toolNumber);

	// Send the Z probe value
	int v0 = platform->ZProbe();
	int v1, v2;
	switch (platform->GetZProbeSecondaryValues(v1, v2))
	{
	case 1:
		response->catf(",\"probe\":\"%d (%d)\"", v0, v1);
		break;
	case 2:
		response->catf(",\"probe\":\"%d (%d, %d)\"", v0, v1, v2);
		break;
	default:
		response->catf(",\"probe\":\"%d\"", v0);
		break;
	}

	// Send the fan0 settings (for PanelDue firmware 1.13)
	response->catf(",\"fanPercent\":[%.02f,%.02f]", platform->GetFanValue(0) * 100.0, platform->GetFanValue(1) * 100.0);

	// Send fan RPM value
	response->catf(",\"fanRPM\":%u", static_cast<unsigned int>(platform->GetFanRPM()));

	// Send the home state. To keep the messages short, we send 1 for homed and 0 for not homed, instead of true and false.
	if (type != 0)
	{
		response->catf(",\"homed\":[%d,%d,%d]",
				(gCodes->GetAxisIsHomed(0)) ? 1 : 0,
				(gCodes->GetAxisIsHomed(1)) ? 1 : 0,
				(gCodes->GetAxisIsHomed(2)) ? 1 : 0);
	}
	else
	{
		response->catf(",\"hx\":%d,\"hy\":%d,\"hz\":%d",
				(gCodes->GetAxisIsHomed(0)) ? 1 : 0,
				(gCodes->GetAxisIsHomed(1)) ? 1 : 0,
				(gCodes->GetAxisIsHomed(2)) ? 1 : 0);
	}

	if (printMonitor->IsPrinting())
	{
		// Send the fraction printed
		response->catf(",\"fraction_printed\":%.4f", max<float>(0.0, gCodes->FractionOfFilePrinted()));
	}

	response->cat(",\"message\":");
	response->EncodeString(message, ARRAY_SIZE(message), false);

	if (type < 2)
	{
		response->catf(",\"buff\":%u", webserver->GetGCodeBufferSpace(WebSource::HTTP));	// send the amount of buffer space available for gcodes
	}
	else if (type == 2)
	{
		if (printMonitor->IsPrinting())
		{
			// Send estimated times left based on file progress, filament usage, and layers
			response->catf(",\"timesLeft\":[%.1f,%.1f,%.1f]",
					printMonitor->EstimateTimeLeft(fileBased),
					printMonitor->EstimateTimeLeft(filamentBased),
					printMonitor->EstimateTimeLeft(layerBased));
		}
	}
	else if (type == 3)
	{
		// Add the static fields. For now this is just geometry and the machine name, but other fields could be added e.g. axis lengths.
		response->catf(",\"geometry\":\"%s\",\"myName\":", move->GetGeometryString());
		response->EncodeString(myName, ARRAY_SIZE(myName), false);
	}

	int auxSeq = (int)gCodes->GetAuxSeq();
	if (type < 2 || (seq != -1 && (int)auxSeq != seq))
	{

		// Send the response to the last command. Do this last because it can be long and may need to be truncated.
		response->catf(",\"seq\":%u,\"resp\":", auxSeq);					// send the response sequence number

		// Send the JSON response
		response->EncodeReply(gCodes->GetAuxGCodeReply(), true);			// also releases the OutputBuffer chain
	}

	response->cat("}");

	return response;
}

// Copy some parameter text, stopping at the first control character or when the destination buffer is full, and removing trailing spaces
void RepRap::CopyParameterText(const char* src, char *dst, size_t length)
{
	size_t i;
	for (i = 0; i + 1 < length && src[i] >= ' '; ++i)
	{
		dst[i] = src[i];
	}
	// Remove any trailing spaces
	while (i > 0 && dst[i - 1] == ' ')
	{
		--i;
	}
	dst[i] = 0;
}

// Get the list of files in the specified directory in JSON format.
// If flagDirs is true then we prefix each directory with a * character.
OutputBuffer *RepRap::GetFilesResponse(const char *dir, bool flagsDirs)
{
	// Need something to write to...
	OutputBuffer *response;
	if (!OutputBuffer::Allocate(response))
	{
		return nullptr;
	}

	response->copy("{\"dir\":");
	response->EncodeString(dir, strlen(dir), false);
	response->cat(",\"files\":[");

	FileInfo fileInfo;
	bool firstFile = true;
	bool gotFile = platform->GetMassStorage()->FindFirst(dir, fileInfo);
	size_t bytesLeft = OutputBuffer::GetBytesLeft(response);	// don't write more bytes than we can
	char filename[FILENAME_LENGTH];
	filename[0] = '*';
	const char *fname;

	while (gotFile)
	{
		if (fileInfo.fileName[0] != '.')			// ignore Mac resource files and Linux hidden files
		{
			// Get the long filename if possible
			if (flagsDirs && fileInfo.isDirectory)
			{
				strncpy(filename + sizeof(char), fileInfo.fileName, FILENAME_LENGTH - 1);
				filename[FILENAME_LENGTH - 1] = 0;
				fname = filename;
			}
			else
			{
				fname = fileInfo.fileName;
			}

			// Make sure we can end this response properly
			if (bytesLeft < strlen(fname) * 2 + 4)
			{
				// No more space available - stop here
				break;
			}

			// Write separator and filename
			if (!firstFile)
			{
				bytesLeft -= response->cat(',');
			}
			bytesLeft -= response->EncodeString(fname, FILENAME_LENGTH, false);

			firstFile = false;
		}
		gotFile = platform->GetMassStorage()->FindNext(fileInfo);
	}
	response->cat("]}");

	return response;
}

void RepRap::Beep(int freq, int ms)
{
	if (gCodes->HaveAux())
	{
		// If there is an LCD device present, make it beep
		platform->Beep(freq, ms);
	}
	else
	{
		// Otherwise queue it until the webserver can process it
		beepFrequency = freq;
		beepDuration = ms;
	}
}

void RepRap::SetMessage(const char *msg)
{
	strncpy(message, msg, MESSAGE_LENGTH);
	message[MESSAGE_LENGTH] = 0;
}

// Get the status character for the new-style status response
char RepRap::GetStatusCharacter() const
{
	return    (processingConfig)										? 'C'	// Reading the configuration file
			: (gCodes->IsFlashing())									? 'F'	// Flashing a new firmware binary
			: (IsStopped()) 											? 'H'	// Halted
			: (gCodes->IsPausing()) 									? 'D'	// Pausing / Decelerating
			: (gCodes->IsResuming()) 									? 'R'	// Resuming
			: (gCodes->IsPaused()) 										? 'S'	// Paused / Stopped
			: (printMonitor->IsPrinting())								? 'P'	// Printing
			: (gCodes->DoingFileMacro() || !move->NoLiveMovement()) 	? 'B'	// Busy
			: 'I';																// Idle
}

bool RepRap::NoPasswordSet() const
{
	return (!password[0] || StringEquals(password, DEFAULT_PASSWORD));
}

bool RepRap::CheckPassword(const char *pw) const
{
	return StringEquals(pw, password);
}

void RepRap::SetPassword(const char* pw)
{
	// Users sometimes put a tab character between the password and the comment, so allow for this
	CopyParameterText(pw, password, ARRAY_SIZE(password));
}

const char *RepRap::GetName() const
{
	return myName;
}

void RepRap::SetName(const char* nm)
{
	// Users sometimes put a tab character between the machine name and the comment, so allow for this
	CopyParameterText(nm, myName, ARRAY_SIZE(myName));

	// Set new DHCP hostname
	network->SetHostname(myName);
}

// Given that we want to extrude/retract the specified extruder drives, check if they are allowed.
// For each disallowed one, log an error to report later and return a bit in the bitmap.
// This may be called by an ISR!
unsigned int RepRap::GetProhibitedExtruderMovements(unsigned int extrusions, unsigned int retractions)
{
	if (GetHeat()->ColdExtrude())
	{
		return 0;
	}

	Tool *tool = currentTool;
	if (tool == nullptr)
	{
		// This should not happen, but if on tool is selected then don't allow any extruder movement
		return extrusions | retractions;
	}

	unsigned int result = 0;
	for (size_t driveNum = 0; driveNum < tool->DriveCount(); driveNum++)
	{
		const unsigned int extruderDrive = (unsigned int)(tool->Drive(driveNum));
		const unsigned int mask = 1 << extruderDrive;
		if (extrusions & mask)
		{
			if (!tool->ToolCanDrive(true))
			{
				result |= mask;
			}
		}
		else if (retractions & mask)
		{
			if (!tool->ToolCanDrive(false))
			{
				result |= mask;
			}
		}
	}

	return result;
}

void RepRap::FlagTemperatureFault(int8_t dudHeater)
{
	if (toolList != nullptr)
	{
		toolList->FlagTemperatureFault(dudHeater);
	}
}

void RepRap::ClearTemperatureFault(int8_t wasDudHeater)
{
	reprap.GetHeat()->ResetFault(wasDudHeater);
	if (toolList != nullptr)
	{
		toolList->ClearTemperatureFault(wasDudHeater);
	}
}

// End
