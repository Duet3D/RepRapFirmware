#include "ODrive.h"

// Print with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

ODrive::ODrive()
	: encoderPosReference{0.0f, 0.0f}, serial_ptr{&SERIAL_AUX_DEVICE}, axes{0, 1}{}

ODrive::ODrive(Stream* serial_ptr_in)
	: encoderPosReference{0.0f, 0.0f}, serial_ptr{serial_ptr_in}, axes{0, 1}{}

ODrive::ODrive(size_t axis0, size_t axis1)
	: encoderPosReference{0.0f, 0.0f}, serial_ptr{&SERIAL_AUX_DEVICE}, axes{axis0, axis1}{}

ODrive::ODrive(size_t axis0, size_t axis1, Stream* serial_ptr_in)
	: encoderPosReference{0.0f, 0.0f}, serial_ptr{serial_ptr_in}, axes{axis0, axis1}{}

void ODrive::SetSerial(Stream* serial_ptr_in)
{
	serial_ptr = serial_ptr_in;
}

void ODrive::SetPosition(ODriveAxis motorNumber, float position) const {
	*serial_ptr << "p " << motorNumber  << " " << position << "\n";
}

void ODrive::SetCurrent(ODriveAxis motorNumber, float current) const {
	*serial_ptr << "c " << motorNumber  << " " << current << "\n";
}

float ODrive::readFloat() const {
	// TODO: To be cool, we should use the included StringRef library here
    char str[50];
    if(!readString(str, 50))
	{
		readString(str, 50);
	}
	return SafeStrtof(str);
}

int32_t ODrive::readInt() const {
    char str[50];
    if(!readString(str, 50))
	{
		readString(str, 50);
	}
	return (int32_t)SafeStrtol(str);
}

void ODrive::flush() const
{
	// Just read out the first broken transmission.
	// I don't know why the first UART package is always broken
	*serial_ptr << "r axis0" << ".encoder.config.cpr\n";
	readInt();

	serial_ptr->flush();
}

int32_t ODrive::AskForEncoderConfigCountsPerRev(ODriveAxis axis) const
{
	*serial_ptr << "r axis" << axis << ".encoder.config.cpr\n";
	return readInt();
}


float ODrive::AskForEncoderPosEstimate(ODriveAxis axis) const
{
	*serial_ptr << "r axis" << axis << ".encoder.pos_estimate\n";
	return readFloat();
}

void ODrive::StoreEncoderPosReference(ODriveAxis axis)
{
	encoderPosReference[axis] = AskForEncoderPosEstimate(axis);
}

void ODrive::StoreCountsPerRev(ODriveAxis axis)
{
	countsPerRev[axis] = AskForEncoderConfigCountsPerRev(axis);
}

void ODrive::SetCtrlMode(ODriveAxis axis, int ctrlMode) const
{
	*serial_ptr << "w axis" << axis << ".controller.config.control_mode " << ctrlMode << "\n";
}

void ODrive::EnableCurrentControlMode(ODriveAxis axis) const
{
	*serial_ptr << "w axis" << axis << ".controller.config.control_mode " << CTRL_MODE_CURRENT_CONTROL << "\n";
}

void ODrive::EnablePositionControlMode(ODriveAxis axis) const
{
	*serial_ptr << "w axis" << axis << ".controller.config.control_mode " << CTRL_MODE_POSITION_CONTROL << "\n";
}

void ODrive::SetPosSetpoint(ODriveAxis axis, float position) const
{
	*serial_ptr << "w axis" << axis  << ".controller.pos_setpoint " << position << '\n';
}

bool ODrive::run_state(ODriveAxis axis, int requested_state, bool wait) const
{
	int timeout_ctr = 100;
	*serial_ptr << "w axis" << axis << ".requested_state " << requested_state << '\n';
	if (wait) {
		do {
			delay(100);
			*serial_ptr << "r axis" << axis << ".current_state\n";
		} while (!readInt() && --timeout_ctr > 0);
	}

	return timeout_ctr > 0;
}

size_t ODrive::readString(char* str, size_t size) const {
	size_t found_chars = 0;
	static const unsigned long timeout = 2000;
	unsigned long timeout_start = millis();
	while (found_chars < size-1) {
		while (!serial_ptr->available()) {
			if (millis() - timeout_start >= timeout) {
                str[found_chars+1] = '\0';
				return found_chars;
			}
		}
		char c = serial_ptr->read();
		if (c == '\n')
        {
            str[found_chars+1] = '\0';
			break;
        }
		str[found_chars] = c;
		found_chars++;
	}
	str[found_chars] = '\0';
	return found_chars;
}

ODriveAxis ODrive::AxisToODriveAxis(size_t axis) const
{
	if (axes[0] == axis)
	{
		return M0;
	}
	if (axes[1] == axis)
	{
		return M1;
	}
	return NO_AXIS;
}

void ODrive::SetRRFToODriveAxis(ODriveAxis odrvAxis, size_t axis)
{
	if (odrvAxis != NO_AXIS) {
		axes[odrvAxis] = axis;
	}
}
