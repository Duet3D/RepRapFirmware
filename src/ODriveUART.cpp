#include "ODriveUART.h"

// Print with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

ODriveUART::ODriveUART(Stream& serial)
	: serial_(serial), serialAttached(true){}

ODriveUART::ODriveUART()
	: serial_(SERIAL_AUX_DEVICE), serialAttached(true){}

void ODriveUART::SetSerial(Stream& serial)
{
	serial_ = serial;
	serialAttached = true;
}

void ODriveUART::SetPosition(int motor_number, float position) {
	SetPosition(motor_number, position, 0.0f, 0.0f);
}

void ODriveUART::SetPosition(int motor_number, float position, float velocity_feedforward) {
	SetPosition(motor_number, position, velocity_feedforward, 0.0f);
}

void ODriveUART::SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward) {
	serial_ << "p " << motor_number  << " " << position << " " << velocity_feedforward << " " << current_feedforward << "\n";
}

void ODriveUART::SetVelocity(int motor_number, float velocity) {
	SetVelocity(motor_number, velocity, 0.0f);
}

void ODriveUART::SetVelocity(int motor_number, float velocity, float current_feedforward) {
	serial_ << "v " << motor_number  << " " << velocity << " " << current_feedforward << "\n";
}

void ODriveUART::SetCurrent(int motor_number, float current) {
	serial_ << "c " << motor_number  << " " << current << "\n";
}

float ODriveUART::readFloat() {
	// TODO: To be cool, we should use the included StringRef library here
    char str[50];
    if(!readString(str, 50))
	{
		readString(str, 50);
	}
	return SafeStrtof(str);
}

int32_t ODriveUART::readInt() {
    char str[50];
    if(!readString(str, 50))
	{
		readString(str, 50);
	}
	return (int32_t)SafeStrtol(str);
}

void ODriveUART::flush()
{
	// Just read out the first broken transmission.
	// I don't know why the first UART package is always broken
	serial_ << "r axis0" << ".encoder.config.cpr\n";
	readInt();
	serial_.flush();
	/*
	int trycount = 100;
	serial_.flush();
	while (serial_.available() && --trycount > 0) {
		serial_.read();
	}
	*/
}

int32_t ODriveUART::AskForEncoderConfigCpr(int axis)
{
	serial_ << "r axis" << axis << ".encoder.config.cpr\n";
	return readInt();
}


float ODriveUART::AskForEncoderPosEstimate(int axis)
{
	serial_ << "r axis" << axis << ".encoder.pos_estimate\n";
	return readFloat();
}

/*
float ODriveUART::SetEncoderPosReference(int axis)
{
	encoderPosReference = AskForEncoderPosEstimate(axis);
	if (encoderPosReference == 0.0)
	{
		int timeout_ctr = 100;
		do {
			delay(100);
			serial_ << "r axis" << axis << ".encoder.pos_estimate\n";
			encoderPosReference = readFloat();
		} while (encoderPosReference == 0.0 && --timeout_ctr > 0);
	}
	return encoderPosReference;
}
*/

void ODriveUART::SetCtrlMode(int axis, int ctrlMode) {
	serial_ << "w axis" << axis << ".controller.config.control_mode " << ctrlMode << "\n";
}

void ODriveUART::EnableCurrentControlMode(int axis) {
	serial_ << "w axis" << axis << ".controller.config.control_mode " << CTRL_MODE_CURRENT_CONTROL << "\n";
}

void ODriveUART::EnablePositionControlMode(int axis) {
	serial_ << "w axis" << axis << ".controller.config.control_mode " << CTRL_MODE_POSITION_CONTROL << "\n";
}

void ODriveUART::SetPosSetpoint(int axis, float position) {
	serial_ << "w axis" << axis  << ".controller.pos_setpoint " << position << '\n';
}

bool ODriveUART::run_state(int axis, int requested_state, bool wait) {
	int timeout_ctr = 100;
	serial_ << "w axis" << axis << ".requested_state " << requested_state << '\n';
	if (wait) {
		do {
			delay(100);
			serial_ << "r axis" << axis << ".current_state\n";
		} while (!readInt() && --timeout_ctr > 0);
	}

	return timeout_ctr > 0;
}

size_t ODriveUART::readString(char* str, size_t size) {
	size_t found_chars = 0;
	static const unsigned long timeout = 2000;
	unsigned long timeout_start = millis();
	while (found_chars < size-1) {
		while (!serial_.available()) {
			if (millis() - timeout_start >= timeout) {
                str[found_chars+1] = '\0';
				return found_chars;
			}
		}
		char c = serial_.read();
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
/* End of tucked in file ODriveUART.cpp */
