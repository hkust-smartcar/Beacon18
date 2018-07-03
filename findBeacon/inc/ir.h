#include "libbase/k60/gpio.h"
#include "libsc/system.h"

using libbase::k60::Gpi;
using libbase::k60::Pin;
using libsc::System;

struct pulse_record{
	uint32_t time = 0;
	uint32_t width = 0;
};

class IR_recevier {
public:
	IR_recevier(libbase::k60::Pin::Name pin) :
			start_time(0), pulse_width(0), state(
					false) {
		gpi_config.pin = pin;
		gpi_config.interrupt = Pin::Config::Interrupt::kBoth;
		gpi_config.isr = [this](Gpi *gpi) {
			uint32_t time = System::Time();
			if (!gpi->Get()) {
				start_time = time;    //low, ir recevied
			} else if(time - start_time < 250) { //high, ir not recevied
				pulse_width = time - start_time;
				state = (time - record.time < 120 && pulse_width > 20 && record.width > 20)?1:0;
				record.time = time;
				record.width = pulse_width;
			}
			else{
				pulse_width = 0;
				state = false;
			}
		};
//		gpi_config.config.set(Pin::Config::ConfigBit::kPassiveFilter);
		m_pin = Gpi(gpi_config);
	}

	bool getState() {
		if (System::Time() - record.time > 110)
			state = false;
		return state;
	}
	uint32_t get_pulse() {
		return pulse_width;
	}

private:
	Gpi m_pin;
	Gpi::Config gpi_config;
	uint32_t start_time;
	pulse_record record;
	uint32_t pulse_width;
	bool state;
};
