// This is a useless library. To use such a useless thing, with the EnableInterrupt library,
// you need to #define LIBCALL_ENABLEINTERRUPT in your library, like so:
#define LIBCALL_ENABLEINTERRUPT
// Thus, none of the functions or ISRs will get compiled but their prototypes are declared
// so you can use them as illustrated in the Useless.cpp.

// You also need to #include the EnableInterrupt.h file in your sketch. After you #include the
// EnableInterrupt.h file, #include the library's .h file (ie, this one in this example). This
// will compile the requisite components. See the SimpleWithLibrary.ino sketch.

#include <stdint.h>
#include <Arduino.h>

#define EI_ARDUINO_INTERRUPTED_PIN
#include <EnableInterrupt.h>

class UselessClass {
	public:
		UselessClass(uint8_t pin, uint8_t mode) {
			init(pin, mode);
		}
		uint8_t getUselessVariable();
		void reset();
		void disable(uint8_t pin);
	private:
		void init(uint8_t pin, uint8_t mode);
};

