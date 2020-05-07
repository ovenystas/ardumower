ARDUINO_CLI_PROJECT = ArduMower
ARDUINO_CLI_FQBN = arduino:avr:mega
ARDUINO_CLI_WARNINGS = all
ARDUINO_CLI_BUILD_PROPERTIES = compiler.cpp.extra_flags="-Werror -fshort-enums",compiler.c.extra_flags="-Werror -fshort-enums"

ifdef VERBOSE
	ARDUINO_CLI_VERBOSE = -v
	SILENCE =
else
	ARDUINO_CLI_VERBOSE =
	SILENCE = @
endif

all:
	$(SILENCE)arduino-cli compile --fqbn $(ARDUINO_CLI_FQBN) --warnings $(ARDUINO_CLI_WARNINGS) --build-properties $(ARDUINO_CLI_BUILD_PROPERTIES) $(ARDUINO_CLI_PROJECT) $(ARDUINO_CLI_VERBOSE)

test:
	$(SILENCE)make -j -C ArduMower_Test

gcov:
	$(SILENCE)make -j -C ArduMower_Test gcov

gcovr:
	$(SILENCE)make -j -C ArduMower_Test gcovr

clean:
	$(SILENCE)rm -rf ArduMower/build
	$(SILENCE)rm -f ArduMower/ArduMower.*.hex
	$(SILENCE)rm -f ArduMower/ArduMower.*.elf

	$(SILENCE)make -j -C ArduMower_Test clean

phony: test gcov clean

