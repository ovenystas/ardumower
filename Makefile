ARDUINO_CLI_PROJECT = ArduMower
ARDUINO_CLI_FQBN = arduino:avr:mega
ARDUINO_CLI_WARNINGS = all
ARDUINO_CLI_BUILD_PROPERTIES = compiler.cpp.extra_flags=-Werror,compiler.c.extra_flags=-Werror

ifdef VERBOSE
	ARDUINO_CLI_VERBOSE = -v
	SILENCE =
else
	ARDUINO_CLI_VERBOSE =
	SILENCE = @
endif

all:
	$(SILENCE)arduino-cli compile --fqbn $(ARDUINO_CLI_FQBN) --warnings $(ARDUINO_CLI_WARNINGS) --build-properties $(ARDUINO_CLI_BUILD_PROPERTIES) $(ARDUINO_CLI_PROJECT) $(ARDUINO_CLI_VERBOSE)
