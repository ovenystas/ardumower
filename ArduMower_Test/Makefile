#---------
#
# CppUTest Examples Makefile
#
#----------

#Set this to @ to keep the makefile quiet
ifndef SILENCE
	SILENCE = @
endif

#--- Inputs ----#
COMPONENT_NAME = ArduMower
CPPUTEST_HOME = ../../cpputest

CPPUTEST_USE_EXTENSIONS = Y
CPPUTEST_USE_GCOV = Y
CPPUTEST_USE_VPATH = N

CPP_PLATFORM = Gcc

# This line is overriding the default new macros.  This is helpful
# when using std library includes like <list> and other containers
# so that memory leak detection does not conflict with stl.
#CPPUTEST_MEMLEAK_DETECTOR_NEW_MACRO_FILE = -include ApplicationLib/ExamplesNewOverrides.h

#SRC_DIRS = \
	../ArduMower \

SRC_FILES = \
	../ArduMower/Battery.cpp \
	../ArduMower/Bumper.cpp \
	../ArduMower/Button.cpp \
	../ArduMower/Buzzer.cpp \
	../ArduMower/DropSensor.cpp \
	../ArduMower/Encoder.cpp \
	../ArduMower/Filter.cpp \
	../ArduMower/Gps.cpp \
	../ArduMower/LawnSensor.cpp \
	../ArduMower/Pid.cpp \
	../ArduMower/RainSensor.cpp \
	../ArduMower/Sonar.cpp \

TEST_SRC_DIRS = \
	tests \

MOCKS_SRC_DIRS = \
	mocks \

INCLUDE_DIRS = \
 	../ArduMower \
 	$(CPPUTEST_HOME)/include \

CPPUTEST_CPPFLAGS += -std=gnu++11	# This is what Arduino 1.8.12 is using

include $(CPPUTEST_HOME)/build/MakefileWorker.mk
