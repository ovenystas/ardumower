#---------
#
# CppUTest Examples Makefile
#
#----------

#Set this to @ to keep the makefile quiet
ifndef SILENCE
	SILENCE = #@
endif

#--- Inputs ----#
COMPONENT_NAME = ArduMower_Test
CPPUTEST_HOME = ../../cpputest

CPPUTEST_USE_EXTENSIONS = Y
CPPUTEST_USE_GCOV = Y
CPPUTEST_USE_VPATH = Y

CPP_PLATFORM = Gcc

# This line is overriding the default new macros.  This is helpful
# when using std library includes like <list> and other containers
# so that memory leak detection does not conflict with stl.
#CPPUTEST_MEMLEAK_DETECTOR_NEW_MACRO_FILE = -include ApplicationLib/ExamplesNewOverrides.h
SRC_DIRS = \
	src \

SRC_FILES = \
	../ArduMower/Bumper.cpp \
	../ArduMower/Button.cpp \
	../ArduMower/DropSensor.cpp \
	../ArduMower/Encoder.cpp \
	../ArduMower/Filter.cpp \
	../ArduMower/Pid.cpp \
	../ArduMower/RainSensor.cpp \

TEST_SRC_DIRS = \
	tests

MOCKS_SRC_DIRS = \
	mocks \

INCLUDE_DIRS = \
  src \
  mocks \
  ../ArduMower \
  ../ArduMower/inc \
  $(CPPUTEST_HOME)/include \

include $(CPPUTEST_HOME)/build/MakefileWorker.mk


