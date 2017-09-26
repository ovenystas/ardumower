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
CPPUTEST_HOME = C:/Users/oveny/git/cpputest

CPPUTEST_USE_EXTENSIONS = Y
CPP_PLATFORM = Gcc

# This line is overriding the default new macros.  This is helpful
# when using std library includes like <list> and other containers
# so that memory leak detection does not conflict with stl.
#CPPUTEST_MEMLEAK_DETECTOR_NEW_MACRO_FILE = -include ApplicationLib/ExamplesNewOverrides.h
SRC_DIRS = \
	src \

SRC_FILES = \
	../ArduMower/Filter.cpp \

TEST_SRC_DIRS = \
	tests

INCLUDE_DIRS = \
  src \
  mocks \
  ../ArduMower/inc \
  $(CPPUTEST_HOME)/include \

include $(CPPUTEST_HOME)/build/MakefileWorker.mk


