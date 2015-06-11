#! /bin/make
##############################################
# Makefile for the 'mvDeviceConfigure' sample,
##############################################
#-------------------------------------------
DEST:=$(shell basename `pwd`)

#-------------------------------------------
ifndef MVIMPACT_ACQUIRE_SOURCE_DIR
    ifndef MVIMPACT_ACQUIRE_DIR
        ROOT_PATH=../..
    else
        ROOT_PATH=$(MVIMPACT_ACQUIRE_DIR)/apps
    endif
else
    ROOT_PATH=$(MVIMPACT_ACQUIRE_SOURCE_DIR)/apps
endif

#-------------------------------------------
BINARY_NAME=mvDeviceConfigure
MODULE_NAME=DeviceConfigureFrame

#-------------------------------------------
OOBJ = \
	DebugFileParser.o \
	DeviceHandler.o \
	DeviceHandlerBlueDevice.o \
	DeviceHandlerBlueFOX.o \
	DeviceHandlerHYPERION.o \
	DeviceListCtrl.o \
	Epcs.o \
	KernelDriverUpdate.o \
	LogOutputConfigurationDlg.o \
	LogOutputHandlerDlg.o \
	PackageDescriptionParser.o

#-------------------------------------------
vpath %.cpp ..:../linux

#-----------------------------------------
LOC_LIB_PATH=$(WORK)/lib/$(TARGET)

#-------------------------------------------
# 'WX_CONF_TOOL' will be set in Makefile.samp.inc
#-------------------------------------------
WXRELEASE=$(shell $(WX_CONF_TOOL) --release)
WXLIBFLAGS=$(shell $(WX_CONF_TOOL) --libs)
WXCXXFLAGS=$(shell $(WX_CONF_TOOL) --cxxflags)

#-------------------------------------------
USR_INCLS =				\

#-------------------------------------------
USR_LIBS =				\
	$(ADD_LIBS)

#-------------------------------------------
LDOPT_1 =				\
	$(WXLIBFLAGS)			\

ifneq ($(DEST),ppc603e)
LOCALFLAGS=				\
	-Wno-variadic-macros -Wno-strict-aliasing
endif

#-------------------------------------------
CPPFLAGS_1 = 				\
	-pedantic -Wno-long-long \
	$(WXCXXFLAGS)			\
	$(LOCALFLAGS)

#-------------------------------------------
include $(ROOT_PATH)/Makefile.samp.inc

#-------------------------------------------
