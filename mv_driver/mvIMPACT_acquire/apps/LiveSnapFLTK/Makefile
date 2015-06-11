#! /bin/make
############################################
# 03.08.2002 td
# 23.03.2004 td (add OSTYPE)
# 02.11.2005 td (add native)
#===========================================
#        == DO NOT CHANGE !!! ==
############################################
OSTYPE := $(shell uname | cut -d _ -f 1 | tr [:upper:] [:lower:])
HOSTTYPE := $(shell uname -m)

#-------------------------------------------
ifeq ($(OSTYPE),cygwin)
	SUBDIRS=$(OSTYPE)
else
	ifeq ($(HOSTTYPE),i686)
		NATIVE=x86
	else
		NATIVE=$(HOSTTYPE)
		ifeq ($(NATIVE),x86_64)
			EXTRA_TARGET=x86
		endif
		ifeq ($(NATIVE),ppc64)
			EXTRA_TARGET=ppc
		endif
	endif

	SUBDIRS = $(NATIVE) ppc603e arm $(EXTRA_TARGET) xcell xcell64 armv7a
endif

#-------------------------------------------
.PHONY:	$(SUBDIRS) all build new info ipk cygwin clean armv7a

#-------------------------------------------
all info ipk:
	@for dir in $(SUBDIRS) ; do mkdir -p $$dir ; $(MAKE) -C $$dir -f ../Makefile.inc $@ || exit $$?; done

#-------------------------------------------
native:
	@mkdir -p $(NATIVE)
	$(MAKE) -C $(NATIVE) -f ../Makefile.inc all || exit $$?

#-------------------------------------------
ppc603e ppc x86 x86_64 arm cygwin xcell xcell64 armv7a:
	@mkdir -p $@
	$(MAKE) -C $@ -f ../Makefile.inc all || exit $$?

#-------------------------------------------
clean:
	@rm -rf $(SUBDIRS)

#-------------------------------------------
build new: clean
	@for dir in $(SUBDIRS) ; do mkdir -p $$dir; $(MAKE) -C $$dir -f ../Makefile.inc $@ || exit $$?; done

#-------------------------------------------
