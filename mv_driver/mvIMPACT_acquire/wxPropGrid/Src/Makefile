#! /bin/make
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

    SUBDIRS = $(NATIVE) ppc603e arm armv7a $(EXTRA_TARGET) xcell xcell64
endif

#-------------------------------------------
.PHONY: $(SUBDIRS) all build new info linfo native clean cygwin xcell xcell64 ppc_6xx

#-------------------------------------------
all info linfo:
	@for dir in $(SUBDIRS) ; do mkdir -p $$dir ; $(MAKE) $(MAKEOPTS) -C $$dir -f ../Makefile.inc $@ || exit $$?; done

#-------------------------------------------
native:
	@mkdir -p $(NATIVE)
	$(MAKE) $(MAKEOPTS) -C $(NATIVE) -f ../Makefile.inc all || exit $$?

#-------------------------------------------
ppc x86 x86_64 cygwin xcell xcell64 arm armv7a:
	@mkdir -p $@
	$(MAKE) $(MAKEOPTS) -C $@ -f ../Makefile.inc all || exit $$?

#-------------------------------------------
clean:
	@for dir in $(SUBDIRS) ; \
		do if test -d $$dir; then \
			$(MAKE) $(MAKEOPTS) -C $$dir -f ../Makefile.inc clean_mod || exit $$?; \
		fi \
	done
	@rm -rf $(SUBDIRS)


#-------------------------------------------
build new: clean
	@for dir in $(SUBDIRS) ; do mkdir -p $$dir; $(MAKE) $(MAKEOPTS) -C $$dir -f ../Makefile.inc $@ || exit $$?; done

#-------------------------------------------
ppc_6xx ppc603e:
	@echo target platform $@ is unsupported

