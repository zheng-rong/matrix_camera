#! /bin/make
# -*- sh -*-
#=========================================================================
#        == DO NOT CHANGE !!! ==
#=========================================================================

SUB_DIRS=$(shell ls)

ifeq ($(findstring xcell,$(MAKECMDGOALS)),xcell) # caches both xcell and xcell64
	ifneq ($(shell uname -m),ppc64)
		HOST_OK := "false"
	endif
endif

#-------------------------------------------------------------------------
.PHONY:	all clean strip new build ppc603e ppc ppc64 x86 x86_64 arm cygwin info native ppc_6xx xcell xcell64 armv7a armv6zk

#=========================================================================
all clean strip new build ppc603e ppc x86 x86_64 arm cygwin info native ppc_6xx xcell xcell64 armv7a armv6zk:
ifeq ($(HOST_OK),"false")
	@echo "Cannot build XCell binaries on this host"
else
	@for sdir in $(SUB_DIRS) ;							\
	do													\
		echo $$sdir; 									\
		(												\
			if [ -d $$sdir ];							\
			then										\
				if [ -e $$sdir/Makefile ];				\
				then									\
					echo "make in :"$$sdir;				\
					$(MAKE) -C $$sdir $@ || exit $$?;	\
				fi;										\
			fi											\
		)												\
		done
endif

#-------------------------------------------------------------------------
