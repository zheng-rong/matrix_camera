#! /bin/make
SUB_DIRS=                \
    apps                 \

#-------------------------------------------------------------------------
.PHONY:    all clean strip new build arm info native armv7l

#=========================================================================
all:
	make arm

clean strip new build arm info native armv7l:
	@for sdir in $(SUB_DIRS) ;                               \
	do                                                        \
		echo $$sdir;                                        \
		(                                                     \
			DRV_DIR=$$sdir;                                 \
			if [ -d $$DRV_DIR ];                            \
			then                                              \
				if [ -e $$DRV_DIR/Makefile ];               \
				then                                          \
					echo "make in :"$$DRV_DIR;              \
					$(MAKE) --silent -C $$DRV_DIR $@ || exit $$?;	\
				fi;                                           \
			fi                                                \
		)                                                     \
	done
#-------------------------------------------------------------------------
