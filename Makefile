# Makefile for the Pioneer Projekt
# Shall be compatible with Uni Network and my local Mac!
# V 0.9
# by Sebastian Rockel
# 2009-12-11
# ++++++++++++++++++++++++++++++++++++++++++++++++++++++
#
CC      = g++
CTAGS   = ctags
TAGFILE = tags
TARGET  = wallfollow
SRCS    = ${TARGET:=.cpp}
OBJS    = ${SRCS:.cpp=.obj}
DEPS    = ${SRCS:.cpp=.dep}
XDEPS   = ${wildcard ${DEPS}}
TAGSRCS = /usr/local/include/player-3.*/libplayerc++/
HOSTTARGET= "demo@tams67:projekt090406/" # to sync target
TMPDIR  = ./PlayerSource
TARFILE = PlayerSource.tgz

CFLAGS  = -pg    \
          -g3    \
          -ggdb    \
          -funit-at-a-time \
          -Wall  \
          -Wcast-align\
          -Waggregate-return \
          -Wcast-qual\
          -Wcomment\
          -Wno-deprecated-declarations\
          -Wdisabled-optimization\
          -Wreturn-type -Wfatal-errors\
          -Wunused\
          `pkg-config --cflags playerc++`
LIBS    = `pkg-config --libs playerc++`

.PHONY: all clean player playerp view run tag doc docclean sync

all:
	@echo
	@echo "make wallfollow\t-- Wallfollow compilation"
	@echo "make clean\t-- Clean objects"
	@echo "make player\t-- Start the player server and stage simulation"
	@echo "make playerp\t-- Start the player server on real pioneer"
	@echo "make view\t-- Start playerv for sensor data"
	@echo "make slam LOGFILE=<logfile>\t-- Start pmaptest creating a grid map"
	@echo "make debug\t-- Start debugger ddd with wallfollow"
	@echo "make tag\t-- Create tags for VIM"
	@echo "make doc\t-- Create doxygen manual for wallfollowing program"
	@echo "make docclean\t-- Clean doxygen manual and files"
	@echo "make sync\t-- Sync mandatory wallfollowing files onto robot laptop"
	@echo "make public\t-- Create a zip archive from mandatory wallfollowing files"
	@echo

${TARGET}: ${SRCS} Makefile
	${CC} -o ${TARGET} ${INC} ${CFLAGS} ${SRCS} ${LIBS}

clean::
	rm -f ${TARGET} ${TAGFILE}
	rm -fr ${TMPDIR}
	rm -fr ${TARFILE}

player:
	./start uhh wallfollow # Start the player server and stage simulation

playerp:
	./start pioneer wallfollow  # Start the player server on real pioneer

view:
	playerv -p 6665 --position:0 --laser:0

slam:
	pmaptest --num_samples 100 --grid_width 16 --grid_height 16 --grid_scale 0.08 --laser_x 0.13 --robot_x -7 --robot_y -7 --robot_rot 90 ${LOGFILE}

debug:
	player stage_local/uhh.cfg &
	ddd ${TARGET} &

tag:
	${CTAGS} -f ${TAGFILE} -R ${TAGSRCS}

doc:
	./makedoc

docclean:
	rm -fr doc/doxygen/*

sync:
	@scp Makefile ${HOSTTARGET}
	@scp start ${HOSTTARGET}
	@scp wallfollow.cpp ${HOSTTARGET}
	@scp -r stage_local ${HOSTTARGET}
	@scp -r pnav_ex ${HOSTTARGET}
	@scp -r tams ${HOSTTARGET}
	@echo "\nCopied files to ${HOSTTARGET}\n"

public:
	@mkdir ${TMPDIR}
	@cp Makefile ${TMPDIR}
	@cp start ${TMPDIR}
	@cp wallfollow.cpp ${TMPDIR}
	@cp -r stage_local ${TMPDIR}
	@tar -czvf ${TARFILE} ${TMPDIR}/*
	@rm -fr ${TMPDIR}
	@echo "\nCreated archive ${TARFILE}\n"
