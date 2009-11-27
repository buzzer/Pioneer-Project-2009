# Makefile for the Pioneer Projekt
# Shall be compatible with Uni Network and my local Mac!
# V 0.9
# by Sebastian Rockel
# 2009-05-25
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
          -Wall  \
					-Wcast-align\
					-Waggregate-return \
					-Wcast-qual\
					-Wconversion\
					-Wcomment\
					-Wno-deprecated-declarations\
					-Wdisabled-optimization\
					-Wreturn-type -Wfatal-errors\
					-Wunused\
          -O2    \
          `pkg-config --cflags playerc++`
LIBS    = `pkg-config --libs playerc++`

.PHONY: all clean player playerp view run tag doc docclean sync
all: ${TARGET} 

${TARGET}: ${SRCS} Makefile
	${CC} -o ${TARGET} ${INC} ${CFLAGS} ${SRCS} ${LIBS}

clean::
	rm -f ${TARGET} ${TAGFILE}
	rm -fr ${TMPDIR}
	rm -fr ${TARFILE}

player:
	./stage # Start the player server and stage simulation

playerp:
	./real # Start the player server on real pioneer

view:
	playerv -p 6665 --position:0 --laser:0

run:
	./${TARGET}

tag:
	${CTAGS} -f ${TAGFILE} -R ${TAGSRCS}

doc:
	./makedoc

docclean:
	rm -fr doc/doxygen/*

sync:
	scp Makefile ${HOSTTARGET}
	scp real ${HOSTTARGET}
	scp stage ${HOSTTARGET}
	scp wallfollow.cpp ${HOSTTARGET}
	scp -r stage_local ${HOSTTARGET}
	scp -r pnav_ex ${HOSTTARGET}
	scp -r tams ${HOSTTARGET}

public:
	mkdir ${TMPDIR}
	cp Makefile ${TMPDIR}
	cp real ${TMPDIR}
	cp stage ${TMPDIR}
	cp wallfollow.cpp ${TMPDIR}
	cp -r stage_local ${TMPDIR}
	tar -czvf ${TARFILE} ${TMPDIR}/*
	rm -fr ${TMPDIR}
