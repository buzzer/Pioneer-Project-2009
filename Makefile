# Makefile for the Pioneer Projekt
# Shall be compatible with Uni Network and my local Mac!
# V 0.9
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

.PHONY: all clean distclean tag doc docclean
all: ${TARGET} 

${TARGET}: ${SRCS} Makefile
	${CC} -o ${TARGET} ${INC} ${CFLAGS} ${SRCS} ${LIBS}

clean::
	-rm -f ${TARGET} ${TAGFILE}

distclean:: clean

player:
	player stage_local/uhh.cfg

playerp:
	player stage_local/pioneer.cfg

view:
	playerv -p 6665 --position:0 --laser:0

run:
	./${TARGET}

tag:
	${CTAGS} -f ${TAGFILE} -R ${TAGSRCS}

doc:
	doxygen

docclean:
	rm -fr doc/doxygen/*
