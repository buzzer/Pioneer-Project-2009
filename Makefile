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
# MacPort and Uni network paths
INC     = -I/opt/local/include/player-3.0 \
					-I/opt/local/include/player-2.0 \
					-I/opt/local/include \
					-I/informatik/isr/tams/playerstage2.1.0rc1/include/player-2.1

TAGSRCS = /informatik/isr/tams/playerstage2.1.0rc1 stage_local

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
					-Wmost\
					-Wunused\
          -O2    \
          `pkg-config --cflags playerc++`
LIBS    = `pkg-config --libs playerc++`

.PHONY: all clean distclean tags
all: ${TARGET} 

${TARGET}: ${SRCS} Makefile
	${CC} -o ${TARGET} ${INC} ${CFLAGS} ${SRCS} ${LIBS}

clean::
	-rm -f ${TARGET} ${TAGFILE}

distclean:: clean

player:
	player stage_local/simple.cfg

playerp:
	player stage_local/pioneer.cfg

view:
	playerv -p 6665 --position:0 --sonar:0 --laser:0

run:
	./${TARGET}

tags:
	${CTAGS} -f ${TAGFILE} -R ${TAGSRCS}
