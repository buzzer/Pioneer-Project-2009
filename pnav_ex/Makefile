# Makefile
# Starts a stage world for localization and navigation
LOGTRUE=   # Disable logging by default
					 # To enable set it to 'LOGTRUE'

.PHONY: tams pnav all uhh

all: 
	@echo
	@echo "make pnav [LOGTRUE=LOGTRUE]\t-- Starts a simple navigation example"
	@echo "make tams [LOGTRUE=LOGTRUE]\t-- Starts the TAMS floor nav example"
	@echo "make uhh  [LOGTRUE=LOGTRUE]\t-- Starts the UHH floor nav example"
	@echo "make slam LOGFILE=<logfile>\t-- Start pmaptest creating a grid map"
	@echo

pnav:
	./stage pnav ${LOGTRUE}

tams:
	./stage tams ${LOGTRUE}

uhh:
	./stage uhh ${LOGTRUE}

slam:
	pmaptest --num_samples 100 --grid_width 16 --grid_height 16 --grid_scale 0.08 --laser_x 0.13 --robot_x -7 --robot_y -7 --robot_rot 90 ${LOGFILE}
