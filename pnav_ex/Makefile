# Makefile
# Starts a stage world for localization and navigation
LOG=FALSE   # Disable logging by default
					 # To enable set it to 'TRUE'

.PHONY: tams pnav all uhh

all: 
	@echo
	@echo "make pnav [LOG=TRUE]\t-- Starts a simple navigation example"
	@echo "make tams [LOG=TRUE]\t-- Starts the TAMS floor nav example"
	@echo "make uhh  [LOG=TRUE]\t-- Starts the UHH floor nav example"
	@echo "make slam LOGFILE=<logfile>\t-- Start pmaptest creating a grid map"
	@echo "make slamtams LOGFILE=<logfile>\t-- Start pmaptest creating a tams grid map"
	@echo

pnav:
	./stage pnav ${LOG}

tams:
	./stage tams ${LOG}

uhh:
	./stage uhh ${LOG}

slam:
	pmaptest --num_samples 100 --grid_width 16 --grid_height 16 --grid_scale 0.08 --laser_x 0.13 --robot_x -7 --robot_y -7 --robot_rot 90 ${LOGFILE}

slamtams:
	pmaptest --num_samples 100 --grid_width 86.016 --grid_height 22.428 --grid_scale 0.08 --laser_x 0.13 --robot_x 0 --robot_y 0 --robot_rot 0 ${LOGFILE}

