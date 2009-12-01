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
	@echo

pnav:
	./stage pnav ${LOGTRUE}

tams:
	./stage tams ${LOGTRUE}

uhh:
	./stage uhh ${LOGTRUE}
