# Try uncommenting this if the build fails
#OLD = 1

include geckonator/include.mk

$(OUTDIR)/main.o: events.c

events.c: ics
	$E '  CURL    $@'
	$Q./ics > '$@'

CHIP = EFM32HG322F64
