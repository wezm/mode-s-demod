#
# When building a package or installing otherwise in the system, make
# sure that the variable PREFIX is defined, e.g. make PREFIX=/usr/local
#
PROGNAME=ten-ninety
RUST_TARGET_DIR=target/debug
DUMP1090=$(RUST_TARGET_DIR)/$(PROGNAME)

ifdef PREFIX
BINDIR=$(PREFIX)/bin
SHAREDIR=$(PREFIX)/share/$(PROGNAME)
EXTRACFLAGS=-DHTMLPATH=\"$(SHAREDIR)\"
endif

all: $(PROGNAME)

$(PROGNAME): src/*.rs
	cargo build

clean:
	cargo clean

test: $(DUMP1090) tests/files/modes1.exp tests/files/modes1.out tests/files/2020-10-03-mel.exp tests/files/2020-10-03-mel.out
	diff -u tests/files/modes1.exp tests/files/modes1.out
	diff -u tests/files/2020-10-03-mel.exp tests/files/2020-10-03-mel.out
	cargo test

tests/files/modes1.out: $(DUMP1090) tests/files/modes1.bin
	./$(DUMP1090) --aggressive --phase-enhance --ifile tests/files/modes1.bin > tests/files/modes1.out

tests/files/2020-10-03-mel.out: $(DUMP1090) tests/files/2020-10-03-mel.bin
	./$(DUMP1090) --aggressive --phase-enhance --ifile tests/files/2020-10-03-mel.bin > tests/files/2020-10-03-mel.out

