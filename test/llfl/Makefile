# Host-side tests for the rppico fastloader timing (src/rppico/llfl-common.c).
#
# llfl-common.c is compiled against the shim headers in shims/, which
# replace the bus pins and the cycle counter with the simulation in
# llfltest.c.
#
#   make check      build and run the tests (plain and sanitized)
#   make clean      remove build products

SRC       := ../../src
BUILD     := build

CFLAGS    ?= -g -O1 -Wall -Wextra
SANFLAGS  := -fsanitize=address,undefined -fno-sanitize-recover=all
# The source is compiled from a copy in $(BUILD): a source file's own
# directory comes first for quoted includes, so compiling it in
# src/rppico would pick up the real atomic.h instead of the shim.
INCLUDES  := -iquote shims -iquote $(SRC)/llfl -Ishims

LLFL      := $(BUILD)/llfl-common.c
HEADERS   := $(wildcard shims/*.h shims/hardware/structs/*.h) $(SRC)/llfl/llfl-common.h

.PHONY: check clean

check: $(BUILD)/llfltest $(BUILD)/llfltest.asan
	$(BUILD)/llfltest
	$(BUILD)/llfltest.asan

$(LLFL): $(SRC)/rppico/llfl-common.c | $(BUILD)
	cp $< $@

$(BUILD)/llfltest: llfltest.c $(LLFL) $(HEADERS) Makefile
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ llfltest.c $(LLFL)

$(BUILD)/llfltest.asan: llfltest.c $(LLFL) $(HEADERS) Makefile
	$(CC) $(CFLAGS) $(SANFLAGS) $(INCLUDES) -o $@ llfltest.c $(LLFL)

$(BUILD):
	mkdir -p $@

clean:
	rm -rf $(BUILD)
