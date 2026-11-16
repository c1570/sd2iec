# Host-side tests for the D64 REL file support.
#
# The firmware sources are compiled against the shim headers in shims/
# and linked with the stubs in stubs.c; harness.c includes d64ops.c
# directly to reach its internal functions.
#
#   make check      build and run the REL CRUD tests
#   make validate   verify the resulting images with the Python checks

SRC       := ../../src
C1541     ?= c1541
BUILD     := build
TESTDATA  := testdata

CFLAGS    ?= -g -O1 -Wall -Wextra -Wno-unused-parameter -Wno-format-truncation
SANFLAGS  := -fsanitize=address,undefined -fno-sanitize-recover=all
INCLUDES  := -I$(BUILD) -Ishims -I$(SRC)

HARNESS_SRCS := harness.c stubs.c
HEADERS   := stub.h $(wildcard shims/*.h) $(wildcard $(SRC)/*.h)

# The firmware sources are compiled from copies in $(BUILD): a source
# file's own directory comes first for quoted includes, so compiling
# them in $(SRC) would pick up src/config.h instead of the shim.
RELDEPS   := $(SRC)/d64ops.c $(SRC)/buffers.c
SRCCOPY   := $(BUILD)/.sources

IMAGES    := $(TESTDATA)/blank.d64 $(TESTDATA)/blank.d71 \
             $(TESTDATA)/blank.d81 $(TESTDATA)/multi.d64 \
             $(TESTDATA)/c1541.d64
IMAGESTAMP := $(TESTDATA)/.images

# working copies the harness leaves in the current directory
WORKFILES := work.d64 work.d71 work.d81 multi.d64 blank.d71 blank.d81

.PHONY: check clean validate

check: $(BUILD)/reltest $(BUILD)/reltest.asan $(IMAGESTAMP)
	$(BUILD)/reltest $(TESTDATA)
	$(BUILD)/reltest.asan $(TESTDATA)

# genimages.py writes all images in one run
$(IMAGESTAMP): genimages.py d64lib.py
	@mkdir -p $(TESTDATA)
	C1541=$(C1541) python3 genimages.py $(TESTDATA)
	@touch $@

$(SRCCOPY): $(RELDEPS) | $(BUILD)
	cp $(RELDEPS) $(BUILD)/
	@touch $@

$(BUILD)/reltest: $(HARNESS_SRCS) $(HEADERS) $(SRCCOPY)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(HARNESS_SRCS) $(BUILD)/buffers.c

$(BUILD)/reltest.asan: $(HARNESS_SRCS) $(HEADERS) $(SRCCOPY)
	$(CC) $(CFLAGS) $(SANFLAGS) $(INCLUDES) -o $@ $(HARNESS_SRCS) $(BUILD)/buffers.c

$(BUILD):
	mkdir -p $(BUILD)

validate: check
	C1541=$(C1541) python3 validate.py $(TESTDATA)

clean:
	rm -rf $(BUILD) $(TESTDATA) __pycache__ $(WORKFILES)
