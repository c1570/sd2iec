# Host-side tests for the REL file support.
#
# The firmware sources are compiled against the shim headers in shims/
# and linked with the stubs in stubs.c; harness.c includes d64ops.c
# directly to reach its internal functions.
#
#   make check      build and run the tests (plain and sanitized) and
#                   validate the resulting images
#   make validate   alias for check
#   make clean      remove build products and test images

SRC       := ../../src
C1541     ?= c1541
PYTHON    ?= python3
BUILD     := build
TESTDATA  := testdata

CFLAGS    ?= -g -O1 -Wall -Wextra -Wno-unused-parameter -Wno-format-truncation
SANFLAGS  := -fsanitize=address,undefined -fno-sanitize-recover=all
# Quoted includes resolve to the shims before src/. src/ is not on the
# angle include path, so its dirent.h does not shadow the system one.
INCLUDES  := -iquote $(BUILD) -iquote shims -iquote $(SRC) -Ishims

HARNESS_SRCS := harness.c stubs.c fatshim.c
HEADERS   := stub.h $(wildcard shims/*.h) $(wildcard $(SRC)/*.h)

# The firmware sources are compiled from copies in $(BUILD): a source
# file's own directory comes first for quoted includes, so compiling
# them in $(SRC) would pick up src/config.h instead of the shim.
RELDEPS   := $(SRC)/d64ops.c $(SRC)/buffers.c $(SRC)/fatops.c \
             $(SRC)/fileops.c $(SRC)/parser.c $(SRC)/utils.c
FAT_SRCS  := $(BUILD)/fatops.c $(BUILD)/fileops.c $(BUILD)/parser.c \
             $(BUILD)/utils.c $(BUILD)/buffers.c
SRCCOPY   := $(BUILD)/.sources

IMAGESTAMP := $(TESTDATA)/.images

.PHONY: check validate clean

check: $(BUILD)/reltest $(BUILD)/reltest.asan $(IMAGESTAMP)
	$(BUILD)/reltest $(TESTDATA)
	$(BUILD)/reltest.asan $(TESTDATA)
	C1541=$(C1541) $(PYTHON) validate.py $(TESTDATA)

validate: check

# genimages.py writes all images in one run
$(IMAGESTAMP): genimages.py d64lib.py Makefile
	@mkdir -p $(TESTDATA)
	C1541=$(C1541) $(PYTHON) genimages.py $(TESTDATA)
	@touch $@

$(SRCCOPY): $(RELDEPS) Makefile | $(BUILD)
	cp $(RELDEPS) $(BUILD)/
	@touch $@

$(BUILD)/reltest: $(HARNESS_SRCS) $(HEADERS) $(SRCCOPY) Makefile
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(HARNESS_SRCS) $(FAT_SRCS)

$(BUILD)/reltest.asan: $(HARNESS_SRCS) $(HEADERS) $(SRCCOPY) Makefile
	$(CC) $(CFLAGS) $(SANFLAGS) $(INCLUDES) -o $@ $(HARNESS_SRCS) $(FAT_SRCS)

$(BUILD):
	mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD) $(TESTDATA) __pycache__
