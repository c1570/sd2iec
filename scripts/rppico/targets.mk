# architecture-dependent additional targets and manual dependencies

PICO_LOG = $(PICO_BUILDDIR)/cmake.log

# Run a command, show its output only on failure unless V=1
ifdef V
  pico_quiet = $(1)
else
  pico_quiet = $(1) >$(PICO_LOG) 2>&1 || (cat $(PICO_LOG); false)
endif

# Configure the SDK project and export its compiler flags
$(PICO_FLAGS_MK): scripts/rppico/CMakeLists.txt scripts/rppico/eeprom.ld $(CONFFILES)
	@test -n "$(PICO_SDK_PATH)" || \
	  (echo "ERROR: Please set PICO_SDK_PATH to a Pico SDK checkout."; false)
	@test -n "$(PICO_PLATFORM)" || \
	  (echo "ERROR: CONFIG_MCU=$(CONFIG_MCU) is not supported."; false)
	$(E) "  CMAKE  $(PICO_BUILDDIR)"
	$(Q)mkdir -p $(PICO_BUILDDIR)
	$(Q)$(call pico_quiet,cmake -S scripts/rppico -B $(PICO_BUILDDIR) \
	  -DCMAKE_BUILD_TYPE=MinSizeRel \
	  -DPICO_SDK_PATH=$(abspath $(PICO_SDK_PATH)) \
	  -DPICO_PLATFORM=$(PICO_PLATFORM) \
	  -DPICO_BOARD=$(PICO_BOARD) \
	  -DSD2IEC_LIB=$(abspath $(OBJDIR)/libsd2iec.a) \
	  -DSD2IEC_FLAGS_MK=$(abspath $@))
	$(Q)touch $@

# All sd2iec objects, linked by the SDK project
$(OBJDIR)/libsd2iec.a: $(OBJ)
	$(E) "  AR     $@"
	$(Q)$(REMOVE) $@
	$(Q)$(AR) rcs $@ $^

# Explicit rule, overrides the generic link rule in Makefile.main
$(TARGET).elf: $(OBJDIR)/libsd2iec.a $(PICO_FLAGS_MK)
	$(E) "  LINK   $@"
	$(Q)$(call pico_quiet,cmake --build $(PICO_BUILDDIR))
	$(Q)$(COPY) $(PICO_BUILDDIR)/sd2iec.elf $@
	$(Q)$(COPY) $(PICO_BUILDDIR)/sd2iec.uf2 $(TARGET).uf2

$(TARGET).uf2: $(TARGET).elf

build: $(TARGET).uf2

# picotool as fetched by the SDK unless installed
PICOTOOL ?= $(firstword $(wildcard $(PICO_BUILDDIR)/_deps/picotool/picotool) picotool)

program: $(TARGET).uf2
	$(E) "  FLASH  $<"
	$(Q)$(PICOTOOL) load -x -f $<

clean: clean-rppico

clean-rppico:
	$(Q)$(REMOVE) -r $(PICO_BUILDDIR)
	$(Q)$(REMOVE) $(PICO_FLAGS_MK) $(OBJDIR)/libsd2iec.a $(TARGET).uf2

.PHONY: clean-rppico
