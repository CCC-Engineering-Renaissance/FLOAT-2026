# =============================================================================
# Makefile for the FLOAT-2026 firmware (Raspberry Pi Pico / RP2040)
#
# This is a thin convenience wrapper around PlatformIO — the actual cross
# compile + link is done by PlatformIO's RP2040 Arduino toolchain. The Makefile
# just gives you short, memorable commands.
#
#   make            build the firmware (-> .pio/build/pico/firmware.uf2)
#   make upload     build + flash over USB (auto-reset; Pico must be reachable)
#   make flash      copy the .uf2 onto a mounted RPI-RP2 drive (BOOTSEL method)
#   make monitor    open the USB serial monitor
#   make clean      remove build artifacts
#   make uf2        print the path to the built .uf2
#   make help       list targets
#
# WSL2 note: USB isn't visible in WSL by default, so `make upload`/`make monitor`
# need usbipd. The easy path is `make` here, then drag the .uf2 onto the RPI-RP2
# drive from Windows — or set UF2_DEST (below) to that drive and `make flash`.
# =============================================================================

# --- config ------------------------------------------------------------------
PIO     ?= pio
ENV     ?= pico2
BUILDDIR = .pio/build/$(ENV)
UF2      = $(BUILDDIR)/firmware.uf2

# Where `make flash` copies the .uf2. Override to your mounted Pico, e.g.:
#   make flash UF2_DEST=/mnt/d            (WSL, RPI-RP2 mounted as D:)
#   make flash UF2_DEST=/media/$(USER)/RPI-RP2
UF2_DEST ?= /media/$(USER)/RPI-RP2

# Serial monitor speed (matches Serial.begin(115200) in main.cpp)
MONITOR_SPEED ?= 115200

# --- targets -----------------------------------------------------------------
.PHONY: all build upload flash monitor clean fullclean uf2 check help

all: build

build:
	$(PIO) run -e $(ENV)

# Build + flash over USB (PlatformIO performs the 1200-baud auto-reset).
upload:
	$(PIO) run -e $(ENV) -t upload

# BOOTSEL drag-drop, scripted: build, then copy the .uf2 onto the Pico's
# mass-storage drive. Hold BOOTSEL while plugging in so RPI-RP2 mounts first.
flash: build
	@test -d "$(UF2_DEST)" || { \
	  echo "ERROR: '$(UF2_DEST)' not found. Put the Pico in BOOTSEL mode and set"; \
	  echo "       UF2_DEST to the mounted RPI-RP2 drive, e.g. make flash UF2_DEST=/mnt/d"; \
	  exit 1; }
	cp "$(UF2)" "$(UF2_DEST)/"
	@echo "Copied $(UF2) -> $(UF2_DEST)/  (Pico will reboot and run)"

monitor:
	$(PIO) device monitor -b $(MONITOR_SPEED)

clean:
	$(PIO) run -e $(ENV) -t clean

# Nuke the whole PlatformIO build tree (forces a fresh toolchain/lib resolve).
fullclean:
	rm -rf .pio

uf2: build
	@echo "$(UF2)"

# Verify PlatformIO is installed before doing anything useful.
check:
	@command -v $(PIO) >/dev/null 2>&1 && $(PIO) --version || { \
	  echo "PlatformIO not found. Install with: pip install platformio"; exit 1; }

help:
	@echo "FLOAT-2026 firmware — make targets:"
	@echo "  make           build the .uf2"
	@echo "  make upload     build + flash over USB"
	@echo "  make flash      copy .uf2 to a mounted RPI-RP2 (set UF2_DEST)"
	@echo "  make monitor    open serial monitor ($(MONITOR_SPEED) baud)"
	@echo "  make clean      remove build artifacts"
	@echo "  make fullclean  remove the entire .pio tree"
	@echo "  make uf2        print the built .uf2 path"
	@echo "  make check      verify PlatformIO is installed"
