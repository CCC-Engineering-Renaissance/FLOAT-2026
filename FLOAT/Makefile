# =============================================================================
# Makefile for the FLOAT-2026 firmware (ESP32-S3 WROOM, arduino-cli)
#
# Thin wrapper around arduino-cli. The mission sketch lives in src/main/
# (folder name must match main.ino). CDCOnBoot=cdc routes the Serial console
# over native USB so GPIO43/44 stay free for the HC-12 radio on Serial1.
#
#   make            compile the firmware
#   make upload     compile + flash over USB (set PORT=/dev/ttyACM0)
#   make monitor    open the USB serial monitor
#   make clean      remove the local build output
#   make check      verify arduino-cli + the esp32 core are installed
#   make help       list targets
# =============================================================================

# --- config ------------------------------------------------------------------
CLI     ?= arduino-cli
FQBN    ?= esp32:esp32:esp32s3:CDCOnBoot=cdc
SKETCH  ?= src/main
PORT    ?= /dev/ttyACM0
MONITOR_SPEED ?= 115200
BUILDDIR = build

# --- targets -----------------------------------------------------------------
.PHONY: all build compile upload monitor clean check help

all: build
build: compile

compile:
	$(CLI) compile --fqbn "$(FQBN)" --build-path $(BUILDDIR) $(SKETCH)

# Compile + flash over USB. Override PORT to your board, e.g. make upload PORT=/dev/ttyACM0
upload:
	$(CLI) compile --fqbn "$(FQBN)" --build-path $(BUILDDIR) $(SKETCH)
	$(CLI) upload  --fqbn "$(FQBN)" -p $(PORT) $(SKETCH)

monitor:
	$(CLI) monitor -p $(PORT) -c baudrate=$(MONITOR_SPEED)

clean:
	rm -rf $(BUILDDIR)

# Verify arduino-cli and the esp32 core are present.
check:
	@command -v $(CLI) >/dev/null 2>&1 || { echo "arduino-cli not found"; exit 1; }
	@$(CLI) core list | grep -q "esp32:esp32" || { \
	  echo "esp32 core missing. Install: $(CLI) core install esp32:esp32"; exit 1; }
	@$(CLI) version

help:
	@echo "FLOAT-2026 firmware (ESP32-S3, arduino-cli) — make targets:"
	@echo "  make           compile the firmware"
	@echo "  make upload     compile + flash over USB (set PORT=...)"
	@echo "  make monitor    open serial monitor ($(MONITOR_SPEED) baud)"
	@echo "  make clean      remove the local build output"
	@echo "  make check      verify arduino-cli + esp32 core"
