# ANCLA_ESP32S3 Custom Board Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a new out-of-tree Zephyr board, `innovaforce/ancla_esp32s3`, to this repo so the app builds with `west build -b ancla_esp32s3/esp32s3/procpu`, with devicetree/Kconfig correctly describing the ESP32-S3-WROOM-1-N8R2 module and its onboard MAX17048 fuel gauge, WS2812 RGB LED, and DW3220 UWB transceiver.

**Architecture:** Board files live under this repo's own `boards/innovaforce/ancla_esp32s3/`, added to `BOARD_ROOT` via `CMakeLists.txt` — not inside the shared Zephyr SDK checkout. Built up in four layers, each independently buildable: (1) SoC + console + UART0 + watchdog/entropy/wifi/bt conventions, (2) DW3220's SPI2 bus + its three GPIO control lines + the MAX17048's ALERT line, (3) the MAX17048 I2C0 device node, (4) the WS2812 LED over I2S0. A final task adds the (unused, parity-only) `appcpu` variant.

**Tech Stack:** Zephyr RTOS (west workspace at `~/zephyrproject`), ESP32-S3 (Xtensa), devicetree + Kconfig, no application C code changes.

## Global Constraints

- Board identifier: `ancla_esp32s3/esp32s3/procpu` (and `/appcpu`), vendor `innovaforce`, full name "Innovaforce ANCLA ESP32-S3".
- SoC dtsi: `espressif/esp32s3/esp32s3_wroom_n8r2.dtsi` (8MB flash + 2MB Quad PSRAM) — no size overrides.
- Out-of-tree board: files live in this repo at `boards/innovaforce/ancla_esp32s3/`, reached via `BOARD_ROOT` set in this repo's `CMakeLists.txt`. Never edit `~/zephyrproject/zephyr/boards/`.
- Console = native USB-Serial-JTAG (`&usb_serial`), never `&uart0`. UART0 stays enabled (default pins GPIO43/44) for the external RPi4 link but is never `zephyr,console`/`zephyr,shell-uart`.
- No devicetree binding exists for the DW3220 (Qorvo UWB transceiver) in this Zephyr tree — do not invent one. Its RESET/IRQ/WAKEUP lines, and the MAX17048's ALERT line, are exposed as plain `zephyr,user` `-gpios` properties, not device-node properties. `-gpios` properties never declare input/output direction — that stays out of scope for board files.
- MAX17048 gets a real devicetree node (`compatible = "maxim,max17048"`, `reg = <0x36>`) since Zephyr has an in-tree driver for it.
- RGB LED uses `worldsemi,ws2812-i2s` on I2S0 (no RMT driver exists for ESP32 in this Zephyr version; GPIO bit-banging is timing-fragile on Xtensa). Only the data pin (GPIO38) is physically wired; the other I2S signal pins route to confirmed-spare GPIO33-37/47.
- GPIO bank math for `zephyr,user` phandles: `&gpio0` = absolute pins 0-31 directly; `&gpio1` = absolute pins 32-53 at index `pin - 32`.
- Full pin table (from the approved spec, `docs/superpowers/specs/2026-08-10-ancla-esp32s3-custom-board-design.md`):
  - MAX17048: SDA=GPIO40, SCL=GPIO39, ALERT=GPIO41
  - WS2812 LED: DIN=GPIO38
  - DW3220: MISO=GPIO13, MOSI=GPIO11, CS=GPIO10, SCLK=GPIO12, WAKE_UP=GPIO14, RESET=GPIO21, IRQ=GPIO42
  - UART0 (RPi4 link, not console): TXD0=GPIO43, RXD0=GPIO44 (stock pins)
- Build/verify command for every task (from this repo's directory, with the Zephyr venv active):
  ```bash
  source /c/Users/JoseAntonioLaraPerez/zephyrproject/.venv/Scripts/activate
  cd /c/Users/JoseAntonioLaraPerez/zephyrproject
  west build -p always -b ancla_esp32s3/esp32s3/procpu \
    -s /c/Users/JoseAntonioLaraPerez/Documents/zephyr_projects/esp32/ANCLA_ESP32S3 \
    -d /c/Users/JoseAntonioLaraPerez/Documents/zephyr_projects/esp32/ANCLA_ESP32S3/build
  ```
  Success looks like the ninja build reaching `[100%] ... zephyr.elf` with no `FATAL ERROR`. This is board bring-up, not application logic — there is no unit-test framework here; a clean `west build` (which fully re-parses devicetree and Kconfig from scratch each time thanks to `-p always`) is the test.
- On-hardware verification (flashing the board, confirming the USB serial monitor prints, confirming the RGB LED/fuel-gauge/DW3220 wiring physically works) is **not** part of this plan's automated steps — it requires the physical PCB and is a manual step for the user after this plan is executed.

---

### Task 1: Board scaffolding + minimal bring-up (SoC, USB console, UART0, GPIO/WDT/TRNG/WiFi/BT)

**Files:**
- Modify: `CMakeLists.txt`
- Create: `boards/innovaforce/ancla_esp32s3/board.yml`
- Create: `boards/innovaforce/ancla_esp32s3/Kconfig`
- Create: `boards/innovaforce/ancla_esp32s3/Kconfig.ancla_esp32s3`
- Create: `boards/innovaforce/ancla_esp32s3/Kconfig.sysbuild`
- Create: `boards/innovaforce/ancla_esp32s3/board.cmake`
- Create: `boards/innovaforce/ancla_esp32s3/ancla_esp32s3-pinctrl.dtsi`
- Create: `boards/innovaforce/ancla_esp32s3/ancla_esp32s3_procpu.dts`
- Create: `boards/innovaforce/ancla_esp32s3/ancla_esp32s3_procpu.yaml`
- Create: `boards/innovaforce/ancla_esp32s3/ancla_esp32s3_procpu_defconfig`

**Interfaces:**
- Produces: node labels `&usb_serial`, `&uart0`, `&gpio0`, `&gpio1`, `&wdt0`, `&trng0`, `&wifi`, `&esp32_bt_hci` (all enabled); pinctrl group `uart0_default`; aliases `watchdog0`, `uart-0`. Task 2 extends the same pinctrl file and dts file with new groups/nodes.
- Consumes: nothing (first task).

- [ ] **Step 1: Confirm the board doesn't build yet (sanity check on a clean tree)**

  Run:
  ```bash
  source /c/Users/JoseAntonioLaraPerez/zephyrproject/.venv/Scripts/activate
  cd /c/Users/JoseAntonioLaraPerez/zephyrproject
  west build -p always -b ancla_esp32s3/esp32s3/procpu \
    -s /c/Users/JoseAntonioLaraPerez/Documents/zephyr_projects/esp32/ANCLA_ESP32S3 \
    -d /c/Users/JoseAntonioLaraPerez/Documents/zephyr_projects/esp32/ANCLA_ESP32S3/build
  ```
  Expected: FAIL — `ancla_esp32s3` is not a known board yet (west/CMake reports it can't find the board).

- [ ] **Step 2: Add `BOARD_ROOT` so this repo's `boards/` directory is discoverable**

  Edit `CMakeLists.txt` to:
  ```cmake
  cmake_minimum_required(VERSION 3.20.0)

  list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

  find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

  project(ANCLA_ESP32S3)

  target_sources(app PRIVATE src/main.c)
  ```

- [ ] **Step 3: Create `boards/innovaforce/ancla_esp32s3/board.yml`**

  ```yaml
  board:
    name: ancla_esp32s3
    full_name: Innovaforce ANCLA ESP32-S3
    vendor: innovaforce
    socs:
    - name: esp32s3
  ```

- [ ] **Step 4: Create `boards/innovaforce/ancla_esp32s3/Kconfig`**

  ```
  # Copyright (c) 2026 Innovaforce
  # SPDX-License-Identifier: Apache-2.0

  config HEAP_MEM_POOL_ADD_SIZE_BOARD
  	int
  	default 4096 if BOARD_ANCLA_ESP32S3_ESP32S3_PROCPU
  	default 256 if BOARD_ANCLA_ESP32S3_ESP32S3_APPCPU
  ```

- [ ] **Step 5: Create `boards/innovaforce/ancla_esp32s3/Kconfig.ancla_esp32s3`**

  ```
  # Copyright (c) 2026 Innovaforce
  # SPDX-License-Identifier: Apache-2.0

  config BOARD_ANCLA_ESP32S3
  	select SOC_ESP32S3_WROOM_N8R2
  	select SOC_ESP32S3_PROCPU if BOARD_ANCLA_ESP32S3_ESP32S3_PROCPU
  	select SOC_ESP32S3_APPCPU if BOARD_ANCLA_ESP32S3_ESP32S3_APPCPU
  ```

- [ ] **Step 6: Create `boards/innovaforce/ancla_esp32s3/Kconfig.sysbuild`**

  ```
  # Copyright (c) 2026 Innovaforce
  # SPDX-License-Identifier: Apache-2.0

  choice BOOTLOADER
  	default BOOTLOADER_MCUBOOT
  endchoice

  choice BOOT_SIGNATURE_TYPE
  	default BOOT_SIGNATURE_TYPE_NONE
  endchoice
  ```

- [ ] **Step 7: Create `boards/innovaforce/ancla_esp32s3/board.cmake`**

  ```cmake
  # SPDX-License-Identifier: Apache-2.0

  if(NOT "${OPENOCD}" MATCHES "^${ESPRESSIF_TOOLCHAIN_PATH}/.*")
    set(OPENOCD OPENOCD-NOTFOUND)
  endif()
  find_program(OPENOCD openocd PATHS ${ESPRESSIF_TOOLCHAIN_PATH}/openocd-esp32/bin NO_DEFAULT_PATH)

  include(${ZEPHYR_BASE}/boards/common/esp32.board.cmake)
  include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
  ```

- [ ] **Step 8: Create `boards/innovaforce/ancla_esp32s3/ancla_esp32s3-pinctrl.dtsi`**

  ```dts
  /*
   * Copyright (c) 2026 Innovaforce
   *
   * SPDX-License-Identifier: Apache-2.0
   */

  #include <zephyr/dt-bindings/pinctrl/esp32s3-pinctrl.h>

  &pinctrl {
  	uart0_default: uart0_default {
  		group1 {
  			pinmux = <UART0_TX_GPIO43>;
  			output-high;
  		};

  		group2 {
  			pinmux = <UART0_RX_GPIO44>;
  			bias-pull-up;
  		};
  	};
  };
  ```

- [ ] **Step 9: Create `boards/innovaforce/ancla_esp32s3/ancla_esp32s3_procpu.dts`**

  ```dts
  /*
   * Copyright (c) 2026 Innovaforce
   *
   * SPDX-License-Identifier: Apache-2.0
   */
  /dts-v1/;

  #include <espressif/esp32s3/esp32s3_wroom_n8r2.dtsi>
  #include <espressif/partitions_0x0_amp.dtsi>
  #include "ancla_esp32s3-pinctrl.dtsi"

  / {
  	model = "Innovaforce ANCLA ESP32-S3 PROCPU";
  	compatible = "innovaforce,ancla-esp32s3";

  	aliases {
  		watchdog0 = &wdt0;
  		uart-0 = &uart0;
  	};

  	chosen {
  		zephyr,sram = &sram1;
  		zephyr,console = &usb_serial;
  		zephyr,shell-uart = &usb_serial;
  		zephyr,flash = &flash0;
  		zephyr,code-partition = &slot0_partition;
  		zephyr,bt-hci = &esp32_bt_hci;
  	};
  };

  &usb_serial {
  	status = "okay";
  };

  &uart0 {
  	status = "okay";
  	current-speed = <115200>;
  	pinctrl-0 = <&uart0_default>;
  	pinctrl-names = "default";
  };

  &gpio0 {
  	status = "okay";
  };

  &gpio1 {
  	status = "okay";
  };

  &wdt0 {
  	status = "okay";
  };

  &trng0 {
  	status = "okay";
  };

  &esp32_bt_hci {
  	status = "okay";
  };

  &wifi {
  	status = "okay";
  };
  ```

- [ ] **Step 10: Create `boards/innovaforce/ancla_esp32s3/ancla_esp32s3_procpu.yaml`**

  ```yaml
  identifier: ancla_esp32s3/esp32s3/procpu
  name: Innovaforce ANCLA ESP32-S3 PROCPU
  type: mcu
  arch: xtensa
  toolchain:
    - zephyr
  supported:
    - gpio
    - uart
    - watchdog
    - entropy
    - netif:wifi
  vendor: innovaforce
  ```

- [ ] **Step 11: Create `boards/innovaforce/ancla_esp32s3/ancla_esp32s3_procpu_defconfig`**

  ```
  # SPDX-License-Identifier: Apache-2.0

  CONFIG_CONSOLE=y
  CONFIG_SERIAL=y
  CONFIG_UART_CONSOLE=y
  CONFIG_GPIO=y
  CONFIG_CLOCK_CONTROL=y
  ```

- [ ] **Step 12: Build and verify**

  Run the build command from Global Constraints. Expected: build completes, ninja reaches 100%, `build/zephyr/zephyr.elf` exists. No `FATAL ERROR`.

- [ ] **Step 13: Commit**

  ```bash
  git add CMakeLists.txt boards/innovaforce/ancla_esp32s3/board.yml \
    boards/innovaforce/ancla_esp32s3/Kconfig boards/innovaforce/ancla_esp32s3/Kconfig.ancla_esp32s3 \
    boards/innovaforce/ancla_esp32s3/Kconfig.sysbuild boards/innovaforce/ancla_esp32s3/board.cmake \
    boards/innovaforce/ancla_esp32s3/ancla_esp32s3-pinctrl.dtsi \
    boards/innovaforce/ancla_esp32s3/ancla_esp32s3_procpu.dts \
    boards/innovaforce/ancla_esp32s3/ancla_esp32s3_procpu.yaml \
    boards/innovaforce/ancla_esp32s3/ancla_esp32s3_procpu_defconfig
  git commit -m "feat(board): scaffold ancla_esp32s3 procpu board (SoC, USB console, UART0)"
  ```

---

### Task 2: DW3220 SPI2 bus + `zephyr,user` control-line GPIOs (RESET, IRQ, WAKE_UP, MAX17048 ALERT)

**Files:**
- Modify: `boards/innovaforce/ancla_esp32s3/ancla_esp32s3-pinctrl.dtsi`
- Modify: `boards/innovaforce/ancla_esp32s3/ancla_esp32s3_procpu.dts`
- Modify: `boards/innovaforce/ancla_esp32s3/ancla_esp32s3_procpu.yaml`

**Interfaces:**
- Consumes: pinctrl file and `/ {...};` root block from Task 1.
- Produces: pinctrl group `spim2_default`; node `&spi2` (enabled); `zephyr,user` properties `dw3000-reset-gpios`, `dw3000-irq-gpios`, `dw3000-wakeup-gpios`, `max17048-alert-gpios`. Task 3/4 don't depend on these but share the same files.

- [ ] **Step 1: Add the `spim2_default` pinctrl group**

  In `ancla_esp32s3-pinctrl.dtsi`, add inside the `&pinctrl { ... };` block (after `uart0_default`):

  ```dts
  	spim2_default: spim2_default {
  		group1 {
  			pinmux = <SPIM2_MISO_GPIO13>,
  				 <SPIM2_SCLK_GPIO12>,
  				 <SPIM2_CSEL_GPIO10>;
  		};

  		group2 {
  			pinmux = <SPIM2_MOSI_GPIO11>;
  			output-low;
  		};
  	};
  ```

- [ ] **Step 2: Enable `&spi2` and add the `zephyr,user` GPIOs**

  In `ancla_esp32s3_procpu.dts`, add a `zephyr,user` child node inside the existing `/ { ... };` block (after the `chosen` block):

  ```dts
  	zephyr,user {
  		dw3000-reset-gpios    = <&gpio0 21 GPIO_ACTIVE_HIGH>;
  		dw3000-irq-gpios      = <&gpio1 10 GPIO_ACTIVE_HIGH>;
  		dw3000-wakeup-gpios   = <&gpio0 14 GPIO_ACTIVE_HIGH>;
  		max17048-alert-gpios  = <&gpio1 9 GPIO_ACTIVE_LOW>;
  	};
  ```

  Then add, at the end of the file (after the existing `&wifi { status = "okay"; };`):

  ```dts
  &spi2 {
  	#address-cells = <1>;
  	#size-cells = <0>;
  	status = "okay";
  	pinctrl-0 = <&spim2_default>;
  	pinctrl-names = "default";
  };
  ```

- [ ] **Step 3: Update the board's `.yaml` `supported` list**

  In `ancla_esp32s3_procpu.yaml`, add `spi` to the `supported:` list (alongside the existing `gpio`, `uart`, etc.).

- [ ] **Step 4: Build and verify**

  Run the build command from Global Constraints. Expected: clean build, no devicetree errors (in particular, no "undefined node label" for `gpio0`/`gpio1` indices — GPIO bank math from Global Constraints must be correct: RESET pin 21 stays under `&gpio0`, IRQ pin 42 and ALERT pin 41 map to `&gpio1` at index `pin - 32`, i.e. 10 and 9).

- [ ] **Step 5: Confirm the generated devicetree actually has these nodes**

  Run:
  ```bash
  grep -n "dw3000-reset-gpios\|dw3000-irq-gpios\|dw3000-wakeup-gpios\|max17048-alert-gpios\|spi@60024000" \
    /c/Users/JoseAntonioLaraPerez/Documents/zephyr_projects/esp32/ANCLA_ESP32S3/build/zephyr/zephyr.dts
  ```
  Expected: all five patterns found, and the `spi@60024000` node shows `status = "okay"`.

- [ ] **Step 6: Commit**

  ```bash
  git add boards/innovaforce/ancla_esp32s3/ancla_esp32s3-pinctrl.dtsi \
    boards/innovaforce/ancla_esp32s3/ancla_esp32s3_procpu.dts \
    boards/innovaforce/ancla_esp32s3/ancla_esp32s3_procpu.yaml
  git commit -m "feat(board): add DW3220 SPI2 bus and control-line GPIOs"
  ```

---

### Task 3: MAX17048 fuel gauge on I2C0

**Files:**
- Modify: `boards/innovaforce/ancla_esp32s3/ancla_esp32s3-pinctrl.dtsi`
- Modify: `boards/innovaforce/ancla_esp32s3/ancla_esp32s3_procpu.dts`
- Modify: `boards/innovaforce/ancla_esp32s3/ancla_esp32s3_procpu.yaml`

**Interfaces:**
- Consumes: pinctrl file and dts root block from Tasks 1-2.
- Produces: pinctrl group `i2c0_default`; node `&i2c0` (enabled) with child `max17048: max17048@36`; alias `i2c-0`.

- [ ] **Step 1: Add the `i2c0_default` pinctrl group**

  In `ancla_esp32s3-pinctrl.dtsi`, add inside `&pinctrl { ... };` (after `spim2_default`):

  ```dts
  	i2c0_default: i2c0_default {
  		group1 {
  			pinmux = <I2C0_SDA_GPIO40>,
  				 <I2C0_SCL_GPIO39>;
  			bias-pull-up;
  			drive-open-drain;
  			output-high;
  		};
  	};
  ```

- [ ] **Step 2: Add the `i2c-0` alias**

  In `ancla_esp32s3_procpu.dts`, edit the existing `aliases { ... };` block to:

  ```dts
  	aliases {
  		i2c-0 = &i2c0;
  		watchdog0 = &wdt0;
  		uart-0 = &uart0;
  	};
  ```

- [ ] **Step 3: Add the `&i2c0` node with the MAX17048 child**

  At the end of `ancla_esp32s3_procpu.dts` (after the `&spi2 { ... };` block added in Task 2):

  ```dts
  &i2c0 {
  	status = "okay";
  	clock-frequency = <I2C_BITRATE_STANDARD>;
  	pinctrl-0 = <&i2c0_default>;
  	pinctrl-names = "default";

  	max17048: max17048@36 {
  		compatible = "maxim,max17048";
  		reg = <0x36>;
  	};
  };
  ```

  (ALERT is intentionally not a property here — it's `max17048-alert-gpios` under `zephyr,user`, added in Task 2, since the in-tree `maxim,max17048` binding has no `alert-gpios` property.)

- [ ] **Step 4: Update the board's `.yaml` `supported` list**

  In `ancla_esp32s3_procpu.yaml`, add `i2c` to the `supported:` list.

- [ ] **Step 5: Build and verify**

  Run the build command from Global Constraints. Expected: clean build.

- [ ] **Step 6: Confirm the MAX17048 node is in the generated devicetree**

  Run:
  ```bash
  grep -n "maxim,max17048\|max17048@36\|i2c@60013000" \
    /c/Users/JoseAntonioLaraPerez/Documents/zephyr_projects/esp32/ANCLA_ESP32S3/build/zephyr/zephyr.dts
  ```
  Expected: all three patterns found, and `i2c@60013000` shows `status = "okay"`.

- [ ] **Step 7: Commit**

  ```bash
  git add boards/innovaforce/ancla_esp32s3/ancla_esp32s3-pinctrl.dtsi \
    boards/innovaforce/ancla_esp32s3/ancla_esp32s3_procpu.dts \
    boards/innovaforce/ancla_esp32s3/ancla_esp32s3_procpu.yaml
  git commit -m "feat(board): add MAX17048 fuel gauge on I2C0"
  ```

---

### Task 4: WS2812 RGB LED on I2S0

**Files:**
- Modify: `boards/innovaforce/ancla_esp32s3/ancla_esp32s3-pinctrl.dtsi`
- Modify: `boards/innovaforce/ancla_esp32s3/ancla_esp32s3_procpu.dts`
- Modify: `boards/innovaforce/ancla_esp32s3/ancla_esp32s3_procpu.yaml`

**Interfaces:**
- Consumes: pinctrl file and dts root block from Tasks 1-3.
- Produces: pinctrl group `i2s0_default`; nodes `&i2s0` (enabled, with child `rgb_led: ws2812@0`) and `&dma` (enabled); alias `rgb0`.

- [ ] **Step 1: Add the `i2s0_default` pinctrl group**

  In `ancla_esp32s3-pinctrl.dtsi`, add inside `&pinctrl { ... };` (after `i2c0_default`):

  ```dts
  	i2s0_default: i2s0_default {
  		group1 {
  			pinmux = <I2S0_MCLK_GPIO33>,
  				 <I2S0_O_WS_GPIO34>,
  				 <I2S0_O_BCK_GPIO35>,
  				 <I2S0_I_WS_GPIO36>,
  				 <I2S0_I_BCK_GPIO37>,
  				 <I2S0_O_SD_GPIO38>;
  			output-enable;
  		};

  		group2 {
  			pinmux = <I2S0_I_SD_GPIO47>;
  			input-enable;
  		};
  	};
  ```

  (Only `I2S0_O_SD_GPIO38` is a real, physically-wired pin — the LED's data line. The rest are spare, otherwise-unconnected GPIOs the I2S0 peripheral requires to be pinned regardless of whether they're used.)

- [ ] **Step 2: Add the `rgb0` alias**

  In `ancla_esp32s3_procpu.dts`, edit `aliases { ... };` to:

  ```dts
  	aliases {
  		i2c-0 = &i2c0;
  		watchdog0 = &wdt0;
  		uart-0 = &uart0;
  		rgb0 = &rgb_led;
  	};
  ```

- [ ] **Step 3: Add the `zephyr/dt-bindings/led/led.h` include**

  At the top of `ancla_esp32s3_procpu.dts`, add after the existing includes:

  ```dts
  #include <zephyr/dt-bindings/led/led.h>
  ```

- [ ] **Step 4: Add the `&i2s0` and `&dma` nodes**

  At the end of `ancla_esp32s3_procpu.dts` (after the `&i2c0 { ... };` block added in Task 3):

  ```dts
  &i2s0 {
  	status = "okay";
  	pinctrl-0 = <&i2s0_default>;
  	pinctrl-names = "default";
  	dmas = <&dma 3>;
  	dma-names = "tx";

  	rgb_led: ws2812@0 {
  		compatible = "worldsemi,ws2812-i2s";
  		reg = <0>;
  		chain-length = <1>;
  		color-mapping = <LED_COLOR_ID_GREEN LED_COLOR_ID_RED LED_COLOR_ID_BLUE>;
  		reset-delay = <500>;
  	};
  };

  &dma {
  	status = "okay";
  };
  ```

- [ ] **Step 5: Update the board's `.yaml` `supported` list**

  In `ancla_esp32s3_procpu.yaml`, add `dma` and `led_strip` to the `supported:` list.

- [ ] **Step 6: Build and verify**

  Run the build command from Global Constraints. Expected: clean build.

- [ ] **Step 7: Confirm the LED node is in the generated devicetree**

  Run:
  ```bash
  grep -n "worldsemi,ws2812-i2s\|ws2812@0\|i2s@6000f000" \
    /c/Users/JoseAntonioLaraPerez/Documents/zephyr_projects/esp32/ANCLA_ESP32S3/build/zephyr/zephyr.dts
  ```
  Expected: all three patterns found, and `i2s@6000f000` shows `status = "okay"`.

- [ ] **Step 8: Commit**

  ```bash
  git add boards/innovaforce/ancla_esp32s3/ancla_esp32s3-pinctrl.dtsi \
    boards/innovaforce/ancla_esp32s3/ancla_esp32s3_procpu.dts \
    boards/innovaforce/ancla_esp32s3/ancla_esp32s3_procpu.yaml
  git commit -m "feat(board): add WS2812 RGB LED over I2S0"
  ```

---

### Task 5: `appcpu` variant (parity only, unused by the app)

**Files:**
- Create: `boards/innovaforce/ancla_esp32s3/ancla_esp32s3_appcpu.dts`
- Create: `boards/innovaforce/ancla_esp32s3/ancla_esp32s3_appcpu.yaml`
- Create: `boards/innovaforce/ancla_esp32s3/ancla_esp32s3_appcpu_defconfig`

**Interfaces:**
- Consumes: `espressif/partitions_0x0_amp.dtsi` (already used by the procpu variant in Task 1) for `slot0_appcpu_partition`.
- Produces: buildable target `ancla_esp32s3/esp32s3/appcpu`. Nothing later depends on this — it exists only for parity with every other ESP32-S3 board in this Zephyr tree, per decision 9 of the design spec.

- [ ] **Step 1: Create `boards/innovaforce/ancla_esp32s3/ancla_esp32s3_appcpu.dts`**

  ```dts
  /*
   * Copyright (c) 2026 Innovaforce
   *
   * SPDX-License-Identifier: Apache-2.0
   */
  /dts-v1/;

  #include <espressif/esp32s3/esp32s3_wroom_n8r2.dtsi>
  #include <espressif/partitions_0x0_amp.dtsi>

  / {
  	model = "Innovaforce ANCLA ESP32-S3 APPCPU";
  	compatible = "innovaforce,ancla-esp32s3";

  	chosen {
  		zephyr,sram = &sram1;
  		zephyr,ipc_shm = &shm0;
  		zephyr,ipc = &ipm0;
  		zephyr,flash = &flash0;
  		zephyr,code-partition = &slot0_appcpu_partition;
  	};
  };

  &trng0 {
  	status = "okay";
  };

  &ipm0 {
  	status = "okay";
  };
  ```

- [ ] **Step 2: Create `boards/innovaforce/ancla_esp32s3/ancla_esp32s3_appcpu.yaml`**

  ```yaml
  identifier: ancla_esp32s3/esp32s3/appcpu
  name: Innovaforce ANCLA ESP32-S3 APPCPU
  type: mcu
  arch: xtensa
  toolchain:
    - zephyr
  supported:
    - uart
  vendor: innovaforce
  ```

- [ ] **Step 3: Create `boards/innovaforce/ancla_esp32s3/ancla_esp32s3_appcpu_defconfig`**

  ```
  # SPDX-License-Identifier: Apache-2.0

  CONFIG_CLOCK_CONTROL=y
  ```

- [ ] **Step 4: Build and verify the appcpu target**

  Run:
  ```bash
  source /c/Users/JoseAntonioLaraPerez/zephyrproject/.venv/Scripts/activate
  cd /c/Users/JoseAntonioLaraPerez/zephyrproject
  west build -p always -b ancla_esp32s3/esp32s3/appcpu \
    -s /c/Users/JoseAntonioLaraPerez/Documents/zephyr_projects/esp32/ANCLA_ESP32S3 \
    -d /c/Users/JoseAntonioLaraPerez/Documents/zephyr_projects/esp32/ANCLA_ESP32S3/build_appcpu
  ```
  Expected: clean build, no `FATAL ERROR`.

- [ ] **Step 5: Re-verify the procpu target still builds clean**

  Run the procpu build command from Global Constraints one more time, to confirm adding the appcpu files didn't disturb board discovery for procpu (e.g. duplicate identifiers, board.yml conflicts).

- [ ] **Step 6: Commit**

  ```bash
  git add boards/innovaforce/ancla_esp32s3/ancla_esp32s3_appcpu.dts \
    boards/innovaforce/ancla_esp32s3/ancla_esp32s3_appcpu.yaml \
    boards/innovaforce/ancla_esp32s3/ancla_esp32s3_appcpu_defconfig
  git commit -m "feat(board): add appcpu variant for parity with other ESP32-S3 boards"
  ```

---

## After this plan

The board builds and the devicetree is verified by content inspection, but nothing here has touched real hardware. Before writing any application logic against this board:

1. Flash `build/zephyr/zephyr.elf` (or the appropriate `.bin`) to the actual PCB and confirm the USB-C serial monitor shows boot output.
2. Confirm the MAX17048, DW3220, and WS2812 LED are electrically wired exactly as described in the spec's hardware table — this plan cannot verify physical wiring, only that the devicetree/Kconfig describing it is internally consistent and builds.
3. Writing a DW3220 Zephyr driver/binding is explicitly out of scope here (see the design spec) and is a separate future plan.
