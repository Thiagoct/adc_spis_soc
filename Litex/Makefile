# ----------------------------------------------------------------------
# Copyright (c) 2026 VIRTUS-UFCG. All rights reserved
# VIRTUS-UFCG Confidential Proprietary
# ----------------------------------------------------------------------
# Makefile para o SoC do ADC SAR 10b + Ponte SPI/Wishbone no LiteX
# ----------------------------------------------------------------------

PYTHON ?= python3
SOC_SCRIPT = adc_spis_soc.py

# Parametros padrao do SoC
ROM_SIZE ?= 0x10000
RAM_SIZE ?= 0x10000
CRC_HEADER_EN ?= 1
DEMO_BIN ?= demo/demo.bin

.PHONY: help build_ng_sim build_gw_sim build_ng_sim_debug build_gw_sim_debug demo clean

help:
	@echo "Targets disponiveis para o ADC SAR 10b + SPI/WB Bridge SoC:"
	@echo "  make build_ng_sim        : Gera CSRs e arquivos de simulacao (sem compilar gateware)"
	@echo "  make demo                : Compila o firmware (helloc.c + main.c) em ./demo"
	@echo "  make build_gw_sim        : Compila gateware e executa simulacao com firmware (demo.bin)"
	@echo "  make build_ng_sim_debug  : Gera simulacao com suporte a debug/trace GTKWave"
	@echo "  make build_gw_sim_debug  : Executa simulacao com debug e geracao de formas de onda"
	@echo "  make clean               : Remove artefatos de build gerados"

build_ng_sim:
	ulimit -s unlimited 2>/dev/null || true
	$(PYTHON) $(SOC_SCRIPT) --with-adc-spis \
		--crc-header-en=$(CRC_HEADER_EN) \
		--integrated-rom-size      $(ROM_SIZE) \
		--integrated-main-ram-size $(RAM_SIZE) \
		--no-compile-gateware

build_gw_sim:
	ulimit -s unlimited 2>/dev/null || true
	$(PYTHON) $(SOC_SCRIPT) --with-adc-spis \
		--crc-header-en=$(CRC_HEADER_EN) \
		--integrated-rom-size      $(ROM_SIZE) \
		--integrated-main-ram-size $(RAM_SIZE) \
		--rom-init=$(DEMO_BIN)

build_ng_sim_debug:
	ulimit -s unlimited 2>/dev/null || true
	$(PYTHON) $(SOC_SCRIPT) --with-adc-spis --trace --gtkwave-savefile \
		--crc-header-en=$(CRC_HEADER_EN) \
		--integrated-rom-size      $(ROM_SIZE) \
		--integrated-main-ram-size $(RAM_SIZE) \
		--no-compile-gateware

build_gw_sim_debug:
	ulimit -s unlimited 2>/dev/null || true
	$(PYTHON) $(SOC_SCRIPT) --with-adc-spis --trace --gtkwave-savefile \
		--crc-header-en=$(CRC_HEADER_EN) \
		--integrated-rom-size      $(ROM_SIZE) \
		--integrated-main-ram-size $(RAM_SIZE) \
		--rom-init=$(DEMO_BIN)

demo:
	@if [ -d "demo" ]; then \
		cp -f helloc.c main.c demo/ 2>/dev/null || true; \
		$(MAKE) -C demo; \
	else \
		echo "Diretorio ./demo nao encontrado. Gere-o com: litex_bare_metal_demo --build-path=build/sim"; \
	fi

clean:
	rm -rf build/
	@if [ -d "demo" ]; then $(MAKE) clean -C demo; fi
