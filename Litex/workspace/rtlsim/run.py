# ----------------------------------------------------------------------
# Copyright (c) 2026 VIRTUS-UFCG. All rights reserved
# VIRTUS-UFCG Confidential Proprietary
# ----------------------------------------------------------------------
# Simulação do topo geral (adc_spi_top): compila os dois sub-blocos
# (frontend/spi_slave_wb_bridge e frontend/adc_wb_slave_regbank) + topo.
import os
import glob
import sys

block = "adc_spi_top"

here = os.path.dirname(os.path.abspath(__file__))
root = os.path.abspath(os.path.join(here, "..", ".."))
frontend = os.path.abspath(os.path.join(root, "..", "frontend"))

# Ordem de compilação: sub-bloco 1, sub-bloco 2, topo
src_dirs = [
    os.path.join(frontend, "adc_wb_slave_regbank", "rtl", "src"),
    os.path.join(frontend, "spi_slave_wb_bridge", "rtl", "src"),
    os.path.join(root, "src"),
]
tb_file = os.path.join(root, "tb", f"{block}_tb.sv")

def setup_library(block):
    print(f"[INFO] Creating library {block}...")
    os.system(f"vlib {block}")
    os.system(f"vmap work {block}")

def collect_sources():
    files, seen = [], set()
    for d in src_dirs:
        if not os.path.isdir(d):
            print(f"[ERROR] Folder {d} does not exist.")
            exit(1)
        for f in sorted(glob.glob(os.path.join(d, "*.sv"))):
            name = os.path.basename(f)
            if name in seen:  # módulos compartilhados (ex.: spi_wb_reset_sync)
                continue
            seen.add(name)
            files.append(f)
    return files

def compile_rtl():
    files = collect_sources()
    print(f"[INFO] Compiling {len(files)} RTL files...")
    os.system("vlog -sv " + " ".join(f'"{f}"' for f in files))

def cleanup(block):
    for ext in ["vvp", "vcd"]:
        file = f"{block}_tb.{ext}"
        if os.path.exists(file):
            print(f"[INFO] Removing old file: {file}")
            os.remove(file)

def compile_testbench():
    print(f"[INFO] Compiling the testbench {tb_file}...")
    if not os.path.exists(tb_file):
        print(f"[ERROR] File {tb_file} does not exist.")
        exit(1)
    os.system(f'vlog -sv "{tb_file}"')

def run_simulation_console(block, crc_header_en=0):
    print(f"[INFO] Running simulation in console mode (CRC_HEADER_EN={crc_header_en})...")
    tcl_cmd = f"run -all; quit"
    os.system(f'vsim -c -voptargs="+acc" "-GCRC_HEADER_EN={crc_header_en}" -do "{tcl_cmd}" {block}_tb')

def main():
    print("[INFO] Starting the simulation flow...")
    setup_library(block)
    compile_rtl()
    cleanup(block)
    compile_testbench()
    if len(sys.argv) > 1 and sys.argv[1] in ["0", "1"]:
        run_simulation_console(block, int(sys.argv[1]))
    else:
        print("\n================== EXECUTANDO MODO 0: CRC_HEADER_EN=0 (Opcao B) ==================")
        run_simulation_console(block, 0)
        print("\n================== EXECUTANDO MODO 1: CRC_HEADER_EN=1 (Opcao A) ==================")
        run_simulation_console(block, 1)
    print("\n[INFO] Flow completed successfully!")

if __name__ == "__main__":
    main()
