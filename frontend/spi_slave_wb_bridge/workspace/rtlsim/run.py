# ----------------------------------------------------------------------
# Copyright (c) 2026 VIRTUS-UFCG. All rights reserved
# VIRTUS-UFCG Confidential Proprietary
# ----------------------------------------------------------------------
import os
import sys

# Block Name
block = "spi_wb_bridge"
# Folders for the files
src_dir = "../../rtl/src"
tb_dir = "../../rtl/tb"

# Library creation and mapping in ModelSim
def setup_library(block):
    print(f"[INFO] Creating library {block}...")
    os.system(f"vlib {block}")
    os.system(f"vmap work {block}")

# Compilation of RTL files (src)
def compile_rtl(src_dir):
    print(f"[INFO] Compiling RTL files in folder {src_dir}...")
    if os.path.exists(src_dir):
        os.system(f"vlog -sv {src_dir}/*.sv")
    else:
        print(f"[ERROR] Folder {src_dir} does not exist.")
        exit(1)

# Removal of old files
def cleanup(block):
    for ext in ["vvp", "vcd"]:
        file = f"{block}_tb.{ext}"
        if os.path.exists(file):
            print(f"[INFO] Removing old file: {file}")
            os.remove(file)

# Compilation of the testbench (tb)
def compile_testbench(tb_dir, block):
    print(f"[INFO] Compiling the testbench in folder {tb_dir}...")
    testbench_file = os.path.join(tb_dir, f"{block}_tb.sv")
    if os.path.exists(testbench_file):
        os.system(f"vlog -sv {testbench_file}")
    else:
        print(f"[ERROR] File {testbench_file} does not exist.")
        exit(1)

# Running the simulation (console mode) with VCD generation
def run_simulation_console(block, crc_header_en=0):
    print(f"[INFO] Running simulation in console mode (CRC_HEADER_EN={crc_header_en})...")
    tcl_cmd = f"run -all; quit"
    os.system(f'vsim -c -voptargs="+acc" "-GCRC_HEADER_EN={crc_header_en}" -do "{tcl_cmd}" {block}_tb')

# Running the simulation with graphical interface
def run_simulation_gui(block):
    print(f"[INFO] Running simulation with graphical interface...")
    os.system(f"vsim")

# Main flow
def main():
    print("[INFO] Starting the simulation flow...")
    setup_library(block)
    compile_rtl(src_dir)
    cleanup(block)
    compile_testbench(tb_dir, block)
    if "0" in sys.argv:
        run_simulation_console(block, 0)
    elif "1" in sys.argv:
        run_simulation_console(block, 1)
    else:
        print("\n================== EXECUTANDO MODO 0: CRC_HEADER_EN=0 (Opcao B) ==================")
        run_simulation_console(block, 0)
        print("\n================== EXECUTANDO MODO 1: CRC_HEADER_EN=1 (Opcao A) ==================")
        run_simulation_console(block, 1)
    if "--gui" in sys.argv:
        run_simulation_gui(block)
    print("\n[INFO] Flow completed successfully!")

if __name__ == "__main__":
    main()
