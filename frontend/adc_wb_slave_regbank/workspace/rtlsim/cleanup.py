# Copyright to Cognisea. All rights reserved.
# Author     : VIRTUS
# Description: script to clean simulation files

import os
import shutil
import platform

# Block Name
block = "adc_wb_slave_top"

# Extensions to be cleaned (except .txt, which will be handled with a specific command)
extensions_to_clean = ["vvp", "vcd"]

# Specific files to be removed
specific_files_to_remove = ["transcript", "modelsim.ini", "vsim.wlf"]

def cleanup_files(block, extensions, specific_files):
    print("[INFO] Starting cleanup of generated files...")

    for ext in extensions:
        file_pattern = f"{block}_tb.{ext}"
        if os.path.exists(file_pattern):
            print(f"[INFO] Removing file: {file_pattern}")
            os.remove(file_pattern)

    for file in specific_files:
        if os.path.exists(file):
            print(f"[INFO] Removing file: {file}")
            os.remove(file)

    print("[INFO] File cleanup completed!")

def cleanup_txt_files():
    print("[INFO] Removing all .txt files...")
    if platform.system() == "Windows":
        os.system("powershell -Command \"Remove-Item -Path *.txt -Force -ErrorAction SilentlyContinue\"")
    else:
        os.system("rm -f *.txt")
    print("[INFO] .txt files successfully removed!")

def cleanup_directory(directory):
    if os.path.exists(directory) and os.path.isdir(directory):
        print(f"[INFO] Removing folder and its contents: {directory}")
        shutil.rmtree(directory)
        print("[INFO] Folder successfully removed!")
    else:
        print(f"[INFO] Folder {directory} does not exist or has already been removed.")

def main():
    cleanup_files(block, extensions_to_clean, specific_files_to_remove)
    cleanup_txt_files()
    cleanup_directory(block)
    print("[INFO] Full cleanup completed!")

if __name__ == "__main__":
    main()

