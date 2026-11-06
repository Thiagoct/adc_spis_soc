# ----------------------------------------------------------------------
# Copyright (c) 2024 VIRTUS-UFCG. All rights reserved
# VIRTUS-UFCG Confidential Proprietary
# ----------------------------------------------------------------------
import os
import shutil
from migen import *
from litex.build.generic_platform import GenericPlatform, Pins, Subsignal
from litex.soc.interconnect import wishbone
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.builder import Builder
import argparse

# ------------------------
# Minimal IO for ASIC
# ------------------------
# 2. Defina a UART como um conector "serial" com subsinais "tx" e "rx"
_io = [
    ("i_clk", 0, Pins(1)),
    ("sys_rst", 0, Pins(1)),
    ("serial", 0,
        Subsignal("tx", Pins(1)), # Pino de transmissão
        Subsignal("rx", Pins(1)), # Pino de recepção
    ),
]

# 1. Plataforma ASIC
class ASICPlatform(GenericPlatform):
    default_clk_name = "i_clk"
    default_clk_period = 1e9 / 100e6  # 10ns -> 100 MHz

    def __init__(self):
        GenericPlatform.__init__(self, "ASIC", io=_io)
        self.special_overrides = {}

    def build(self, soc_or_fragment, build_dir="build", build_name="asic_top", **kwargs):
        gateware_dir = os.path.join(build_dir)
        os.makedirs(gateware_dir, exist_ok=True)
        
        soc_or_fragment.platform.do_finalize(soc_or_fragment)
        
        v_output = soc_or_fragment.platform.get_verilog(soc_or_fragment, name=build_name, special_overrides=self.special_overrides)
        vns = v_output.ns

        verilog_file = os.path.join(gateware_dir, build_name + ".v")
        v_output.write(verilog_file)

        for f in self.sources:
            shutil.copy(f[0], gateware_dir)
        
        return vns

    def do_finalize(self, fragment):
        GenericPlatform.do_finalize(self, fragment)

# 2. CRG simplificado para ASIC
class ASIC_CRG(Module):
    def __init__(self, platform):
        self.cd_sys = ClockDomain("sys")
        self.clock_domains += [self.cd_sys]
        
        self.comb += self.cd_sys.clk.eq(platform.request("i_clk"))
        self.comb += self.cd_sys.rst.eq(platform.request("sys_rst"))

# 3. Wrapper do acelerador
class AdderWBTop(Module):
    def __init__(self, platform, data_width=32, addr_width=16):
        self.bus = wishbone.Interface(data_width=data_width, adr_width=addr_width)
        self.err_o = Signal()

        rst_active_high = Signal()
        self.comb += rst_active_high.eq(ResetSignal("sys"))

        self.specials += Instance("adder_wb_top",
            i_clk_i = ClockSignal("sys"),
            i_rst_i = rst_active_high,
            i_adr_i = self.bus.adr,
            i_dat_i = self.bus.dat_w,
            o_dat_o = self.bus.dat_r,
            i_sel_i = self.bus.sel,
            i_we_i  = self.bus.we,
            i_stb_i = self.bus.stb,
            i_cyc_i = self.bus.cyc,
            o_ack_o = self.bus.ack,
            o_err_o = self.err_o
        )

        files = [
            "src/adder_wb_top.sv", "src/wb_slave.sv", "src/adder_top.sv",
            "src/adder.sv", "src/adder1d.sv", "src/adder1d_datapath.sv",
            "src/batchnorm2d.sv", "src/bn2d_datapath.sv", "src/relu.sv",
            "src/sqrt.sv", "src/divider.sv", "src/dense.sv"
        ]
        for f in files:
            platform.add_source(f, language="systemverilog")

# 4. SRAM
class SRAM(Module):
    def __init__(self, size):
        self.bus = wishbone.Interface(data_width=32, adr_width=32)
        self.specials += Instance("SRAM_64KB",
            i_CLK = ClockSignal("sys"),
            i_CSn = ~self.bus.cyc,
            i_WEn = ~self.bus.we,
            i_A   = self.bus.adr,
            i_D   = self.bus.dat_w,
            o_Q   = self.bus.dat_r
        )

# 5. SoC principal
class ASICSoC(SoCCore):
    def __init__(self, sys_clk_freq=100e6, with_adder=True, **kwargs):
        platform = ASICPlatform()

        kwargs.update({
            "integrated_sram_size": 0,
            "integrated_rom_size": 0,
            "cpu_type": "vexriscv",
            "cpu_variant": "minimal",
            "with_uart": True,
            "uart_name": "serial",
            "ident": "LiteX ASIC SoC with Adder"
        })

        SoCCore.__init__(self, platform, sys_clk_freq, **kwargs)
        self.submodules.crg = ASIC_CRG(platform)

        # SRAM externa
        self.submodules.sram = SRAM(size=0x10000)
        self.bus.add_slave("sram", self.sram.bus,
                           region=SoCRegion(origin=0x00000000, size=0x10000))

        # Acelerador
        if with_adder:
            self.submodules.adder = AdderWBTop(platform)
            self.bus.add_slave("adder", self.adder.bus,
                               region=SoCRegion(origin=0x80000000, size=0x100, cached=False))

# 6. Fluxo de construção
def main():
    parser = argparse.ArgumentParser(description="ASIC SoC with Adder Accelerator")
    parser.add_argument("--build", action="store_true", help="Build design")
    parser.add_argument("--sys-clk-freq", default=100e6, type=float, help="System clock frequency")
    parser.add_argument("--with-adder", action="store_true", default=True, help="Enable Adder accelerator")
    args = parser.parse_args()

    soc = ASICSoC(sys_clk_freq=args.sys_clk_freq, with_adder=args.with_adder)

    builder = Builder(soc, csr_csv="csr.csv")

    if args.build:
        builder.build(build_name="asic_top", run=False)
        print("Build finalizado. Arquivos gerados em 'build/asic_top/gateware'.")

if __name__ == "__main__":
    main()