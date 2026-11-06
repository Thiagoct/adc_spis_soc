# ----------------------------------------------------------------------
# Copyright (c) 2024 VIRTUS-UFCG. All rights reserved
# VIRTUS-UFCG Confidential Proprietary
# ----------------------------------------------------------------------
from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *

from litex_boards.platforms import xilinx_vcu118 # <--- MODIFICADO: xilinx_vcu118

from litex.soc.interconnect import wishbone # <--- ADICIONADO: Import para o barramento Wishbone
from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.led import LedChaser

from litedram.modules import EDY4016A # <--- MODIFICADO: Memória da VCU118
from litedram.phy import usddrphy

from litepcie.phy.usppciephy import USPPCIEPHY
from litepcie.software import generate_litepcie_software

import sys
import argparse

from migen import *
from litex.soc.interconnect import wishbone

from litex.build.generic_platform import *
from litex.build.sim              import SimPlatform
from litex.build.sim.config       import SimConfig

from litex.soc.integration.common   import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder  import *
from litex.soc.integration.soc      import *

from litex.soc.cores.bitbang import *
from litex.soc.cores.gpio    import GPIOTristate
from litex.soc.cores.cpu     import CPUS
from litex.soc.cores.video   import VideoGenericPHY

from litedram           import modules as litedram_modules
from litedram.modules   import parse_spd_hexdump
from litedram.phy.model import sdram_module_nphases, get_sdram_phy_settings
from litedram.phy.model import SDRAMPHYModel

from liteeth.common             import *
from liteeth.phy.gmii           import LiteEthPHYGMII
from liteeth.phy.xgmii          import LiteEthPHYXGMII
from liteeth.phy.model          import LiteEthPHYModel
from liteeth.mac                import LiteEthMAC
from liteeth.core.arp           import LiteEthARP
from liteeth.core.ip            import LiteEthIP
from liteeth.core.udp           import LiteEthUDP
from liteeth.core.icmp          import LiteEthICMP
from liteeth.core               import LiteEthUDPIPCore
from liteeth.frontend.etherbone import LiteEthEtherbone

# PrototypeWBTop (Provided by user) -------------------------------------------------------------------
class PrototypeWBTop(Module):
    """Wrapper para o acelerador 'prototype_wb_top' escrito em SystemVerilog."""
    def __init__(self, platform, clk_signal, data_width=32, addr_width=16):
        self.bus = wishbone.Interface(
            data_width=data_width,
            adr_width=addr_width
        )
        self.err_o = Signal()

        # Converte o sinal de reset de ativo-baixo (padrão LiteX) para ativo-alto
        rst_active_high = Signal()
        self.comb += rst_active_high.eq(~ResetSignal())

        # Instancia o módulo SystemVerilog
        self.specials += Instance("prototype_wb_top",
            # Conecta explicitamente o clock e o reset
            i_clk_i=clk_signal,
            i_rst_i=rst_active_high,

            # Conecta a interface Wishbone
            i_adr_i=self.bus.adr,
            i_dat_i=self.bus.dat_w,
            o_dat_o=self.bus.dat_r,
            i_sel_i=self.bus.sel,
            i_we_i=self.bus.we,
            i_stb_i=self.bus.stb,
            i_cyc_i=self.bus.cyc,
            o_ack_o=self.bus.ack,
            o_err_o=self.err_o
        )

        # Adicionar arquivos fonte
        platform.add_source("src/prototype_wb_top.sv", language="systemverilog")
        platform.add_source("src/wb_slave.sv", language="systemverilog")
        platform.add_source("src/prototype_top.sv", language="systemverilog")
        platform.add_source("src/prototype.sv", language="systemverilog")
        platform.add_source("src/relu.sv", language="systemverilog")
        platform.add_source("src/dense.sv", language="systemverilog")
        platform.add_source("src/dense_datapath.sv", language="systemverilog")


# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst       = Signal()
        self.cd_sys    = ClockDomain()
        self.cd_sys4x  = ClockDomain()
        self.cd_pll4x  = ClockDomain()
        self.cd_idelay = ClockDomain()

        # # #

        # MODIFICADO: Adaptado para usar 'cpu_reset' e 'clk125' diretamente no PLL, padrão da VCU118
        self.pll = pll = USMMCM(speedgrade=-2)
        self.comb += pll.reset.eq(platform.request("cpu_reset") | self.rst) 
        pll.register_clkin(platform.request("clk125"), 125e6)
        pll.create_clkout(self.cd_pll4x, sys_clk_freq*4, buf=None, with_reset=False)
        pll.create_clkout(self.cd_idelay, 500e6)
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin) # Ignore sys_clk to pll.clkin path created by SoC's rst.

        self.specials += [
            Instance("BUFGCE_DIV",
                p_BUFGCE_DIVIDE=4,
                i_CE=1, i_I=self.cd_pll4x.clk, o_O=self.cd_sys.clk),
            Instance("BUFGCE",
                i_CE=1, i_I=self.cd_pll4x.clk, o_O=self.cd_sys4x.clk),
        ]

        self.idelayctrl = USIDELAYCTRL(cd_ref=self.cd_idelay, cd_sys=self.cd_sys)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    # MODIFICADO: Adicionado `with_prototype` como argumento
    def __init__(self, sys_clk_freq=125e6, with_led_chaser=True, with_pcie=False, with_prototype=False, **kwargs):
        platform = xilinx_vcu118.Platform() # <--- MODIFICADO: Instância xilinx_vcu118

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, sys_clk_freq, ident="LiteX SoC on VCU118 with prototype", **kwargs) # <--- MODIFICADO: String de ID

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # DDR4 SDRAM -------------------------------------------------------------------------------
        if not self.integrated_main_ram_size:
            self.ddrphy = usddrphy.USPDDRPHY(platform.request("ddram"),
                memtype          = "DDR4",
                sys_clk_freq     = sys_clk_freq,
                iodelay_clk_freq = 500e6)
            self.add_sdram("sdram",
                phy           = self.ddrphy,
                module        = EDY4016A(sys_clk_freq, "1:4"), # <--- MODIFICADO: EDY4016A
                size          = 0x40000000,                    # <--- MODIFICADO: 0x40000000
                l2_cache_size = kwargs.get("l2_size", 8192)
            )

        # PCIe -------------------------------------------------------------------------------------
        # if with_pcie:
        #     self.pcie_phy = USPPCIEPHY(platform, platform.request("pcie_x4"),
        #         speed      = "gen3",
        #         data_width = 128,
        #         bar0_size  = 0x20000)
        #     self.add_pcie(phy=self.pcie_phy, ndmas=1)

        # #########################################################################################
        # ADICIONADO: Integração condicional do PrototypeWBTop no SoC
        # #########################################################################################
        if with_prototype:
                    self.prototype = PrototypeWBTop(platform, clk_signal=self.crg.cd_sys.clk)
                    self.bus.add_slave(
                        name="prototype",
                        slave=self.prototype.bus,
                        region=SoCRegion(
                            origin=0x80000000,  # <-- ENDEREÇO CORRIGIDO (32-bit válido)
                            size=0x100,
                            cached=False
                        )
                    )

        # Leds -------------------------------------------------------------------------------------
        if with_led_chaser:
            self.leds = LedChaser(
                pads         = platform.request_all("user_led"),
                sys_clk_freq = sys_clk_freq)

# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=xilinx_vcu118.Platform, description="LiteX SoC on VCU118.") # <--- MODIFICADO: xilinx_vcu118.Platform
    parser.add_target_argument("--sys-clk-freq", default=125e6, type=float, help="System clock frequency.")
    parser.add_target_argument("--with-pcie",    action="store_true",       help="Enable PCIe support")
    # ADICIONADO: Argumento de linha de comando para habilitar o prototype
    parser.add_target_argument("--with-prototype",   action="store_true",       help="Enable PrototypeWBTop custom accelerator.")
    args = parser.parse_args()

    # MODIFICADO: Passa o novo argumento `with_prototype` para o construtor do SoC
    soc = BaseSoC(
        sys_clk_freq = args.sys_clk_freq,
        with_pcie    = args.with_pcie,
        with_prototype   = args.with_prototype,
        **parser.soc_argdict
    )
    
    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()