# ----------------------------------------------------------------------
# Copyright (c) 2026 VIRTUS-UFCG. All rights reserved
# VIRTUS-UFCG Confidential Proprietary
# ----------------------------------------------------------------------
# File       : adc_spis_soc.py
# Description: Integração LiteX SoC do topo geral adc_spi_top (Litex/src), que reúne
#              o sub-bloco 2 (frontend/spi_slave_wb_bridge: SPI Slave -> Wishbone
#              Master + CRC8) e o sub-bloco 1 (frontend/adc_wb_slave_regbank:
#              Wishbone Slave + Banco de Registos + ADC SAR 10b).
# ----------------------------------------------------------------------

import os
import sys
import argparse

from migen import *
from litex.soc.interconnect import wishbone
from litex.soc.interconnect.csr import *

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

from litescope import LiteScopeAnalyzer

# Caminhos dos sub-blocos (frontend) e do topo geral (Litex/src)
THIS_DIR       = os.path.dirname(os.path.abspath(__file__))
FRONTEND_DIR   = os.path.abspath(os.path.join(THIS_DIR, "..", "frontend"))
ADC_RTL_DIR    = os.path.join(FRONTEND_DIR, "adc_wb_slave_regbank", "rtl", "src")   # Sub-bloco 1
BRIDGE_RTL_DIR = os.path.join(FRONTEND_DIR, "spi_slave_wb_bridge", "rtl", "src")    # Sub-bloco 2
TOP_RTL_DIR    = os.path.join(THIS_DIR, "src")                                      # Topo geral

# AdcSpiTop (Topo geral: spi_wb_bridge + adc_wb_slave_top) -----------------------------------------
class AdcSpiTop(Module):
    def __init__(self, platform, spi_pads, crc_header_en=1): 
        rst_sys_n = Signal()
        self.comb += rst_sys_n.eq(~ResetSignal("sys"))

        self.specials += Instance("adc_spi_top",
            p_CRC_HEADER_EN = crc_header_en,

            i_clk       = ClockSignal("sys"),
            i_sck       = spi_pads.sck,
            i_reset_s   = spi_pads.reset_s,
            i_rst_sys   = rst_sys_n,

            # Configuração SPI (Secção 5.1)
            i_cpol      = spi_pads.cpol,
            i_cpha      = spi_pads.cpha,
            i_bit_order = spi_pads.bit_order,

            # Transação SPI Slave 
            i_ss        = spi_pads.ss,
            i_mosi      = spi_pads.mosi,
            o_miso      = spi_pads.miso,
            o_spi_busy  = spi_pads.spi_busy,
        )

        # Fontes RTL: sub-bloco 1, sub-bloco 2 e topo 
        seen = set()
        for d in (ADC_RTL_DIR, BRIDGE_RTL_DIR, TOP_RTL_DIR):
            for f in sorted(os.listdir(d)):
                if f.endswith(".sv") and f not in seen:
                    seen.add(f)
                    platform.add_source(os.path.join(d, f), language="systemverilog")


# SPIBitBang (Módulo CSR para controle dos sinais SPI pelo firmware da CPU) ------------------------
class SPIBitBang(Module, AutoCSR):
    def __init__(self, spi_pads):
        # [0]=sck, [1]=ss, [2]=mosi, [3]=reset_s, [4]=cpol, [5]=cpha, [6]=bit_order
        self.control = CSRStorage(8, reset=0x4A, name="control",
            description="Controle dos pinos SPI pelo firmware da CPU: [0]=sck, [1]=ss, [2]=mosi, [3]=reset_s, [4]=cpol, [5]=cpha, [6]=bit_order")
        # [0]=miso, [1]=spi_busy
        self.status  = CSRStatus(8, name="status",
            description="Status dos pinos SPI: [0]=miso, [1]=spi_busy")

        self.comb += [
            spi_pads.sck.eq(self.control.storage[0]),
            spi_pads.ss.eq(self.control.storage[1]),
            spi_pads.mosi.eq(self.control.storage[2]),
            spi_pads.reset_s.eq(self.control.storage[3]),
            spi_pads.cpol.eq(self.control.storage[4]),
            spi_pads.cpha.eq(self.control.storage[5]),
            spi_pads.bit_order.eq(self.control.storage[6]),
            self.status.status[0].eq(spi_pads.miso),
            self.status.status[1].eq(spi_pads.spi_busy),
        ]


# IOs ----------------------------------------------------------------------------------------------

_io = [
    # Clk / Rst.
    ("sys_clk", 0, Pins(1)),
    ("sys_rst", 0, Pins(1)),

    # Serial.
    ("serial", 0,
        Subsignal("source_valid", Pins(1)),
        Subsignal("source_ready", Pins(1)),
        Subsignal("source_data",  Pins(8)),

        Subsignal("sink_valid",   Pins(1)),
        Subsignal("sink_ready",   Pins(1)),
        Subsignal("sink_data",    Pins(8)),
    ),

    # SPI Slave External Interface + Config Pins.
    ("spi_slave", 0,
        Subsignal("sck",       Pins(1)),
        Subsignal("ss",        Pins(1)),
        Subsignal("mosi",      Pins(1)),
        Subsignal("miso",      Pins(1)),
        Subsignal("spi_busy",  Pins(1)),
        Subsignal("reset_s",   Pins(1)),
        Subsignal("cpol",      Pins(1)),
        Subsignal("cpha",      Pins(1)),
        Subsignal("bit_order", Pins(1)),
    ),
]

# Platform -----------------------------------------------------------------------------------------

class Platform(SimPlatform):
    def __init__(self):
        SimPlatform.__init__(self, "SIM", _io)

# Simulation SoC -----------------------------------------------------------------------------------

class SimSoC(SoCCore):
    def __init__(self,
        with_sdram             = False,
        with_sdram_bist        = False,
        with_analyzer          = False,
        sdram_module           = "MT48LC16M16",
        sdram_init             = [],
        sdram_data_width       = 32,
        sdram_spd_data         = None,
        sdram_verbosity        = 0,
        sim_debug              = False,
        trace_reset_on         = False,
        with_adc_spis          = True,  # Habilita a integração SPI Slave WB Master + ADC WB Slave
        crc_header_en          = 1,     # 1: Opção A (Sec. 6.3.3.14); 0: Opção B (Sec. 6.3.4)
        external_spi           = False, # True: pinos externos em sim.v; False: conectado a CSRs para a CPU
        **kwargs):

        # Platform ---------------------------------------------------------------------------------
        platform = Platform()

        # Parameters -------------------------------------------------------------------------------
        sys_clk_freq = int(100e6) # 100 MHz conforme requisito da especificação

        # CRG com sys_clk e sys_rst conectados -----------------------------------------------------
        self.crg = CRG(platform.request("sys_clk"), platform.request("sys_rst"))

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq,
            ident = "LiteX Simulation - SPI Slave WB Master to ADC RegBank WB Slave",
            **kwargs)

        # Integração: Topo geral adc_spi_top (spi_wb_bridge + adc_wb_slave_top) ---------------------
        if with_adc_spis:
            if external_spi:
                spi_pads = platform.request("spi_slave")
            else:
                class SPIPads:
                    def __init__(self):
                        self.sck       = Signal(reset=0)
                        self.ss        = Signal(reset=1)
                        self.mosi      = Signal(reset=0)
                        self.miso      = Signal()
                        self.spi_busy  = Signal(reset=1)
                        self.reset_s   = Signal(reset=1)
                        self.cpol      = Signal(reset=0)
                        self.cpha      = Signal(reset=0)
                        self.bit_order = Signal(reset=1)
                spi_pads = SPIPads()
                self.submodules.spi_bitbang = SPIBitBang(spi_pads)

            self.submodules.adc_spi = AdcSpiTop(platform=platform, spi_pads=spi_pads, crc_header_en=crc_header_en)

        # SDRAM ------------------------------------------------------------------------------------
        if not self.integrated_main_ram_size and with_sdram:
            sdram_clk_freq = int(100e6)
            if sdram_spd_data is None:
                sdram_module_cls = getattr(litedram_modules, sdram_module)
                sdram_rate       = "1:{}".format(sdram_module_nphases[sdram_module_cls.memtype])
                sdram_module     = sdram_module_cls(sdram_clk_freq, sdram_rate)
            else:
                sdram_module = litedram_modules.SDRAMModule.from_spd_data(sdram_spd_data, sdram_clk_freq)
            self.sdrphy = SDRAMPHYModel(
                module     = sdram_module,
                data_width = sdram_data_width,
                clk_freq   = sdram_clk_freq,
                verbosity  = sdram_verbosity,
                init       = sdram_init)
            self.add_sdram("sdram",
                phy                     = self.sdrphy,
                module                  = sdram_module,
                l2_cache_size           = kwargs.get("l2_size", 8192),
                l2_cache_min_data_width = kwargs.get("min_l2_data_width", 128),
                l2_cache_reverse        = False,
                with_bist               = with_sdram_bist
            )
            if sdram_init != []:
                self.add_constant("SDRAM_TEST_DISABLE")
            else:
                self.add_constant("MEMTEST_DATA_SIZE", 8*1024)
                self.add_constant("MEMTEST_ADDR_SIZE", 8*1024)

        # Simulation debugging ----------------------------------------------------------------------
        if sim_debug:
            platform.add_debug(self, reset=1 if trace_reset_on else 0)
        else:
            self.comb += platform.trace.eq(1)

        # Analyzer ---------------------------------------------------------------------------------
        if with_analyzer:
            analyzer_signals = [
                self.cpu.ibus.stb,
                self.cpu.ibus.cyc,
                self.cpu.ibus.adr,
                self.cpu.ibus.we,
                self.cpu.ibus.ack,
                self.cpu.ibus.sel,
                self.cpu.ibus.dat_w,
                self.cpu.ibus.dat_r,
                self.cpu.dbus.stb,
                self.cpu.dbus.cyc,
                self.cpu.dbus.adr,
                self.cpu.dbus.we,
                self.cpu.dbus.ack,
                self.cpu.dbus.sel,
                self.cpu.dbus.dat_w,
                self.cpu.dbus.dat_r,
            ]
            self.analyzer = LiteScopeAnalyzer(analyzer_signals,
                depth        = 512,
                clock_domain = "sys",
                csr_csv      = "analyzer.csv")

# Build --------------------------------------------------------------------------------------------

def generate_gtkw_savefile(builder, vns, trace_fst):
    from litex.build.sim import gtkwave as gtkw
    dumpfile = os.path.join(builder.gateware_dir, "sim.{}".format("fst" if trace_fst else "vcd"))
    savefile = os.path.join(builder.gateware_dir, "sim.gtkw")
    soc = builder.soc

    with gtkw.GTKWSave(vns, savefile=savefile, dumpfile=dumpfile) as save:
        save.clocks()
        save.fsm_states(soc)
        if "main_ram" in soc.bus.slaves.keys():
            save.add(soc.bus.slaves["main_ram"], mappers=[gtkw.wishbone_sorter(), gtkw.wishbone_colorer()])

def sim_args(parser):
    # ROM / RAM.
    parser.add_argument("--rom-init",             default=None,            help="ROM init file (.bin or .json).")
    parser.add_argument("--ram-init",             default=None,            help="RAM init file (.bin or .json).")

    # DRAM.
    parser.add_argument("--with-sdram",           action="store_true",     help="Enable SDRAM support.")
    parser.add_argument("--with-sdram-bist",      action="store_true",     help="Enable SDRAM BIST Generator/Checker modules.")
    parser.add_argument("--sdram-module",         default="MT48LC16M16",   help="Select SDRAM chip.")
    parser.add_argument("--sdram-data-width",     default=32,              help="Set SDRAM chip data width.")
    parser.add_argument("--sdram-init",           default=None,            help="SDRAM init file (.bin or .json).")
    parser.add_argument("--sdram-from-spd-dump",  default=None,            help="Generate SDRAM module based on data from SPD EEPROM dump.")
    parser.add_argument("--sdram-verbosity",      default=0,               help="Set SDRAM checker verbosity.")

    # Integração SPI Slave WB Master + ADC RegBank WB Slave.
    parser.add_argument("--with-adc-spis",        action="store_true",     default=True, help="Enable SPI Slave WB Master + ADC WB Slave integration.")
    parser.add_argument("--crc-header-en",        type=int, default=1,     choices=[0, 1], help="Enable 13-bit header CRC (1: Option A / Sec 6.3.3.14; 0: Option B / Sec 6.3.4).")
    parser.add_argument("--external-spi",         action="store_true",     help="Expose SPI slave pins as external pads on sim.v (default connects to CPU BitBang CSR).")

    # Debug/Waveform.
    parser.add_argument("--sim-debug",            action="store_true",     help="Add simulation debugging modules.")
    parser.add_argument("--gtkwave-savefile",     action="store_true",     help="Generate GTKWave savefile.")
    parser.add_argument("--non-interactive",      action="store_true",     help="Run simulation without user input.")

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(description="LiteX SoC Simulation utility (SPI Slave WB Master + ADC WB Slave)")
    parser.set_platform(SimPlatform)
    sim_args(parser)
    args = parser.parse_args()

    soc_kwargs = soc_core_argdict(args)

    sys_clk_freq = int(100e6)
    sim_config   = SimConfig()
    sim_config.add_clocker("sys_clk", freq_hz=sys_clk_freq)

    # UART.
    if soc_kwargs["uart_name"] == "serial":
        soc_kwargs["uart_name"] = "sim"
        sim_config.add_module("serial2console", "serial")

    # Create config SoC that will be used to prepare/configure real one.
    conf_soc = SimSoC(crc_header_en=args.crc_header_en, external_spi=args.external_spi, **soc_kwargs)

    # ROM.
    if args.rom_init:
        soc_kwargs["integrated_rom_init"] = get_mem_data(args.rom_init,
            data_width = conf_soc.bus.data_width,
            endianness = conf_soc.cpu.endianness
        )

    # RAM / SDRAM.
    ram_boot_address = None
    soc_kwargs["integrated_main_ram_size"] = args.integrated_main_ram_size
    if args.integrated_main_ram_size:
        if args.ram_init is not None:
            soc_kwargs["integrated_main_ram_init"] = get_mem_data(args.ram_init,
                data_width = conf_soc.bus.data_width,
                endianness = conf_soc.cpu.endianness,
                offset     = conf_soc.mem_map["main_ram"]
            )
            ram_boot_address = get_boot_address(args.ram_init)
    elif args.with_sdram:
        assert args.ram_init is None
        soc_kwargs["sdram_module"]     = args.sdram_module
        soc_kwargs["sdram_data_width"] = int(args.sdram_data_width)
        soc_kwargs["sdram_verbosity"]  = int(args.sdram_verbosity)
        if args.sdram_from_spd_dump:
            soc_kwargs["sdram_spd_data"] = parse_spd_hexdump(args.sdram_from_spd_dump)
        if args.sdram_init is not None:
            soc_kwargs["sdram_init"] = get_mem_data(args.sdram_init,
                data_width = conf_soc.bus.data_width,
                endianness = conf_soc.cpu.endianness,
                offset     = conf_soc.mem_map["main_ram"]
            )
            ram_boot_address = get_boot_address(args.sdram_init)

    # SoC ------------------------------------------------------------------------------------------
    soc = SimSoC(
        with_sdram      = args.with_sdram,
        with_sdram_bist = args.with_sdram_bist,
        sim_debug       = args.sim_debug,
        trace_reset_on  = int(float(args.trace_start)) > 0 or int(float(args.trace_end)) > 0,
        with_adc_spis   = args.with_adc_spis,
        crc_header_en   = args.crc_header_en,
        external_spi    = args.external_spi,
        **soc_kwargs)
    if ram_boot_address is not None:
        if ram_boot_address == 0:
            ram_boot_address = conf_soc.mem_map["main_ram"]
        soc.add_constant("ROM_BOOT_ADDRESS", ram_boot_address)

    # Build/Run ------------------------------------------------------------------------------------
    builder = Builder(soc, **parser.builder_argdict)
    builder.build(
        sim_config  = sim_config,
        interactive = not args.non_interactive,
        **parser.toolchain_argdict,
    )

if __name__ == "__main__":
    main()

