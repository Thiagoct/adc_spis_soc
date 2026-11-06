// ----------------------------------------------------------------------
// Copyright (c) 2026 VIRTUS-UFCG. All rights reserved
// VIRTUS-UFCG Confidential Proprietary
// ----------------------------------------------------------------------
// Module     : adc_spi_top
// Description: Topo geral do IP. Integra os dois sub-blocos:
//              - Sub-bloco 2: spi_wb_bridge    (SPI Slave -> Wishbone Master + CRC8-CCITT)
//                             frontend/spi_slave_wb_bridge
//              - Sub-bloco 1: adc_wb_slave_top (Wishbone Slave + Banco de Registos + ADC SAR 10b)
//                             frontend/adc_wb_slave_regbank
//              O barramento Wishbone (12b endereço / 16b dados) é interno ao top e
//              o crc_err da ponte alimenta i_err_ext (SPI_STATUS.ERR) do banco de registos.
// ----------------------------------------------------------------------

module adc_spi_top #(
    parameter int  ADDR_WIDTH    = 12,
    parameter int  DATA_WIDTH    = 16,
    parameter int  GRANULE       = 8,
    parameter int  SEL_WIDTH     = DATA_WIDTH / GRANULE,
    parameter int  REGISTER_NUM  = 15,
    parameter bit  CRC_HEADER_EN = 1'b0,      // 0: Opção B (6.3.4); 1: Opção A (6.3.3.14)
    parameter real V_FULL_SCALE  = 0.8,
    parameter int  RESOLUTION    = 10,
    parameter real T_CONV_NS     = 100000.0
) (
    // Clocks e Resets
    input  logic                  clk,        // >= 100 MHz
    input  logic                  sck,        // <= 1 MHz
    input  logic                  reset_s,    // reset do domínio SPI (ativo baixo)
    input  logic                  rst_sys,    // reset do sistema (ativo baixo)

    // Configuração SPI
    input  logic                  cpol,
    input  logic                  cpha,
    input  logic                  bit_order,  // 0: LSB primeiro; 1: MSB primeiro

    // Interface SPI Slave
    input  logic                  ss,
    input  logic                  mosi,
    output logic                  miso,
    output logic                  spi_busy,   // ativo baixo

    // Entrada analógica do ADC (modelo comportamental)
    input  real                   v_in = 0.0,

    // Saídas de configuração / monitorização do ADC
    output logic                  o_adc_reset,
    output logic                  o_adc_enable,
    output logic [5:0]            o_adc_mode,
    output logic [7:0]            o_sample_rate,
    output logic [15:0]           o_filter_cfg,
    output logic [7:0]            o_gain_cfg,
    output logic [9:0]            o_adc_dout,
    output logic                  o_adc_data_rdy,
    output logic                  o_crc_err
);

    // Barramento Wishbone interno entre os dois sub-blocos
    logic [ADDR_WIDTH-1:0] wb_adr;
    logic [DATA_WIDTH-1:0] wb_dat_m2s;
    logic [DATA_WIDTH-1:0] wb_dat_s2m;
    logic [SEL_WIDTH-1:0]  wb_sel;
    logic                  wb_we;
    logic                  wb_stb;
    logic                  wb_cyc;
    logic                  wb_ack;

    // Sub-bloco 2: SPI Slave -> Wishbone Master
    spi_wb_bridge #(
        .ADDR_WIDTH    (ADDR_WIDTH),
        .DATA_WIDTH    (DATA_WIDTH),
        .SEL_WIDTH     (SEL_WIDTH),
        .CRC_HEADER_EN (CRC_HEADER_EN)
    ) u_spi_wb_bridge (
        .clk       (clk),
        .sck       (sck),
        .reset_s   (reset_s),
        .rst_sys   (rst_sys),
        .cpol      (cpol),
        .cpha      (cpha),
        .bit_order (bit_order),
        .ss        (ss),
        .mosi      (mosi),
        .miso      (miso),
        .spi_busy  (spi_busy),
        .wb_adr_o  (wb_adr),
        .wb_dat_o  (wb_dat_m2s),
        .wb_dat_i  (wb_dat_s2m),
        .wb_sel_o  (wb_sel),
        .wb_we_o   (wb_we),
        .wb_stb_o  (wb_stb),
        .wb_cyc_o  (wb_cyc),
        .wb_ack_i  (wb_ack),
        .crc_err   (o_crc_err)
    );

    // Sub-bloco 1: Wishbone Slave + Banco de Registos + ADC SAR 10b
    adc_wb_slave_top #(
        .ADDR_WIDTH   (ADDR_WIDTH),
        .DATA_WIDTH   (DATA_WIDTH),
        .GRANULE      (GRANULE),
        .REGISTER_NUM (REGISTER_NUM),
        .SEL_WIDTH    (SEL_WIDTH),
        .V_FULL_SCALE (V_FULL_SCALE),
        .RESOLUTION   (RESOLUTION),
        .T_CONV_NS    (T_CONV_NS)
    ) u_adc_wb_slave_top (
        .clk             (clk),
        .rst_sys         (rst_sys),
        .wb_adr_i        (wb_adr),
        .wb_dat_i        (wb_dat_m2s),
        .wb_dat_o        (wb_dat_s2m),
        .wb_sel_i        (wb_sel),
        .wb_cyc_i        (wb_cyc),
        .wb_stb_i        (wb_stb),
        .wb_we_i         (wb_we),
        .wb_ack_o        (wb_ack),
        .v_in            (v_in),
        .i_err_ext       (o_crc_err),
        .o_adc_reset     (o_adc_reset),
        .o_adc_enable    (o_adc_enable),
        .o_adc_mode      (o_adc_mode),
        .o_sample_rate   (o_sample_rate),
        .o_filter_cfg    (o_filter_cfg),
        .o_gain_cfg      (o_gain_cfg),
        .o_filter_coeffs (),
        .o_adc_dout      (o_adc_dout),
        .o_adc_data_rdy  (o_adc_data_rdy)
    );

endmodule

