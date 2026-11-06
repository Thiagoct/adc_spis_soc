// ----------------------------------------------------------------------
// Copyright (c) 2026 VIRTUS-UFCG. All rights reserved
// VIRTUS-UFCG Confidential Proprietary
// ----------------------------------------------------------------------
// Module     : spi_wb_bridge
// Description: Topo Digital da Ponte SPI Slave para Wishbone Master.
//              Permite que um mestre SPI externo (até 1 MHz) acesse via
//              barramento Wishbone (pelo menos 100 MHz) o banco de
//              registradores de 16 bits de um ADC SAR de 10 bits.
// ----------------------------------------------------------------------

module spi_wb_bridge #(
    parameter int ADDR_WIDTH    = 12,
    parameter int DATA_WIDTH    = 16,
    parameter int SEL_WIDTH     = DATA_WIDTH / 8,
    parameter bit CRC_HEADER_EN = 1'b0  // 0: Opção B (Seção 6.3.4), 1: Opção A (Seção 6.3.3.14)
) (
    // Sinais de Clock e Reset
    input  logic                  clk,       // Clock principal do sistema e Wishbone (>= 100 MHz)
    input  logic                  sck,       // Clock serial do SPI (<= 1 MHz)
    input  logic                  reset_s,   // Reset do domínio SPI, ativo baixo (assert assíncrono, deassert síncrono)
    input  logic                  rst_sys,   // Reset do domínio do sistema, ativo baixo (assert assíncrono, deassert síncrono)

    // Sinais de Configuração (Síncronos a clk)
    input  logic                  cpol,      // Polaridade do sck em estado ocioso
    input  logic                  cpha,      // Fase ativa de captura e transição do sck
    input  logic                  bit_order, // Ordem dos bits (0: LSB primeiro; 1: MSB primeiro)

    // Sinais da Transação SPI
    input  logic                  ss,        // Slave Select, ativo em nível baixo
    input  logic                  mosi,      // Master-Out Slave-In
    output logic                  miso,      // Master-In Slave-Out (domínio clk)
    output logic                  spi_busy,  // Flag ativa em nível baixo (0 = transação em andamento)

    // Interface Wishbone Master (Síncrona a clk)
    output logic [ADDR_WIDTH-1:0] wb_adr_o,
    output logic [DATA_WIDTH-1:0] wb_dat_o,
    input  logic [DATA_WIDTH-1:0] wb_dat_i,
    output logic [SEL_WIDTH-1:0]  wb_sel_o,
    output logic                  wb_we_o,
    output logic                  wb_stb_o,
    output logic                  wb_cyc_o,
    input  logic                  wb_ack_i,

    // Feedback de validação de CRC
    output logic                  crc_err
);

    // Sinais internos de reset sincronizados
    logic rst_s_sync_n;
    logic rst_sys_sync_n;

    // Sinais de interconexão entre SPI Slave Core e CRC8-CCITT
    logic [15:0] w_crc_tx_data;
    logic [7:0]  w_crc_tx_val;
    logic [15:0] w_crc_rx_data;
    logic [7:0]  w_crc_rx_val;
    logic        w_crc_rx_data_ok;
    logic [12:0] w_crc_rx_hdr;
    logic [7:0]  w_crc_rx_hdr_val;
    logic        w_crc_rx_hdr_ok;

    // Sinais de interconexão entre SPI Slave Core e Wishbone Master Controller
    logic                  w_cmd_load_addr;
    logic [ADDR_WIDTH-1:0] w_init_addr;
    logic                  w_wb_wr_req;
    logic [DATA_WIDTH-1:0] w_wb_wr_data;
    logic                  w_wb_rd_req;
    logic                  w_wb_addr_skip;
    logic                  w_wb_rd_valid;
    logic [DATA_WIDTH-1:0] w_wb_rd_data;
    logic                  w_wb_busy;

    // ------------------------------------------------------------------
    // 1. Sincronizadores de Reset (Ativação Assíncrona, Desativação Síncrona)
    // ------------------------------------------------------------------
    spi_wb_reset_sync u_reset_sync_spi (
        .clk         (clk),
        .rst_n_async (reset_s),
        .rst_n_sync  (rst_s_sync_n)
    );

    spi_wb_reset_sync u_reset_sync_sys (
        .clk         (clk),
        .rst_n_async (rst_sys),
        .rst_n_sync  (rst_sys_sync_n)
    );

    // ------------------------------------------------------------------
    // 2. Unidade de Cálculo e Verificação CRC8-CCITT
    // ------------------------------------------------------------------
    spi_crc8_ccitt u_spi_crc8_ccitt (
        .i_tx_data          (w_crc_tx_data),
        .o_tx_crc           (w_crc_tx_val),
        .i_rx_data          (w_crc_rx_data),
        .i_rx_data_crc      (w_crc_rx_val),
        .o_rx_data_calc_crc (),
        .o_rx_data_crc_ok   (w_crc_rx_data_ok),
        .i_rx_hdr           (w_crc_rx_hdr),
        .i_rx_hdr_crc       (w_crc_rx_hdr_val),
        .o_rx_hdr_calc_crc  (),
        .o_rx_hdr_crc_ok    (w_crc_rx_hdr_ok)
    );

    // ------------------------------------------------------------------
    // 3. Núcleo SPI Slave
    // ------------------------------------------------------------------
    spi_slave_core #(
        .CRC_HEADER_EN (CRC_HEADER_EN)
    ) u_spi_slave_core (
        .clk              (clk),
        .rst_s_n          (rst_s_sync_n),
        .cpol             (cpol),
        .cpha             (cpha),
        .bit_order        (bit_order),
        .sck              (sck),
        .ss               (ss),
        .mosi             (mosi),
        .miso             (miso),
        .spi_busy         (spi_busy),
        .o_crc_tx_data    (w_crc_tx_data),
        .i_crc_tx_val     (w_crc_tx_val),
        .o_crc_rx_data    (w_crc_rx_data),
        .o_crc_rx_val     (w_crc_rx_val),
        .i_crc_rx_data_ok (w_crc_rx_data_ok),
        .o_crc_rx_hdr     (w_crc_rx_hdr),
        .o_crc_rx_hdr_val (w_crc_rx_hdr_val),
        .i_crc_rx_hdr_ok  (w_crc_rx_hdr_ok),
        .o_cmd_load_addr  (w_cmd_load_addr),
        .o_init_addr      (w_init_addr),
        .o_wb_wr_req      (w_wb_wr_req),
        .o_wb_wr_data     (w_wb_wr_data),
        .o_wb_rd_req      (w_wb_rd_req),
        .o_wb_addr_skip   (w_wb_addr_skip),
        .i_wb_rd_valid    (w_wb_rd_valid),
        .i_wb_rd_data     (w_wb_rd_data),
        .i_wb_busy        (w_wb_busy),
        .o_crc_err        (crc_err)
    );

    // ------------------------------------------------------------------
    // 4. Controlador Wishbone Master
    // ------------------------------------------------------------------
    wb_master_ctrl #(
        .ADDR_WIDTH (ADDR_WIDTH),
        .DATA_WIDTH (DATA_WIDTH),
        .SEL_WIDTH  (SEL_WIDTH)
    ) u_wb_master_ctrl (
        .clk             (clk),
        .rst_sys_n       (rst_sys_sync_n),
        .i_cmd_load_addr (w_cmd_load_addr),
        .i_init_addr     (w_init_addr),
        .i_wr_req        (w_wb_wr_req),
        .i_wr_data       (w_wb_wr_data),
        .i_rd_req        (w_wb_rd_req),
        .i_addr_skip     (w_wb_addr_skip),
        .o_rd_valid      (w_wb_rd_valid),
        .o_rd_data       (w_wb_rd_data),
        .o_wb_busy       (w_wb_busy),
        .wb_adr_o        (wb_adr_o),
        .wb_dat_o        (wb_dat_o),
        .wb_dat_i        (wb_dat_i),
        .wb_sel_o        (wb_sel_o),
        .wb_we_o         (wb_we_o),
        .wb_stb_o        (wb_stb_o),
        .wb_cyc_o        (wb_cyc_o),
        .wb_ack_i        (wb_ack_i)
    );

endmodule

