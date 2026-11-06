// ----------------------------------------------------------------------
// Copyright (c) 2026 VIRTUS-UFCG. All rights reserved
// VIRTUS-UFCG Confidential Proprietary
// ----------------------------------------------------------------------
// Module     : adc_wb_slave_top
// Description: Topo do Banco de Registos (Wishbone Slave) conectado
//              diretamente ao RTL do núcleo do ADC SAR de 10 bits (adc).
//              Atua como Wishbone Slave (12 bits de endereço, 16 bits
//              de dados) operando em >= 100 MHz com sincronismo de reset
//              rst_sys (ativação assíncrona e desativação síncrona).
// ----------------------------------------------------------------------

module adc_wb_slave_top #(
    parameter int  ADDR_WIDTH   = 12,
    parameter int  DATA_WIDTH   = 16,
    parameter int  GRANULE      = 8,
    parameter int  REGISTER_NUM = 15,
    parameter int  SEL_WIDTH    = DATA_WIDTH / GRANULE,
    parameter real V_FULL_SCALE = 0.8,
    parameter int  RESOLUTION   = 10,
    parameter real T_CONV_NS    = 100000.0
) (
    // Sinais de Sistema
    input  logic                  clk,       // Relógio principal do sistema (>= 100 MHz)
    input  logic                  rst_sys,   // Reset dedicado do sistema (assert assíncrono, deassert síncrono)

    // Interface Wishbone Slave (Substituindo a interface APB)
    input  logic [ADDR_WIDTH-1:0] wb_adr_i,  // Endereço de 12 bits
    input  logic [DATA_WIDTH-1:0] wb_dat_i,  // Dados de escrita de 16 bits
    output logic [DATA_WIDTH-1:0] wb_dat_o,  // Dados de leitura de 16 bits
    input  logic [SEL_WIDTH-1:0]  wb_sel_i,  // Write strobes / seleção de bytes
    input  logic                  wb_cyc_i,  // Cycle valid
    input  logic                  wb_stb_i,  // Strobe
    input  logic                  wb_we_i,   // Write Enable (1 = Escrita, 0 = Leitura)
    output logic                  wb_ack_o,  // Acknowledge de conclusão de transação

    // Interface com o Núcleo do ADC e Sistema
    input  real                   v_in      = 0.0, // Entrada analógica para o núcleo do ADC SAR 10b
    input  logic                  i_err_ext = 1'b0,// Entrada de erro vindo da lógica de validação/CRC

    // Saídas de Configuração, Controlo e Monitorização (Registos <-> ADC)
    output logic                  o_adc_reset,          // Reset síncrono por software (ADC_CTRL[7])
    output logic                  o_adc_enable,         // Enable do ADC (ADC_CTRL[6])
    output logic [5:0]            o_adc_mode,           // Seleção de modo (ADC_CTRL[5:0])
    output logic [7:0]            o_sample_rate,        // Taxa de amostragem (SAMPLE_RATE[7:0])
    output logic [15:0]           o_filter_cfg,         // Configuração do filtro digital (FILTER_CFG[15:0])
    output logic [7:0]            o_gain_cfg,           // Controlo de ganho (GAIN_CFG[7:0])
    output logic [15:0]           o_filter_coeffs [0:9],// Coeficientes programáveis (0x06 a 0x0F)
    output logic [9:0]            o_adc_dout,           // Palavra de 10 bits convertida pelo ADC
    output logic                  o_adc_data_rdy        // Flag DATA_RDY gerada pelo ADC
);

    // ------------------------------------------------------------------
    // 1. Sincronizador de Reset (rst_sys: assert assíncrono, deassert síncrono)
    // ------------------------------------------------------------------
    logic rst_sys_sync_n;

    spi_wb_reset_sync u_rst_sys_sync (
        .clk         (clk),
        .rst_n_async (rst_sys),
        .rst_n_sync  (rst_sys_sync_n)
    );

    // ------------------------------------------------------------------
    // 2. Sinais Internos de Interconexão (wb_slave <-> adc_reg_bank <-> adc)
    // ------------------------------------------------------------------
    logic                  w_mem_valid;
    logic                  w_mem_we;
    logic [ADDR_WIDTH-1:0] w_mem_addr;
    logic [DATA_WIDTH-1:0] w_mem_data_w;
    logic [SEL_WIDTH-1:0]  w_mem_sel;
    logic [DATA_WIDTH-1:0] w_mem_data_r;
    logic                  w_mem_ready;
    logic                  w_wb_err_unused;

    logic [9:0]            w_adc_dout;
    logic                  w_adc_data_rdy;
    logic                  w_adc_rst_n;

    // O banco de registos responde em ciclo único
    assign w_mem_ready = 1'b1;

    // ------------------------------------------------------------------
    // 3. Instanciação do Controlador Wishbone Slave (wb_slave)
    // ------------------------------------------------------------------
    wb_slave #(
        .ADDR_WIDTH   (ADDR_WIDTH),
        .DATA_WIDTH   (DATA_WIDTH),
        .GRANULE      (GRANULE),
        .REGISTER_NUM (REGISTER_NUM),
        .SEL_WIDTH    (SEL_WIDTH)
    ) u_wb_slave (
        .rst_i        (rst_sys_sync_n),
        .i_clk        (clk),
        .adr_i        (wb_adr_i),
        .dat_i        (wb_dat_i),
        .dat_o        (wb_dat_o),
        .sel_i        (wb_sel_i),
        .we_i         (wb_we_i),
        .stb_i        (wb_stb_i),
        .ack_o        (wb_ack_o),
        .err_o        (w_wb_err_unused),
        .cyc_i        (wb_cyc_i),
        .o_mem_valid  (w_mem_valid),
        .o_mem_we     (w_mem_we),
        .o_mem_addr   (w_mem_addr),
        .o_mem_data_w (w_mem_data_w),
        .o_mem_sel    (w_mem_sel),
        .i_mem_data_r (w_mem_data_r),
        .i_mem_ready  (w_mem_ready)
    );

    // ------------------------------------------------------------------
    // 4. Instanciação do Banco de Registos (adc_reg_bank)
    // ------------------------------------------------------------------
    adc_reg_bank #(
        .ADDR_WIDTH (ADDR_WIDTH),
        .DATA_WIDTH (DATA_WIDTH),
        .SEL_WIDTH  (SEL_WIDTH),
        .NUM_REGS   (REGISTER_NUM + 1)
    ) u_adc_reg_bank (
        .i_clk           (clk),
        .i_rst_n         (rst_sys_sync_n),
        .i_valid         (w_mem_valid),
        .i_write         (w_mem_we),
        .i_addr          (w_mem_addr),
        .i_data          (w_mem_data_w),
        .i_sel           (w_mem_sel),
        .o_data          (w_mem_data_r),
        .i_err_ext       (i_err_ext),
        .i_adc_data_rdy  (w_adc_data_rdy),
        .i_adc_dout      (w_adc_dout),
        .o_adc_reset     (o_adc_reset),
        .o_adc_enable    (o_adc_enable),
        .o_adc_mode      (o_adc_mode),
        .o_sample_rate   (o_sample_rate),
        .o_filter_cfg    (o_filter_cfg),
        .o_gain_cfg      (o_gain_cfg),
        .o_filter_coeffs (o_filter_coeffs)
    );

    // ------------------------------------------------------------------
    // 5. Instanciação Direta do Núcleo do ADC SAR de 10 bits (adc)
    // ------------------------------------------------------------------
    // Reset do ADC ativo baixo: acionado por rst_sys ou pelo bit RESET (Bit 7) de ADC_CTRL
    assign w_adc_rst_n = rst_sys_sync_n && (!o_adc_reset);

    adc #(
        .V_FULL_SCALE (V_FULL_SCALE),
        .RESOLUTION   (RESOLUTION),
        .T_CONV_NS    (T_CONV_NS)
    ) u_adc (
        .clk      (clk),
        .rst_n    (w_adc_rst_n),
        .enable   (o_adc_enable),
        .v_in     (v_in),
        .d_out    (w_adc_dout),
        .data_rdy (w_adc_data_rdy)
    );

    assign o_adc_dout     = w_adc_dout;
    assign o_adc_data_rdy = w_adc_data_rdy;

endmodule

