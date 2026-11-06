// ----------------------------------------------------------------------
// Copyright (c) 2026 VIRTUS-UFCG. All rights reserved
// VIRTUS-UFCG Confidential Proprietary
// ----------------------------------------------------------------------
// Module     : adc
// Description: Módulo topo comportamental do ADC SAR de 10 bits,
//              integrando o bloco analógico (adc_analog) e o bloco
//              digital (adc_digital).
// ----------------------------------------------------------------------

module adc #(
    parameter real V_FULL_SCALE = 0.8,
    parameter int  RESOLUTION   = 10,
    parameter real T_CONV_NS    = 100000.0
) (
    input  logic       clk,      // System clock
    input  logic       rst_n,    // System reset (ativo baixo)
    input  logic       enable,   // ADC enable (ADC_CTRL bit 6)
    input  real        v_in,     // Entrada analógica

    output logic [9:0] d_out,    // Dados da conversão de 10 bits
    output logic       data_rdy  // Flag indicadora de dado pronto
);

    logic       w_sample;
    logic [9:0] w_d_raw;

    adc_analog #(
        .V_FULL_SCALE (V_FULL_SCALE),
        .RESOLUTION   (RESOLUTION)
    ) u_analog (
        .v_in   (v_in),
        .sample (w_sample),
        .d_raw  (w_d_raw)
    );

    adc_digital #(
        .T_CONV_NS (T_CONV_NS)
    ) u_digital (
        .rst_n    (rst_n),
        .enable   (enable),
        .d_raw    (w_d_raw),
        .sample   (w_sample),
        .d_out    (d_out),
        .data_rdy (data_rdy)
    );

endmodule

