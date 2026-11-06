`timescale 1ns / 1ps
// ----------------------------------------------------------------------
// Copyright (c) 2026 VIRTUS-UFCG. All rights reserved
// VIRTUS-UFCG Confidential Proprietary
// ----------------------------------------------------------------------
// Module     : adc_reg_bank_tb
// Description: Testbench unitário para validação direta do adc_reg_bank.
// ----------------------------------------------------------------------

module adc_reg_bank_tb;

    localparam real CLK_PERIOD = 10.0; // 100 MHz

    logic        i_clk;
    logic        i_rst_n;
    logic        i_valid;
    logic        i_write;
    logic [11:0] i_addr;
    logic [15:0] i_data;
    logic [1:0]  i_sel;
    logic [15:0] o_data;

    logic        i_err_ext;
    logic        i_adc_data_rdy;
    logic [9:0]  i_adc_dout;

    logic        o_adc_reset;
    logic        o_adc_enable;
    logic [5:0]  o_adc_mode;
    logic [7:0]  o_sample_rate;
    logic [15:0] o_filter_cfg;
    logic [7:0]  o_gain_cfg;
    logic [15:0] o_filter_coeffs [0:9];

    int err_cnt = 0;

    adc_reg_bank uut (
        .i_clk           (i_clk),
        .i_rst_n         (i_rst_n),
        .i_valid         (i_valid),
        .i_write         (i_write),
        .i_addr          (i_addr),
        .i_data          (i_data),
        .i_sel           (i_sel),
        .o_data          (o_data),
        .i_err_ext       (i_err_ext),
        .i_adc_data_rdy  (i_adc_data_rdy),
        .i_adc_dout      (i_adc_dout),
        .o_adc_reset     (o_adc_reset),
        .o_adc_enable    (o_adc_enable),
        .o_adc_mode      (o_adc_mode),
        .o_sample_rate   (o_sample_rate),
        .o_filter_cfg    (o_filter_cfg),
        .o_gain_cfg      (o_gain_cfg),
        .o_filter_coeffs (o_filter_coeffs)
    );

    initial i_clk = 1'b0;
    always #(CLK_PERIOD / 2.0) i_clk = ~i_clk;

    task automatic reg_write(input logic [11:0] addr, input logic [15:0] data, input logic [1:0] sel = 2'b11);
        @(posedge i_clk);
        i_valid <= 1'b1;
        i_write <= 1'b1;
        i_addr  <= addr;
        i_data  <= data;
        i_sel   <= sel;
        @(posedge i_clk);
        i_valid <= 1'b0;
        i_write <= 1'b0;
    endtask

    initial begin
        $display("=========================================================");
        $display("  TESTBENCH UNITARIO: adc_reg_bank                       ");
        $display("=========================================================");

        i_rst_n        = 1'b0;
        i_valid        = 1'b0;
        i_write        = 1'b0;
        i_addr         = 12'h000;
        i_data         = 16'h0000;
        i_sel          = 2'b11;
        i_err_ext      = 1'b0;
        i_adc_data_rdy = 1'b0;
        i_adc_dout     = 10'h000;

        repeat (4) @(posedge i_clk);
        i_rst_n = 1'b1;
        repeat (2) @(posedge i_clk);

        // Escrita em SAMPLE_RATE (0x01), FILTER_CFG (0x02), GAIN_CFG (0x04)
        reg_write(12'h001, 16'h0055);
        reg_write(12'h002, 16'h1234);
        reg_write(12'h004, 16'h00AA);
        @(posedge i_clk);

        if (o_sample_rate !== 8'h55 || o_filter_cfg !== 16'h1234 || o_gain_cfg !== 8'hAA) begin
            $display("[FAIL] Escrita nos registos de configuracao falhou!");
            err_cnt++;
        end else begin
            $display("[PASS] SAMPLE_RATE=0x%02h, FILTER_CFG=0x%04h, GAIN_CFG=0x%02h",
                     o_sample_rate, o_filter_cfg, o_gain_cfg);
        end

        // Teste de Write Strobe parcial em FILTER_CFG (0x02): atualiza apenas byte alto
        reg_write(12'h002, 16'h99FF, 2'b10);
        @(posedge i_clk);
        if (o_filter_cfg !== 16'h9934) begin
            $display("[FAIL] Write strobe parcial falhou: obtido 0x%04h (esp 0x9934)", o_filter_cfg);
            err_cnt++;
        end else begin
            $display("[PASS] Write strobe parcial (wb_sel_i=2'b10) -> FILTER_CFG=0x%04h", o_filter_cfg);
        end

        if (err_cnt == 0)
            $display("RESULTADO FINAL: PASS (0 erros)");
        else
            $display("RESULTADO FINAL: FAIL (%0d erros)", err_cnt);

        $finish;
    end

endmodule

