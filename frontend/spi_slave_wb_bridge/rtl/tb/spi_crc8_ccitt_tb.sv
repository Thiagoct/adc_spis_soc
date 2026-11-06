`timescale 1ns / 1ps
// ----------------------------------------------------------------------
// Copyright (c) 2026 VIRTUS-UFCG. All rights reserved
// VIRTUS-UFCG Confidential Proprietary
// ----------------------------------------------------------------------
// Module     : spi_crc8_ccitt_tb
// Description: Testbench unitário para validação do módulo spi_crc8_ccitt.
// ----------------------------------------------------------------------

module spi_crc8_ccitt_tb;

    logic [15:0] i_tx_data;
    logic [7:0]  o_tx_crc;

    logic [15:0] i_rx_data;
    logic [7:0]  i_rx_data_crc;
    logic [7:0]  o_rx_data_calc_crc;
    logic        o_rx_data_crc_ok;

    logic [12:0] i_rx_hdr;
    logic [7:0]  i_rx_hdr_crc;
    logic [7:0]  o_rx_hdr_calc_crc;
    logic        o_rx_hdr_crc_ok;

    int err_cnt = 0;

    spi_crc8_ccitt uut (
        .i_tx_data          (i_tx_data),
        .o_tx_crc           (o_tx_crc),
        .i_rx_data          (i_rx_data),
        .i_rx_data_crc      (i_rx_data_crc),
        .o_rx_data_calc_crc (o_rx_data_calc_crc),
        .o_rx_data_crc_ok   (o_rx_data_crc_ok),
        .i_rx_hdr           (i_rx_hdr),
        .i_rx_hdr_crc       (i_rx_hdr_crc),
        .o_rx_hdr_calc_crc  (o_rx_hdr_calc_crc),
        .o_rx_hdr_crc_ok    (o_rx_hdr_crc_ok)
    );

    initial begin
        $display("=========================================================");
        $display("  TESTBENCH: spi_crc8_ccitt (Polinomio 0x07, Init 0x00)  ");
        $display("=========================================================");

        i_tx_data     = 16'h0000;
        i_rx_data     = 16'h0000;
        i_rx_data_crc = 8'h00;
        i_rx_hdr      = 13'h0000;
        i_rx_hdr_crc  = 8'h00;
        #10;

        // Vetor 1: 0x0000 -> CRC = 0x00
        if (o_tx_crc !== 8'h00 || o_rx_data_crc_ok !== 1'b1) begin
            $display("[FAIL] CRC(0x0000) = 0x%02h (esperado 0x00)", o_tx_crc);
            err_cnt++;
        end else begin
            $display("[PASS] CRC(0x0000) = 0x%02h", o_tx_crc);
        end

        // Vetor 2: 0x1234 -> valida loopback TX -> RX
        i_tx_data = 16'h1234;
        #5;
        i_rx_data     = 16'h1234;
        i_rx_data_crc = o_tx_crc;
        #5;
        if (o_rx_data_crc_ok !== 1'b1) begin
            $display("[FAIL] Loopback CRC(0x1234) = 0x%02h falhou", o_tx_crc);
            err_cnt++;
        end else begin
            $display("[PASS] Loopback CRC(0x1234) = 0x%02h validado com sucesso", o_tx_crc);
        end

        // Vetor 3: Injeção de erro no CRC recebido
        i_rx_data_crc = o_tx_crc ^ 8'h01;
        #5;
        if (o_rx_data_crc_ok !== 1'b0) begin
            $display("[FAIL] Erro de CRC nao detectado!");
            err_cnt++;
        end else begin
            $display("[PASS] Divergencia (mismatch) de CRC detectada corretamente");
        end

        // Vetor 4: Cabeçalho de 13 bits
        i_rx_hdr = {1'b1, 12'h005};
        #5;
        i_rx_hdr_crc = o_rx_hdr_calc_crc;
        #5;
        if (o_rx_hdr_crc_ok !== 1'b1) begin
            $display("[FAIL] Validacao de CRC de cabecalho falhou");
            err_cnt++;
        end else begin
            $display("[PASS] CRC de cabecalho (13b = 0x%04h) = 0x%02h validado", i_rx_hdr, o_rx_hdr_calc_crc);
        end

        if (err_cnt == 0)
            $display("RESULTADO FINAL: PASS (0 erros)");
        else
            $display("RESULTADO FINAL: FAIL (%0d erros)", err_cnt);

        $finish;
    end

endmodule

