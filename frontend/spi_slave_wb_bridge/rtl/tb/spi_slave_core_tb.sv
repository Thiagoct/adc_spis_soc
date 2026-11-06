`timescale 1ns / 1ps
// ----------------------------------------------------------------------
// Copyright (c) 2026 VIRTUS-UFCG. All rights reserved
// VIRTUS-UFCG Confidential Proprietary
// ----------------------------------------------------------------------
// Module     : spi_slave_core_tb
// Description: Testbench unitário para o núcleo spi_slave_core.
// ----------------------------------------------------------------------

module spi_slave_core_tb;

    localparam real CLK_PERIOD = 10.0;   // 100 MHz
    localparam real SCK_HALF   = 500.0;  // 1 MHz SCK

    logic        clk;
    logic        rst_s_n;
    logic        cpol;
    logic        cpha;
    logic        bit_order;
    logic        sck;
    logic        ss;
    logic        mosi;
    logic        miso;
    logic        spi_busy;

    logic [15:0] w_crc_tx_data;
    logic [7:0]  w_crc_tx_val;
    logic [15:0] w_crc_rx_data;
    logic [7:0]  w_crc_rx_val;
    logic        w_crc_rx_data_ok;
    logic [12:0] w_crc_rx_hdr;
    logic [7:0]  w_crc_rx_hdr_val;
    logic        w_crc_rx_hdr_ok;

    logic        o_cmd_load_addr;
    logic [11:0] o_init_addr;
    logic        o_wb_wr_req;
    logic [15:0] o_wb_wr_data;
    logic        o_wb_rd_req;
    logic        i_wb_rd_valid;
    logic [15:0] i_wb_rd_data;
    logic        i_wb_busy;
    logic        o_crc_err;

    spi_crc8_ccitt u_crc (
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

    spi_slave_core #(
        .CRC_HEADER_EN (1'b0)
    ) uut (
        .clk              (clk),
        .rst_s_n          (rst_s_n),
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
        .i_crc_rx_hdr_ok  (w_crc_rx_hdr_ok),
        .o_crc_rx_hdr_val (w_crc_rx_hdr_val),
        .o_cmd_load_addr  (o_cmd_load_addr),
        .o_init_addr      (o_init_addr),
        .o_wb_wr_req      (o_wb_wr_req),
        .o_wb_wr_data     (o_wb_wr_data),
        .o_wb_rd_req      (o_wb_rd_req),
        .o_wb_addr_skip   (),
        .i_wb_rd_valid    (i_wb_rd_valid),
        .i_wb_rd_data     (i_wb_rd_data),
        .i_wb_busy        (i_wb_busy),
        .o_crc_err        (o_crc_err)
    );

    always #(CLK_PERIOD / 2.0) clk = ~clk;

    // Respondedor simples ao pulso o_wb_rd_req
    always_ff @(posedge clk or negedge rst_s_n) begin
        if (!rst_s_n) begin
            i_wb_rd_valid <= 1'b0;
            i_wb_rd_data  <= 16'hA55A;
            i_wb_busy     <= 1'b0;
        end else begin
            i_wb_rd_valid <= o_wb_rd_req;
        end
    end

    // Função CRC8-CCITT auxiliar
    function automatic logic [7:0] calc_crc8(input logic [15:0] d);
        logic [7:0] c;
        logic fb;
        c = 8'h00;
        for (int i = 15; i >= 0; i--) begin
            fb = c[7] ^ d[i];
            c  = {c[6:0], 1'b0} ^ (fb ? 8'h07 : 8'h00);
        end
        return c;
    endfunction

    task automatic send_bit_mode0(input logic b, output logic rx_b);
        mosi = b;
        #(SCK_HALF);
        sck  = 1'b1; // Borda de subida (captura em Modo 0)
        rx_b = miso;
        #(SCK_HALF);
        sck  = 1'b0; // Borda de descida (lançamento em Modo 0)
    endtask

    initial begin
        logic dummy;
        logic [7:0]  crc;
        logic [11:0] test_addr;
        logic [15:0] test_data;
        clk       = 1'b0;
        rst_s_n   = 1'b0;
        cpol      = 1'b0;
        cpha      = 1'b0;
        bit_order = 1'b1;
        sck       = 1'b0;
        ss        = 1'b1;
        mosi      = 1'b0;
        test_addr = 12'h004;
        test_data = 16'hABCD;

        #100;
        rst_s_n = 1'b1;
        #100;

        $display("[TB] Iniciando teste de escrita no spi_slave_core...");
        ss = 1'b0;
        #200;

        // Comando Write (0)
        send_bit_mode0(1'b0, dummy);
        // Endereço 0x004 (MSB first)
        for (int i = 11; i >= 0; i--)
            send_bit_mode0(test_addr[i], dummy);

        // Dados 0xABCD + CRC
        crc = calc_crc8(test_data);
        for (int i = 15; i >= 0; i--)
            send_bit_mode0(test_data[i], dummy);
        for (int i = 7; i >= 0; i--)
            send_bit_mode0(crc[i], dummy);

        #200;
        ss = 1'b1;
        #200;

        if (o_wb_wr_data == 16'hABCD && o_init_addr == 12'h004 && !o_crc_err)
            $display("[PASS] spi_slave_core capturou Addr=0x%03h, Data=0x%04h sem erro de CRC!", o_init_addr, o_wb_wr_data);
        else
            $display("[FAIL] Addr=0x%03h, Data=0x%04h, crc_err=%b", o_init_addr, o_wb_wr_data, o_crc_err);

        $finish;
    end

endmodule
