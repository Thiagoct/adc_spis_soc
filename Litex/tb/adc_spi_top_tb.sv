`timescale 1ns / 1ps
// ----------------------------------------------------------------------
// Copyright (c) 2026 VIRTUS-UFCG. All rights reserved
// VIRTUS-UFCG Confidential Proprietary
// ----------------------------------------------------------------------
// Module     : adc_spi_top_tb
// Description: Testbench completo de verificacao do Topo Geral (adc_spi_top):
//              - Sub-bloco 2: spi_wb_bridge (SPI Slave -> Wishbone Master + CRC8-CCITT)
//              - Sub-bloco 1: adc_wb_slave_top (Wishbone Slave + Banco de Registos + ADC SAR 10b)
//
//              Cobre 100% dos requisitos contratuais ("Shall") da Especificacao:
//               [1] Escrita e leitura simples dos registos + Conversao ADC SAR 10b
//               [2] Burst de escrita e leitura com auto-incremento (0x006 a 0x009)
//               [3] Todos os 4 modos SPI (CPOL/CPHA) x Bit Order (MSB/LSB)
//               [4] Injecao de erro de CRC8-CCITT de dados e status em SPI_STATUS (0x05)
//               [5] Reset de software via ADC_CTRL bit 7
//               [6] GAP A (6.3.1.12/13): reset_s assert assincrono, deassert sincrono
//                   e reset_s no meio de uma transacao SPI
//               [7] GAP C (5.3 e 6.3.1.14): abort por ss no meio de endereco/dado/CRC/leitura
//                   e isolamento de trafego espurio com ss=1
//               [8] GAP D: Burst de escrita com CRC invalido na palavra intermediaria
//                   (palavra ruim bloqueada, auto-incremento preservado para as seguintes)
//               [9] GAP B (6.3.3.14): CRC do cabecalho de 13 bits (quando CRC_HEADER_EN=1)
//              [10] GAP E (6.2.1.2.3): Configuracao da taxa de amostragem (SAMPLE_RATE 0x01)
// ----------------------------------------------------------------------

module adc_spi_top_tb #(
    parameter bit CRC_HEADER_EN = 1'b0
);

    // =========================================================================
    // 1. Parametros de Tempo (clk = 100 MHz -> 10 ns, sck = 1 MHz -> 1000 ns)
    // =========================================================================
    localparam real CLK_PERIOD_NS = 10.0;   // 100 MHz
    localparam real SCK_HALF_NS   = 500.0;  // 1 MHz (meio periodo = 500 ns)

    // =========================================================================
    // 2. Sinais do Topo Digital e Interface
    // =========================================================================
    logic        clk;
    logic        sck;
    logic        reset_s;
    logic        rst_sys;

    logic        cpol;
    logic        cpha;
    logic        bit_order;

    logic        ss;
    logic        mosi;
    logic        miso;
    logic        spi_busy;

    logic        crc_err;

    // Sinais do ADC SAR 10b / Banco de Registos
    real         v_in;
    logic        adc_reset;
    logic        adc_enable;
    logic [5:0]  adc_mode;
    logic [7:0]  sample_rate;
    logic [15:0] filter_cfg;
    logic [7:0]  gain_cfg;
    logic [9:0]  adc_dout;
    logic        adc_data_rdy;

    // Contadores de verificacao
    int total_errors = 0;
    int total_checks = 0;

    // Monitores de atividade
    int wb_wr_cnt    = 0;
    int wb_rd_cnt    = 0;
    int wb_start_cnt = 0;
    int crc_err_seen = 0;

    always @(posedge clk) begin
        if (crc_err) crc_err_seen++;
        if ($rose(u_dut.wb_cyc)) wb_start_cnt++;
        if (u_dut.wb_cyc && u_dut.wb_stb && u_dut.wb_ack) begin
            if (u_dut.wb_we) wb_wr_cnt++;
            else             wb_rd_cnt++;
        end
    end

    // =========================================================================
    // 3. Instanciacao do Topo Geral (spi_wb_bridge + adc_wb_slave_top)
    // =========================================================================
    adc_spi_top #(
        .CRC_HEADER_EN (CRC_HEADER_EN),
        .V_FULL_SCALE  (0.8),
        .RESOLUTION    (10),
        .T_CONV_NS     (10000.0) // 10 us para conversao durante teste
    ) u_dut (
        .clk            (clk),
        .sck            (sck),
        .reset_s        (reset_s),
        .rst_sys        (rst_sys),
        .cpol           (cpol),
        .cpha           (cpha),
        .bit_order      (bit_order),
        .ss             (ss),
        .mosi           (mosi),
        .miso           (miso),
        .spi_busy       (spi_busy),
        .v_in           (v_in),
        .o_adc_reset    (adc_reset),
        .o_adc_enable   (adc_enable),
        .o_adc_mode     (adc_mode),
        .o_sample_rate  (sample_rate),
        .o_filter_cfg   (filter_cfg),
        .o_gain_cfg     (gain_cfg),
        .o_adc_dout     (adc_dout),
        .o_adc_data_rdy (adc_data_rdy),
        .o_crc_err      (crc_err)
    );

    // =========================================================================
    // 4. Geracao de Clock
    // =========================================================================
    initial clk = 1'b0;
    always #(CLK_PERIOD_NS / 2.0) clk = ~clk;

    // Helper de assercoes
    task automatic chk(input bit cond, input string msg);
        begin
            total_checks++;
            if (cond) $display("[PASS] %s", msg);
            else begin
                $display("[FAIL] %s", msg);
                total_errors++;
            end
        end
    endtask

    // =========================================================================
    // 5. Funcoes de CRC8-CCITT (Polinomio x^8 + x^2 + x + 1 = 0x07)
    // =========================================================================
    function automatic logic [7:0] calc_crc8_ccitt(input logic [15:0] data_word);
        logic [7:0] crc;
        logic       fb;
        begin
            crc = 8'h00;
            for (int i = 15; i >= 0; i--) begin
                fb  = crc[7] ^ data_word[i];
                crc = {crc[6:0], 1'b0} ^ (fb ? 8'h07 : 8'h00);
            end
            return crc;
        end
    endfunction

    // CRC do cabecalho: 13 bits {cmd_read, addr[11:0]} processados MSB first
    function automatic logic [7:0] calc_hdr_crc(input logic cmd_r, input logic [11:0] addr);
        logic [12:0] p;
        logic [7:0]  c;
        logic        fb;
        begin
            p = {cmd_r, addr};
            c = 8'h00;
            for (int i = 12; i >= 0; i--) begin
                fb = c[7] ^ p[i];
                c  = {c[6:0], 1'b0} ^ (fb ? 8'h07 : 8'h00);
            end
            return c;
        end
    endfunction

    // =========================================================================
    // 6. Tasks do BFM do Mestre SPI
    // =========================================================================
    task automatic spi_transfer_bit(
        input  logic tx_b,
        output logic rx_b
    );
        logic lead_lvl;
        logic trail_lvl;
        begin
            lead_lvl  = (cpol == 1'b0) ? 1'b1 : 1'b0;
            trail_lvl = (cpol == 1'b0) ? 1'b0 : 1'b1;

            if (cpha == 1'b0) begin
                mosi = tx_b;
                #(SCK_HALF_NS);
                sck  = lead_lvl;
                rx_b = miso;
                #(SCK_HALF_NS);
                sck  = trail_lvl;
            end else begin
                sck  = lead_lvl;
                mosi = tx_b;
                #(SCK_HALF_NS);
                sck  = trail_lvl;
                rx_b = miso;
                #(SCK_HALF_NS);
            end
        end
    endtask

    task automatic spi_send_bits(
        input logic [15:0] v,
        input int          width,
        input int          nbits,
        input bit          expect_miso0 = 1'b1
    );
        logic rx_b;
        int   idx;
        begin
            for (int i = 0; i < nbits; i++) begin
                idx = bit_order ? (width - 1 - i) : i;
                spi_transfer_bit(v[idx], rx_b);
                if (expect_miso0 && rx_b !== 1'b0) begin
                    $display("[ERRO] MISO != 0 durante envio (bit %0d)", i);
                    total_errors++;
                end
            end
        end
    endtask

    task automatic spi_send_header(
        input logic        cmd_read,
        input logic [11:0] addr,
        input bit          bad_hdr_crc = 1'b0,
        input int          nbits_addr  = 12
    );
        logic rx_b;
        logic [7:0] hc;
        begin
            spi_transfer_bit(cmd_read, rx_b);
            if (rx_b !== 1'b0) begin
                $display("[ERRO] MISO != 0 no bit de comando!");
                total_errors++;
            end
            spi_send_bits({4'h0, addr}, 12, nbits_addr);
            if (CRC_HEADER_EN && nbits_addr == 12) begin
                hc = calc_hdr_crc(cmd_read, addr);
                if (bad_hdr_crc) hc = ~hc;
                spi_send_bits({8'h00, hc}, 8, 8);
            end
        end
    endtask

    task automatic spi_send_write_packet(
        input logic [15:0] data_w,
        input logic [7:0]  crc_w
    );
        begin
            spi_send_bits(data_w, 16, 16);
            spi_send_bits({8'h00, crc_w}, 8, 8);
        end
    endtask

    task automatic spi_recv_read_packet(
        output logic [15:0] data_r,
        output logic [7:0]  crc_r
    );
        logic rx_b;
        int   idx;
        begin
            data_r = 16'h0000;
            crc_r  = 8'h00;
            for (int i = 0; i < 16; i++) begin
                idx = bit_order ? (15 - i) : i;
                spi_transfer_bit(1'b0, rx_b);
                data_r[idx] = rx_b;
            end
            for (int i = 0; i < 8; i++) begin
                idx = bit_order ? (7 - i) : i;
                spi_transfer_bit(1'b0, rx_b);
                crc_r[idx] = rx_b;
            end
        end
    endtask

    task automatic spi_begin();
        begin
            sck = cpol;
            #(SCK_HALF_NS);
            ss  = 1'b0;
            #(SCK_HALF_NS);
        end
    endtask

    task automatic spi_end();
        begin
            #(SCK_HALF_NS);
            ss   = 1'b1;
            mosi = 1'b0;
            #(SCK_HALF_NS * 2);
        end
    endtask

    task automatic spi_write_single(
        input logic [11:0] addr,
        input logic [15:0] data_w,
        input bit          corrupt_crc     = 1'b0,
        input bit          corrupt_hdr_crc = 1'b0
    );
        logic [7:0] crc_val;
        begin
            crc_val = calc_crc8_ccitt(data_w);
            if (corrupt_crc) crc_val = ~crc_val;

            spi_begin();
            if (spi_busy !== 1'b0) begin
                $display("[ERRO] spi_busy deveria estar em 0 durante transacao!");
                total_errors++;
            end

            spi_send_header(1'b0, addr, corrupt_hdr_crc);
            spi_send_write_packet(data_w, crc_val);
            spi_end();
        end
    endtask

    task automatic spi_read_and_check(
        input logic [11:0] addr,
        input logic [15:0] expected_data,
        input string       reg_name
    );
        logic [15:0] rd_data;
        logic [7:0]  rd_crc;
        logic [7:0]  exp_crc;
        begin
            exp_crc = calc_crc8_ccitt(expected_data);

            spi_begin();
            spi_send_header(1'b1, addr);
            spi_recv_read_packet(rd_data, rd_crc);
            spi_end();

            total_checks++;
            if ((rd_data === expected_data) && (rd_crc === exp_crc)) begin
                $display("[PASS] Leitura %s (0x%03h): Dado=0x%04h | CRC=0x%02h (CPOL=%0b, CPHA=%0b, MSB=%0b)",
                         reg_name, addr, rd_data, rd_crc, cpol, cpha, bit_order);
            end else begin
                $display("[FAIL] Leitura %s (0x%03h): Obtido=0x%04h (esp 0x%04h) | CRC=0x%02h (esp 0x%02h)",
                         reg_name, addr, rd_data, expected_data, rd_crc, exp_crc);
                total_errors++;
            end
        end
    endtask

    task automatic check_idle(input string msg);
        begin
            repeat (10) @(posedge clk);
            chk(spi_busy === 1'b1 && miso === 1'b0 &&
                u_dut.u_spi_wb_bridge.u_spi_slave_core.state === 3'd0, msg);
        end
    endtask

    // =========================================================================
    // 7. Sequencia Principal de Testes
    // =========================================================================
    initial begin
        logic [15:0] burst_tx [0:3];
        logic [15:0] burst_rx;
        logic [7:0]  burst_crc;
        logic [7:0]  exp_crc;
        logic [15:0] rd;
        logic [7:0]  rc;
        logic [7:0]  c8;
        logic [15:0] reg_snap [0:15];
        bit          regs_ok;
        int          snap_wr, snap_rd, snap_start, bad_idx;

        $display("===================================================================================");
        $display("   SIMULACAO: TOPO GERAL (adc_spi_top) | CRC_HEADER_EN=%0d", CRC_HEADER_EN);
        $display("===================================================================================");

        reset_s   = 1'b0;
        rst_sys   = 1'b0;
        cpol      = 1'b0;
        cpha      = 1'b0;
        bit_order = 1'b1;
        sck       = 1'b0;
        ss        = 1'b1;
        mosi      = 1'b0;
        v_in      = 0.4; // 0.4V -> metade da escala de 0.8V (código esperado = 511 = 0x01FF)

        #(CLK_PERIOD_NS * 10);
        reset_s = 1'b1;
        rst_sys = 1'b1;
        #(CLK_PERIOD_NS * 10);

        chk(spi_busy === 1'b1 && miso === 1'b0, "Estado Ocioso Inicial: spi_busy=1, miso=0");

        // ---------------------------------------------------------------------
        // TESTE 1: Escrita e Leitura Simples no Mapa de Registos + Conversao do ADC
        // ---------------------------------------------------------------------
        $display("\n--- [TESTE 1] Escrita e Leitura Simples nos Registos do ADC (Modo 0, MSB) ---");
        spi_write_single(12'h000, 16'h0040); // ADC_CTRL: ENABLE = 1 (bit 6)
        spi_write_single(12'h001, 16'h003C); // SAMPLE_RATE = 0x3C
        spi_write_single(12'h002, 16'hA55A); // FILTER_CFG  = 0xA55A
        spi_write_single(12'h004, 16'h001F); // GAIN_CFG    = 0x1F

        chk(adc_enable === 1'b1 && sample_rate === 8'h3C && filter_cfg === 16'hA55A && gain_cfg === 8'h1F,
            "Sinais fisicos do ADC configurados (enable=1, rate=0x3C, filter=0xA55A, gain=0x1F)");

        spi_read_and_check(12'h000, 16'h0040, "ADC_CTRL");
        spi_read_and_check(12'h001, 16'h003C, "SAMPLE_RATE");
        spi_read_and_check(12'h002, 16'hA55A, "FILTER_CFG");
        spi_read_and_check(12'h004, 16'h001F, "GAIN_CFG");
        spi_read_and_check(12'h003, 16'h01FF, "ADC_DATA (0.4V -> 511 = 0x01FF)");

        // ---------------------------------------------------------------------
        // TESTE 2: Operacao em Modo BURST (Write & Read com Auto-Incremento)
        // ---------------------------------------------------------------------
        $display("\n--- [TESTE 2] Escrita e Leitura em BURST com Auto-Incremento (0x006 a 0x009) ---");
        burst_tx[0] = 16'hCAFE;
        burst_tx[1] = 16'hBEEF;
        burst_tx[2] = 16'hDEAD;
        burst_tx[3] = 16'hF00D;

        spi_begin();
        spi_send_header(1'b0, 12'h006);
        for (int w = 0; w < 4; w++) begin
            spi_send_write_packet(burst_tx[w], calc_crc8_ccitt(burst_tx[w]));
        end
        spi_end();

        spi_begin();
        spi_send_header(1'b1, 12'h006);
        for (int w = 0; w < 4; w++) begin
            spi_recv_read_packet(burst_rx, burst_crc);
            exp_crc = calc_crc8_ccitt(burst_tx[w]);
            total_checks++;
            if ((burst_rx === burst_tx[w]) && (burst_crc === exp_crc)) begin
                $display("[PASS] Burst Read Palavra %0d (Addr 0x%03h): Dado=0x%04h | CRC=0x%02h",
                         w, 12'h006 + w, burst_rx, burst_crc);
            end else begin
                $display("[FAIL] Burst Read Palavra %0d (Addr 0x%03h): Obtido=0x%04h (esp 0x%04h)",
                         w, 12'h006 + w, burst_rx, burst_tx[w]);
                total_errors++;
            end
        end
        spi_end();

        // ---------------------------------------------------------------------
        // TESTE 3: Verificacao dos 4 Modos SPI (CPOL/CPHA) e Ordem de Bits (MSB/LSB)
        // ---------------------------------------------------------------------
        $display("\n--- [TESTE 3] Verificacao de Todos os Modos (CPOL/CPHA = 00,01,10,11) e Bit Order ---");
        for (int mode = 0; mode < 4; mode++) begin
            for (int bo = 0; bo < 2; bo++) begin
                logic [15:0] test_val;
                cpol      = mode[1];
                cpha      = mode[0];
                bit_order = bo[0];
                sck       = cpol;
                #(SCK_HALF_NS * 2);

                test_val = 16'h5000 | (mode << 8) | (bo << 4) | 16'h000A;
                spi_write_single(12'h00A + mode[1:0], test_val);
                spi_read_and_check(12'h00A + mode[1:0], test_val, "RESERVED_CFG");
            end
        end

        cpol      = 1'b0;
        cpha      = 1'b0;
        bit_order = 1'b1;
        sck       = 1'b0;
        #(SCK_HALF_NS * 2);

        // ---------------------------------------------------------------------
        // TESTE 4: Validacao de Erro de CRC8-CCITT de Dado e SPI_STATUS (0x05)
        // ---------------------------------------------------------------------
        $display("\n--- [TESTE 4] Injecao de Erro de CRC8-CCITT e Feedback em SPI_STATUS (0x05) ---");
        spi_write_single(12'h002, 16'hFFFF, 1'b1);
        spi_read_and_check(12'h002, 16'hA55A, "FILTER_CFG (Protegido por CRC)");
        spi_read_and_check(12'h005, 16'h00C0, "SPI_STATUS (ERR=1, DATA_RDY=1)");

        // ---------------------------------------------------------------------
        // TESTE 5: Reset Sincrono via ADC_CTRL Bit 7 (RESET = 1 -> 0)
        // ---------------------------------------------------------------------
        $display("\n--- [TESTE 5] Reset Sincrono via Registo ADC_CTRL (Bit 7 = 1 -> 0) ---");
        spi_write_single(12'h000, 16'h0080); // Bit 7 = RESET
        spi_write_single(12'h000, 16'h0000); // Limpa Bit 7 apos reset
        spi_read_and_check(12'h001, 16'h0000, "SAMPLE_RATE Pos-Reset");
        spi_read_and_check(12'h005, 16'h0000, "SPI_STATUS Pos-Reset (ERR limpo)");

        // ---------------------------------------------------------------------
        // TESTE 6: GAP A (6.3.1.12 & 6.3.1.13) - reset_s Asymmetry & Synchronism
        // ---------------------------------------------------------------------
        $display("\n--- [TESTE 6] GAP A: reset_s (Assert Assincrono, Deassert Sincrono e Mid-Stream) ---");
        // 6a: Assert assincrono (cai imediatamente sem borda de clk)
        chk(u_dut.u_spi_wb_bridge.rst_s_sync_n === 1'b1, "reset_s: sincronizado em 1 antes do teste");
        @(posedge clk); #3 reset_s = 1'b0; #1;
        chk(u_dut.u_spi_wb_bridge.rst_s_sync_n === 1'b0, "reset_s: assert ASSINCRONO imediato");
        #(CLK_PERIOD_NS * 3);
        chk(u_dut.u_spi_wb_bridge.rst_s_sync_n === 1'b0, "reset_s: mantido em 0 enquanto reset_s=0");

        // 6b: Deassert sincrono (requer 2 bordas de clk)
        @(posedge clk); #3 reset_s = 1'b1; #1;
        chk(u_dut.u_spi_wb_bridge.rst_s_sync_n === 1'b0, "reset_s: deassert NAO assincrono");
        @(posedge clk); #1;
        chk(u_dut.u_spi_wb_bridge.rst_s_sync_n === 1'b0, "reset_s: deassert sincrono, ainda 0 apos 1a borda");
        @(posedge clk); #1;
        chk(u_dut.u_spi_wb_bridge.rst_s_sync_n === 1'b1, "reset_s: deassert sincrono, 1 apos 2a borda");

        // 6c: Reset no meio de uma transacao SPI
        spi_write_single(12'h00C, 16'h1234); // Valor de referencia
        snap_start = wb_start_cnt;
        spi_begin();
        spi_send_header(1'b0, 12'h00C);
        spi_send_bits(16'h9999, 16, 5);      // 5 bits enviados
        @(posedge clk); #3 reset_s = 1'b0;
        #(CLK_PERIOD_NS * 3);
        chk(u_dut.u_spi_wb_bridge.u_spi_slave_core.state === 3'd0 && miso === 1'b0,
            "reset_s no meio da transacao: FSM volta a ST_IDLE, miso=0");
        @(posedge clk); #3 reset_s = 1'b1;
        #(CLK_PERIOD_NS * 5);
        spi_end();
        check_idle("reset_s no meio da transacao: apos ss=1, spi_busy=1 e FSM ociosa");
        chk(wb_start_cnt == snap_start, "reset_s no meio da transacao: nenhuma escrita Wishbone gerada");
        spi_read_and_check(12'h00C, 16'h1234, "REG 0x00C intacto apos reset_s");
        spi_write_single(12'h00D, 16'hBEEF);
        spi_read_and_check(12'h00D, 16'hBEEF, "Transacao normal recuperada apos reset_s");

        // ---------------------------------------------------------------------
        // TESTE 7: GAP C (5.3 & 6.3.1.14) - Desativacao de ss e Isolamento
        // ---------------------------------------------------------------------
        $display("\n--- [TESTE 7] GAP C: Desativacao de ss (Abortos) e Trafego Espurio com ss=1 ---");
        spi_write_single(12'h00E, 16'h7777); // Referencia

        // 7a: Abort no meio do endereco (5 de 12 bits)
        snap_start = wb_start_cnt;
        spi_begin();
        spi_send_header(1'b0, 12'h00E, 1'b0, 5);
        spi_end();
        check_idle("Abort no meio do endereco: FSM ociosa e spi_busy=1");
        chk(wb_start_cnt == snap_start, "Abort no meio do endereco: nenhuma transacao Wishbone");

        // 7b: Abort no meio do dado (8 de 16 bits)
        spi_begin();
        spi_send_header(1'b0, 12'h00E);
        spi_send_bits(16'hFFFF, 16, 8);
        spi_end();
        check_idle("Abort no meio do dado: FSM ociosa e spi_busy=1");
        spi_read_and_check(12'h00E, 16'h7777, "Abort no meio do dado: palavra incompleta descartada");

        // 7c: Abort no meio do CRC do dado
        snap_wr = wb_wr_cnt;
        spi_begin();
        spi_send_header(1'b0, 12'h00E);
        spi_send_bits(16'hFFFF, 16, 16);
        spi_send_bits({8'h00, calc_crc8_ccitt(16'hFFFF)}, 8, 4);
        spi_end();
        check_idle("Abort no meio do CRC: FSM ociosa e spi_busy=1");
        chk(wb_wr_cnt == snap_wr, "Abort no meio do CRC: nenhuma escrita Wishbone");
        spi_read_and_check(12'h00E, 16'h7777, "0x00E inalterado apos abort de CRC");

        // 7d: Abort no meio da leitura (8 de 24 bits)
        spi_begin();
        spi_send_header(1'b1, 12'h00E);
        begin
            logic rx_b;
            for (int i = 0; i < 8; i++) spi_transfer_bit(1'b0, rx_b);
        end
        spi_end();
        check_idle("Abort no meio da leitura: FSM ociosa, miso=0 e spi_busy=1");

        // 7e: Trafego espurio em SCK e MOSI com ss=1
        snap_start = wb_start_cnt;
        ss = 1'b1;
        for (int k = 0; k < 60; k++) begin
            mosi = $urandom_range(0, 1);
            #(SCK_HALF_NS); sck = ~sck;
        end
        sck = cpol; mosi = 1'b0;
        #(SCK_HALF_NS);
        chk(wb_start_cnt == snap_start && miso === 1'b0 && spi_busy === 1'b1 &&
            u_dut.u_spi_wb_bridge.u_spi_slave_core.state === 3'd0,
            "Trafego espurio com ss=1 totalmente ignorado (sem Wishbone, miso=0, spi_busy=1)");

        // 7f: Transacao normal subsequente
        spi_write_single(12'h00F, 16'h4321);
        spi_read_and_check(12'h00F, 16'h4321, "Transacao normal apos abortos e trafego espurio");

        // ---------------------------------------------------------------------
        // TESTE 8: GAP D - Burst Write com CRC Corrompido no Meio da Sequencia
        // ---------------------------------------------------------------------
        $display("\n--- [TESTE 8] GAP D: Burst Write com CRC Ruim na Palavra 2 (0x006 a 0x009) ---");
        spi_write_single(12'h008, 16'h1111); // Pre-carrega 0x008 com valor de referencia
        burst_tx[0] = 16'hCAFE;
        burst_tx[1] = 16'hBEEF;
        burst_tx[2] = 16'hDEAD; // Esta sera enviada com CRC ruim
        burst_tx[3] = 16'hF00D;
        bad_idx = 2;

        snap_wr = wb_wr_cnt;
        crc_err_seen = 0;

        spi_begin();
        spi_send_header(1'b0, 12'h006);
        for (int w = 0; w < 4; w++) begin
            c8 = calc_crc8_ccitt(burst_tx[w]);
            if (w == bad_idx) c8 = ~c8;
            spi_send_write_packet(burst_tx[w], c8);
        end
        spi_end();

        check_idle("Burst com CRC ruim: FSM ociosa e spi_busy=1");
        chk(crc_err_seen > 0, "Burst com CRC ruim: crc_err sinalizado");
        chk(wb_wr_cnt - snap_wr == 3, "Burst com CRC ruim: exatamente 3 das 4 palavras escritas no Wishbone");

        // Confere que palavra 0 e 1 foram escritas, palavra 2 foi bloqueada, palavra 3 avancou para 0x009
        spi_read_and_check(12'h006, 16'hCAFE, "Burst CRC ruim: palavra 0 em 0x006");
        spi_read_and_check(12'h007, 16'hBEEF, "Burst CRC ruim: palavra 1 em 0x007");
        spi_read_and_check(12'h008, 16'h1111, "Burst CRC ruim: palavra 2 BLOQUEADA (0x008 inalterado)");
        spi_read_and_check(12'h009, 16'hF00D, "Burst CRC ruim: palavra 3 em 0x009 (auto-incremento ok)");

        // Confere feedback em SPI_STATUS (ERR = 1)
        spi_read_and_check(12'h005, 16'h0080, "SPI_STATUS (ERR=1 apos erro de CRC em burst)");

        // Limpa flag de erro por reset de software
        spi_write_single(12'h000, 16'h0080);
        spi_write_single(12'h000, 16'h0000);

        // ---------------------------------------------------------------------
        // TESTE 9: GAP B (6.3.3.14) - CRC do Cabecalho de 13 bits (CRC_HEADER_EN=1)
        // ---------------------------------------------------------------------
        $display("\n--- [TESTE 9] GAP B: CRC do Cabecalho de 13 bits (Opcao A vs Opcao B) ---");
        if (CRC_HEADER_EN) begin
            $display("[INFO] Executando validacao do CRC do Cabecalho (CRC_HEADER_EN=1)...");
            // 9a: Cabecalho invalido na escrita + dado com CRC valido -> deve abortar sem escrever
            spi_write_single(12'h00E, 16'h1111);
            snap_start = wb_start_cnt; snap_wr = wb_wr_cnt; crc_err_seen = 0;
            for (int i = 0; i < 16; i++) reg_snap[i] = u_dut.u_adc_wb_slave_top.u_adc_reg_bank.regs[i];

            spi_write_single(12'h00E, 16'hFFFF, 1'b0, 1'b1); // Cabecalho corrompido!
            check_idle("Cabecalho invalido (escrita): FSM volta a ociosa");
            chk(crc_err_seen > 0, "Cabecalho invalido (escrita): crc_err sinalizado");
            chk(wb_wr_cnt == snap_wr && wb_start_cnt == snap_start,
                "Cabecalho invalido (escrita): nenhuma transacao Wishbone gerada");
            spi_read_and_check(12'h00E, 16'h1111, "Cabecalho invalido: registo 0x00E protegido");

            regs_ok = 1'b1;
            for (int i = 0; i < 16; i++) begin
                if (u_dut.u_adc_wb_slave_top.u_adc_reg_bank.regs[i] !== reg_snap[i]) regs_ok = 1'b0;
            end
            chk(regs_ok, "Cabecalho invalido: nenhum registo foi alterado indevidamente");

            // 9b: Cabecalho invalido na leitura -> MISO=0 e sem leitura Wishbone
            snap_rd = wb_rd_cnt; crc_err_seen = 0;
            spi_begin();
            spi_send_header(1'b1, 12'h00E, 1'b1);
            spi_recv_read_packet(rd, rc);
            spi_end();
            chk(rd === 16'h0000 && rc === 8'h00 && wb_rd_cnt == snap_rd,
                "Cabecalho invalido (leitura): MISO=0 e sem leitura Wishbone");
            chk(crc_err_seen > 0, "Cabecalho invalido (leitura): crc_err sinalizado");

            // 9c: Recuperacao com cabecalho valido
            spi_write_single(12'h00E, 16'h2222);
            spi_read_and_check(12'h00E, 16'h2222, "Cabecalho valido apos erro: escrita/leitura OK");

            // Limpa flag de erro por reset de software
            spi_write_single(12'h000, 16'h0080);
            spi_write_single(12'h000, 16'h0000);
        end else begin
            $display("[N/A ] CRC_HEADER_EN=0 (Opcao B 6.3.4): sem CRC de cabecalho.");
            $display("       Para validar a Opcao A (6.3.3.14), execute: vsim -GCRC_HEADER_EN=1 adc_spi_top_tb");
        end

        // ---------------------------------------------------------------------
        // TESTE 10: GAP E (6.2.1.2.3) - Configuracao da Taxa de Amostragem
        // ---------------------------------------------------------------------
        $display("\n--- [TESTE 10] GAP E: Configuracao de SAMPLE_RATE (0x001) e Saida Fisica ---");
        spi_write_single(12'h001, 16'h0055);
        chk(sample_rate === 8'h55, "o_sample_rate atualizado para 0x55 na porta de saida fisica");
        spi_read_and_check(12'h001, 16'h0055, "Readback SAMPLE_RATE = 0x55");

        spi_write_single(12'h001, 16'h00FF);
        chk(sample_rate === 8'hFF, "o_sample_rate atualizado para 0xFF na porta de saida fisica");
        spi_read_and_check(12'h001, 16'h00FF, "Readback SAMPLE_RATE = 0xFF");

        spi_write_single(12'h001, 16'h0000);
        chk(sample_rate === 8'h00, "o_sample_rate atualizado para 0x00 na porta de saida fisica");
        spi_read_and_check(12'h001, 16'h0000, "Readback SAMPLE_RATE = 0x00");

        // ---------------------------------------------------------------------
        // Resumo Final
        // ---------------------------------------------------------------------
        $display("\n###################################################################################");
        if (total_errors == 0) begin
            $display("   STATUS FINAL: PASS! Todas as %0d verificacoes passaram com 0 erros! [CRC_HEADER_EN=%0d]",
                     total_checks, CRC_HEADER_EN);
            $display("\tPPPP   AA   SSS   SSS      !! !! !!");
            $display("\tP   P A  A S     S         !! !! !!");
            $display("\tPPPP  AAAA  SSS   SSS      !! !! !!");
            $display("\tP     A  A     S     S             ");
            $display("\tP     A  A SSSS  SSSS      !! !! !!");
        end else begin
            $display("   STATUS FINAL: FAIL! %0d erros encontrados em %0d verificacoes. [CRC_HEADER_EN=%0d]",
                     total_errors, total_checks, CRC_HEADER_EN);
        end
        $display("###################################################################################\n");
        $finish;
    end

endmodule
