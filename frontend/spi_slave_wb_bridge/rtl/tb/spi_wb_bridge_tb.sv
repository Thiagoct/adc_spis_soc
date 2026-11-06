`timescale 1ns / 1ps
// ----------------------------------------------------------------------
// Copyright (c) 2026 VIRTUS-UFCG. All rights reserved
// VIRTUS-UFCG Confidential Proprietary
// ----------------------------------------------------------------------
// Module     : spi_wb_bridge_tb
// Description: Testbench unitário do Sub-bloco 2 (spi_wb_bridge) usando um
//              escravo Wishbone comportamental (memória de 16 x 16 bits).
//              O parâmetro CRC_HEADER_EN seleciona a Opção A (1) ou B (0);
//              rode o testbench nas duas configurações:
//                vsim -GCRC_HEADER_EN=0 spi_wb_bridge_tb
//                vsim -GCRC_HEADER_EN=1 spi_wb_bridge_tb
//
//              Testes:
//               [1] Escrita/Leitura simples
//               [2] Burst com auto-incremento
//               [3] 4 modos CPOL/CPHA x MSB/LSB
//               [4] CRC de dado invalido: escrita bloqueada + crc_err
//               [5] reset_s (6.3.1.12/13): assert assincrono, deassert sincrono
//                   e reset no meio de uma transacao
//               [6] CRC do cabecalho (6.3.3.14, so CRC_HEADER_EN=1):
//                   cabecalho invalido aborta escrita e leitura
//               [7] ss (6.3.1.14): abort no meio de endereco/dado/CRC/leitura
//                   e trafego espurio de sck/mosi com ss=1
//               [8] Burst de escrita com CRC invalido no meio da sequencia
// ----------------------------------------------------------------------

module spi_wb_bridge_tb #(
    parameter bit CRC_HEADER_EN = 1'b0
);

    localparam real CLK_PERIOD_NS = 10.0;   // 100 MHz
    localparam real SCK_HALF_NS   = 500.0;  // 1 MHz

    logic        clk, sck, reset_s, rst_sys;
    logic        cpol, cpha, bit_order;
    logic        ss, mosi, miso, spi_busy;

    logic [11:0] wb_adr;
    logic [15:0] wb_dat_m2s, wb_dat_s2m;
    logic [1:0]  wb_sel;
    logic        wb_we, wb_stb, wb_cyc, wb_ack;
    logic        crc_err;

    int total_errors = 0;
    int total_checks = 0;
    int crc_err_seen = 0;
    int wb_wr_cnt    = 0;   // escritas Wishbone concluidas
    int wb_rd_cnt    = 0;   // leituras Wishbone concluidas
    int wb_start_cnt = 0;   // ciclos Wishbone iniciados

    spi_wb_bridge #(
        .ADDR_WIDTH(12), .DATA_WIDTH(16), .SEL_WIDTH(2), .CRC_HEADER_EN(CRC_HEADER_EN)
    ) u_dut (
        .clk(clk), .sck(sck), .reset_s(reset_s), .rst_sys(rst_sys),
        .cpol(cpol), .cpha(cpha), .bit_order(bit_order),
        .ss(ss), .mosi(mosi), .miso(miso), .spi_busy(spi_busy),
        .wb_adr_o(wb_adr), .wb_dat_o(wb_dat_m2s), .wb_dat_i(wb_dat_s2m),
        .wb_sel_o(wb_sel), .wb_we_o(wb_we), .wb_stb_o(wb_stb),
        .wb_cyc_o(wb_cyc), .wb_ack_i(wb_ack), .crc_err(crc_err)
    );

    // Escravo Wishbone comportamental: memória 16 x 16, ACK em 1 ciclo
    logic [15:0] mem [0:15];
    always_ff @(posedge clk or negedge rst_sys) begin
        if (!rst_sys) begin
            wb_ack     <= 1'b0;
            wb_dat_s2m <= 16'h0000;
            for (int i = 0; i < 16; i++) mem[i] <= 16'h0000;
        end else begin
            wb_ack <= 1'b0;
            if (wb_cyc && wb_stb && !wb_ack) begin
                wb_ack <= 1'b1;
                if (wb_we) begin
                    if (wb_adr < 16) mem[wb_adr[3:0]] <= wb_dat_m2s;
                end else begin
                    wb_dat_s2m <= (wb_adr < 16) ? mem[wb_adr[3:0]] : 16'h0000;
                end
            end
        end
    end

    initial clk = 1'b0;
    always #(CLK_PERIOD_NS / 2.0) clk = ~clk;

    // Monitores
    always @(posedge clk) begin
        if (crc_err) crc_err_seen++;
        if (wb_cyc && wb_stb && wb_ack) begin
            if (wb_we) wb_wr_cnt++; else wb_rd_cnt++;
        end
    end
    always @(posedge wb_cyc) wb_start_cnt++;

    // ------------------------------------------------------------------
    // Funcoes e tasks auxiliares (BFM do mestre SPI)
    // ------------------------------------------------------------------
    function automatic logic [7:0] crc8_step_vec(input logic [23:0] vec, input int nbits);
        logic [7:0] crc; logic fb;
        begin
            crc = 8'h00;
            for (int i = nbits - 1; i >= 0; i--) begin
                fb  = crc[7] ^ vec[i];
                crc = {crc[6:0], 1'b0} ^ (fb ? 8'h07 : 8'h00);
            end
            return crc;
        end
    endfunction

    function automatic logic [7:0] calc_crc8_ccitt(input logic [15:0] d);
        return crc8_step_vec({8'h00, d}, 16);
    endfunction

    function automatic logic [7:0] calc_hdr_crc(input logic cmd, input logic [11:0] addr);
        return crc8_step_vec({11'h0, cmd, addr}, 13);
    endfunction

    task automatic chk(input bit cond, input string msg);
        begin
            total_checks++;
            if (cond) $display("[PASS] %s", msg);
            else begin $display("[FAIL] %s", msg); total_errors++; end
        end
    endtask

    task automatic spi_transfer_bit(input logic tx_b, output logic rx_b);
        logic lead_lvl, trail_lvl;
        begin
            lead_lvl  = (cpol == 1'b0) ? 1'b1 : 1'b0;
            trail_lvl = (cpol == 1'b0) ? 1'b0 : 1'b1;
            if (cpha == 1'b0) begin
                mosi = tx_b;  #(SCK_HALF_NS);
                sck  = lead_lvl; rx_b = miso; #(SCK_HALF_NS);
                sck  = trail_lvl;
            end else begin
                sck  = lead_lvl; mosi = tx_b; #(SCK_HALF_NS);
                sck  = trail_lvl; rx_b = miso; #(SCK_HALF_NS);
            end
        end
    endtask

    // Envia n bits de um vetor respeitando bit_order; verifica MISO = 0 (cabecalho/escrita)
    task automatic spi_send_bits(input logic [15:0] v, input int width, input int nbits, input bit expect_miso0 = 1'b1);
        logic rx_b; int idx;
        begin
            for (int i = 0; i < nbits; i++) begin
                idx = bit_order ? (width - 1 - i) : i;
                spi_transfer_bit(v[idx], rx_b);
                if (expect_miso0 && rx_b !== 1'b0) begin
                    $display("[ERRO] MISO != 0 durante cabecalho/escrita"); total_errors++;
                end
            end
        end
    endtask

    // Cabecalho: cmd + 12b addr (+ 8b CRC se CRC_HEADER_EN). nbits_addr<12 -> cabecalho truncado
    task automatic spi_send_header(input logic cmd_read, input logic [11:0] addr,
                                   input bit bad_hdr_crc = 1'b0, input int nbits_addr = 12);
        logic rx_b; logic [7:0] hc;
        begin
            spi_transfer_bit(cmd_read, rx_b);
            if (rx_b !== 1'b0) begin $display("[ERRO] MISO != 0 no comando"); total_errors++; end
            spi_send_bits({4'h0, addr}, 12, nbits_addr);
            if (CRC_HEADER_EN && nbits_addr == 12) begin
                hc = calc_hdr_crc(cmd_read, addr);
                if (bad_hdr_crc) hc = ~hc;
                spi_send_bits({8'h00, hc}, 8, 8);
            end
        end
    endtask

    task automatic spi_send_write_packet(input logic [15:0] d, input logic [7:0] c);
        begin
            spi_send_bits(d, 16, 16);
            spi_send_bits({8'h00, c}, 8, 8);
        end
    endtask

    task automatic spi_recv_read_packet(output logic [15:0] d, output logic [7:0] c);
        logic rx_b; int idx;
        begin
            d = 16'h0; c = 8'h0;
            for (int i = 0; i < 16; i++) begin
                idx = bit_order ? (15 - i) : i;
                spi_transfer_bit(1'b0, rx_b); d[idx] = rx_b;
            end
            for (int i = 0; i < 8; i++) begin
                idx = bit_order ? (7 - i) : i;
                spi_transfer_bit(1'b0, rx_b); c[idx] = rx_b;
            end
        end
    endtask

    task automatic spi_begin();
        begin
            sck = cpol; #(SCK_HALF_NS); ss = 1'b0; #(SCK_HALF_NS);
        end
    endtask

    task automatic spi_end();
        begin
            #(SCK_HALF_NS); ss = 1'b1; mosi = 1'b0; #(SCK_HALF_NS * 2);
        end
    endtask

    task automatic spi_write_single(input logic [11:0] addr, input logic [15:0] d,
                                    input bit bad_crc = 1'b0, input bit bad_hdr_crc = 1'b0);
        logic [7:0] c;
        begin
            c = calc_crc8_ccitt(d); if (bad_crc) c = ~c;
            spi_begin();
            if (spi_busy !== 1'b0) begin $display("[ERRO] spi_busy deveria ser 0"); total_errors++; end
            spi_send_header(1'b0, addr, bad_hdr_crc);
            spi_send_write_packet(d, c);
            spi_end();
        end
    endtask

    task automatic spi_read_and_check(input logic [11:0] addr, input logic [15:0] exp, input string name);
        logic [15:0] d; logic [7:0] c;
        begin
            spi_begin();
            spi_send_header(1'b1, addr);
            spi_recv_read_packet(d, c);
            spi_end();
            total_checks++;
            if (d === exp && c === calc_crc8_ccitt(exp))
                $display("[PASS] %s (0x%03h): 0x%04h CRC=0x%02h (CPOL=%0b CPHA=%0b MSB=%0b)", name, addr, d, c, cpol, cpha, bit_order);
            else begin
                $display("[FAIL] %s (0x%03h): obtido 0x%04h CRC=0x%02h, esperado 0x%04h", name, addr, d, c, exp);
                total_errors++;
            end
        end
    endtask

    // Aguarda e verifica FSM ociosa e spi_busy = 1
    task automatic check_idle(input string msg);
        begin
            repeat (10) @(posedge clk);
            chk(spi_busy === 1'b1 && miso === 1'b0 && u_dut.u_spi_slave_core.state === 3'd0, msg);
        end
    endtask

    // ------------------------------------------------------------------
    // Sequencia principal
    // ------------------------------------------------------------------
    logic [15:0] bt [0:3];
    logic [15:0] rd; logic [7:0] rc;
    logic [15:0] tv;
    int          snap_wr, snap_rd, snap_start, bad_idx;
    logic [7:0]  c8;
    logic [15:0] mem_snap [0:15];
    bit          mem_ok;

    initial begin
        $display("=== TB: spi_wb_bridge (Sub-bloco 2) | CRC_HEADER_EN=%0d ===", CRC_HEADER_EN);
        reset_s = 0; rst_sys = 0; cpol = 0; cpha = 0; bit_order = 1;
        sck = 0; ss = 1; mosi = 0;
        #(CLK_PERIOD_NS * 10); reset_s = 1; rst_sys = 1; #(CLK_PERIOD_NS * 10);

        chk(spi_busy === 1'b1 && miso === 1'b0, "Ocioso: spi_busy=1, miso=0");

        // ---------------------------------------------------------------
        $display("\n--- [1] Escrita/Leitura simples ---");
        spi_write_single(12'h001, 16'h003C);
        spi_write_single(12'h002, 16'hA55A);
        spi_read_and_check(12'h001, 16'h003C, "REG1");
        spi_read_and_check(12'h002, 16'hA55A, "REG2");

        // ---------------------------------------------------------------
        $display("\n--- [2] Burst com auto-incremento (0x006..0x009) ---");
        bt[0] = 16'hCAFE; bt[1] = 16'hBEEF; bt[2] = 16'hDEAD; bt[3] = 16'hF00D;
        spi_begin();
        spi_send_header(1'b0, 12'h006);
        for (int w = 0; w < 4; w++) spi_send_write_packet(bt[w], calc_crc8_ccitt(bt[w]));
        spi_end();
        spi_begin();
        spi_send_header(1'b1, 12'h006);
        for (int w = 0; w < 4; w++) begin
            spi_recv_read_packet(rd, rc);
            chk(rd === bt[w] && rc === calc_crc8_ccitt(bt[w]), $sformatf("Burst palavra %0d: 0x%04h", w, rd));
        end
        spi_end();

        // ---------------------------------------------------------------
        $display("\n--- [3] Modos CPOL/CPHA x bit_order ---");
        for (int mode = 0; mode < 4; mode++) begin
            for (int bo = 0; bo < 2; bo++) begin
                cpol = mode[1]; cpha = mode[0]; bit_order = bo[0]; sck = cpol;
                #(SCK_HALF_NS * 2);
                tv = 16'h5000 | (mode << 8) | (bo << 4) | 16'h000A;
                spi_write_single(12'h00A + mode[1:0], tv);
                spi_read_and_check(12'h00A + mode[1:0], tv, "MODE_TEST");
            end
        end
        cpol = 0; cpha = 0; bit_order = 1; sck = 0; #(SCK_HALF_NS * 2);

        // ---------------------------------------------------------------
        $display("\n--- [4] CRC de dado invalido: escrita bloqueada e pulso em crc_err ---");
        crc_err_seen = 0;
        spi_write_single(12'h002, 16'hFFFF, 1'b1);
        chk(crc_err_seen > 0, "crc_err sinalizado");
        spi_read_and_check(12'h002, 16'hA55A, "REG2 protegido");

        // ---------------------------------------------------------------
        $display("\n--- [5] reset_s: assert assincrono / deassert sincrono / reset no meio da transacao ---");
        // 5a: assert assincrono (cai antes da proxima borda de clk)
        chk(u_dut.rst_s_sync_n === 1'b1, "reset_s: sincronizado em 1 antes do teste");
        @(posedge clk); #3 reset_s = 1'b0; #1;
        chk(u_dut.rst_s_sync_n === 1'b0, "reset_s: assert ASSINCRONO (rst_s_sync_n=0 sem borda de clk)");
        #(CLK_PERIOD_NS * 3);
        chk(u_dut.rst_s_sync_n === 1'b0, "reset_s: mantido em 0 enquanto reset_s=0");
        // 5b: deassert sincrono (2 bordas de clk)
        @(posedge clk); #3 reset_s = 1'b1; #1;
        chk(u_dut.rst_s_sync_n === 1'b0, "reset_s: deassert NAO assincrono (ainda 0 logo apos reset_s=1)");
        @(posedge clk); #1;
        chk(u_dut.rst_s_sync_n === 1'b0, "reset_s: deassert sincrono, ainda 0 apos 1a borda");
        @(posedge clk); #1;
        chk(u_dut.rst_s_sync_n === 1'b1, "reset_s: deassert sincrono, 1 apos 2a borda");

        // 5c: reset_s no meio de uma transacao de escrita
        spi_write_single(12'h00C, 16'h1234);       // valor de referencia
        snap_start = wb_start_cnt;
        spi_begin();
        spi_send_header(1'b0, 12'h00C);
        spi_send_bits(16'h9999, 16, 5);              // 5 bits do dado
        @(posedge clk); #3 reset_s = 1'b0;
        #(CLK_PERIOD_NS * 3);
        chk(u_dut.u_spi_slave_core.state === 3'd0 && miso === 1'b0,
            "reset_s no meio da transacao: FSM volta a ST_IDLE, miso=0");
        @(posedge clk); #3 reset_s = 1'b1;
        #(CLK_PERIOD_NS * 5);
        spi_end();
        check_idle("reset_s no meio da transacao: apos ss=1, spi_busy=1 e FSM ociosa");
        chk(wb_start_cnt == snap_start, "reset_s no meio da transacao: nenhuma transacao Wishbone gerada");
        spi_read_and_check(12'h00C, 16'h1234, "REG 0x00C intacto apos reset_s");
        spi_write_single(12'h00D, 16'hBEEF);
        spi_read_and_check(12'h00D, 16'hBEEF, "Transacao normal apos reset_s");

        // ---------------------------------------------------------------
        $display("\n--- [6] CRC do cabecalho (6.3.3.14) ---");
        if (CRC_HEADER_EN) begin
            // Cabecalho invalido + dado com CRC correto: nao deve escrever
            spi_write_single(12'h00E, 16'h0BAD);      // valor inicial limpo para 0x00E
            spi_write_single(12'h00E, 16'h1111);
            snap_start = wb_start_cnt; snap_wr = wb_wr_cnt; crc_err_seen = 0;
            for (int i = 0; i < 16; i++) mem_snap[i] = mem[i];
            spi_write_single(12'h00E, 16'hFFFF, 1'b0, 1'b1);
            mem_ok = 1'b1;
            for (int i = 0; i < 16; i++) if (mem[i] !== mem_snap[i]) mem_ok = 1'b0;
            check_idle("Cabecalho invalido (escrita): FSM volta a ociosa");
            chk(crc_err_seen > 0, "Cabecalho invalido (escrita): crc_err sinalizado");
            chk(wb_wr_cnt == snap_wr && wb_start_cnt == snap_start, "Cabecalho invalido (escrita): nenhuma transacao Wishbone");
            spi_read_and_check(12'h00E, 16'h1111, "Cabecalho invalido: 0x00E nao foi alterado");
            chk(mem_ok, "Cabecalho invalido: nenhum outro endereco foi escrito (varredura da memoria)");
            // Cabecalho invalido na LEITURA: nada sai pelo MISO e nenhuma leitura Wishbone
            snap_rd = wb_rd_cnt; crc_err_seen = 0;
            spi_begin();
            spi_send_header(1'b1, 12'h00E, 1'b1);
            spi_recv_read_packet(rd, rc);
            spi_end();
            chk(rd === 16'h0000 && rc === 8'h00 && wb_rd_cnt == snap_rd, "Cabecalho invalido (leitura): MISO=0 e sem leitura Wishbone");
            chk(crc_err_seen > 0, "Cabecalho invalido (leitura): crc_err sinalizado");
            // Recuperacao: cabecalho valido volta a funcionar
            spi_write_single(12'h00E, 16'h2222);
            spi_read_and_check(12'h00E, 16'h2222, "Cabecalho valido apos erro: escrita OK");
        end else begin
            $display("[N/A ] CRC_HEADER_EN=0 (Opcao B): sem CRC de cabecalho. Rode com -GCRC_HEADER_EN=1");
        end

        // ---------------------------------------------------------------
        $display("\n--- [7] ss (6.3.1.14): abort no meio do pacote e trafego espurio com ss=1 ---");
        spi_write_single(12'h003, 16'h7777);   // valor de referencia em 0x003
        // 7a: abort no meio do endereco (5 de 12 bits)
        snap_start = wb_start_cnt;
        spi_begin();
        spi_send_header(1'b0, 12'h003, 1'b0, 5);
        spi_end();
        check_idle("Abort no meio do endereco: FSM ociosa e spi_busy=1");
        chk(wb_start_cnt == snap_start, "Abort no meio do endereco: nenhuma transacao Wishbone");
        // 7b: abort no meio do dado de escrita (8 de 16 bits)
        spi_begin();
        spi_send_header(1'b0, 12'h003);
        spi_send_bits(16'hFFFF, 16, 8);
        spi_end();
        check_idle("Abort no meio do dado (8/16 bits): FSM ociosa e spi_busy=1");
        chk(mem[3] === 16'h7777, "Abort no meio do dado: palavra incompleta descartada");
        // 7c: abort no meio do CRC do dado (16 dados + 4 de 8 bits de CRC)
        snap_wr = wb_wr_cnt;
        spi_begin();
        spi_send_header(1'b0, 12'h003);
        spi_send_bits(16'hFFFF, 16, 16);
        spi_send_bits({8'h00, calc_crc8_ccitt(16'hFFFF)}, 8, 4);
        spi_end();
        check_idle("Abort no meio do CRC: FSM ociosa e spi_busy=1");
        chk(wb_wr_cnt == snap_wr && mem[3] === 16'h7777, "Abort no meio do CRC: nenhuma escrita Wishbone");
        // 7d: abort no meio da leitura (8 de 24 bits)
        spi_begin();
        spi_send_header(1'b1, 12'h003);
        begin
            logic rx_b;
            for (int i = 0; i < 8; i++) spi_transfer_bit(1'b0, rx_b);
        end
        spi_end();
        check_idle("Abort no meio da leitura: FSM ociosa, miso=0 e spi_busy=1");
        // 7e: trafego espurio com ss=1 (sck + mosi aleatorios)
        snap_start = wb_start_cnt;
        ss = 1'b1;
        for (int k = 0; k < 60; k++) begin
            mosi = $urandom_range(0, 1);
            #(SCK_HALF_NS); sck = ~sck;
        end
        sck = cpol; mosi = 1'b0;
        #(SCK_HALF_NS);
        chk(wb_start_cnt == snap_start && miso === 1'b0 && spi_busy === 1'b1 && u_dut.u_spi_slave_core.state === 3'd0,
            "Trafego espurio com ss=1 ignorado (sem Wishbone, miso=0, spi_busy=1, FSM ociosa)");
        // 7f: transacoes normais continuam funcionando
        spi_write_single(12'h004, 16'h4321);
        spi_read_and_check(12'h004, 16'h4321, "Transacao normal apos abortos e trafego espurio");
        spi_read_and_check(12'h003, 16'h7777, "0x003 continua intacto");

        // ---------------------------------------------------------------
        $display("\n--- [8] Burst de escrita com CRC invalido na palavra 2 (0x006..0x009) ---");
        spi_write_single(12'h008, 16'h1111);       // valor de referencia no endereco da palavra 2
        bt[0] = 16'hCAFE; bt[1] = 16'hBEEF; bt[2] = 16'hDEAD; bt[3] = 16'hF00D;
        bad_idx = 2;
        snap_wr = wb_wr_cnt; crc_err_seen = 0;
        spi_begin();
        spi_send_header(1'b0, 12'h006);
        for (int w = 0; w < 4; w++) begin
            c8 = calc_crc8_ccitt(bt[w]);
            if (w == bad_idx) c8 = ~c8;
            spi_send_write_packet(bt[w], c8);
        end
        spi_end();
        check_idle("Burst com CRC ruim: FSM ociosa e spi_busy=1");
        chk(crc_err_seen > 0, "Burst com CRC ruim: crc_err sinalizado");
        chk(wb_wr_cnt - snap_wr == 3, "Burst com CRC ruim: apenas 3 das 4 palavras escritas no Wishbone");
        // Comportamento da FSM: bloqueia so a palavra ruim; burst continua; auto-incremento por pacote
        spi_read_and_check(12'h006, 16'hCAFE, "Burst CRC ruim: palavra 0 escrita em 0x006");
        spi_read_and_check(12'h007, 16'hBEEF, "Burst CRC ruim: palavra 1 escrita em 0x007");
        spi_read_and_check(12'h008, 16'h1111, "Burst CRC ruim: palavra 2 BLOQUEADA (0x008 inalterado)");
        spi_read_and_check(12'h009, 16'hF00D, "Burst CRC ruim: palavra 3 escrita em 0x009 (auto-incremento por pacote)");

        $display("\n###################################################################");
        if (total_errors == 0) $display("   STATUS FINAL: PASS (%0d verificacoes, 0 erros) [CRC_HEADER_EN=%0d]", total_checks, CRC_HEADER_EN);
        else                   $display("   STATUS FINAL: FAIL (%0d erros em %0d verificacoes) [CRC_HEADER_EN=%0d]", total_errors, total_checks, CRC_HEADER_EN);
        $display("###################################################################");
        $finish;
    end

endmodule
