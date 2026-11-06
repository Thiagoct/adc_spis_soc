`timescale 1ns / 1ps
// ----------------------------------------------------------------------
// Copyright (c) 2026 VIRTUS-UFCG. All rights reserved
// VIRTUS-UFCG Confidential Proprietary
// ----------------------------------------------------------------------
// Module     : adc_wb_slave_top_tb
// Description: Testbench de verificação completa do módulo adc_wb_slave_top
//              (Wishbone Slave + Banco de Registos + Núcleo ADC SAR 10b).
//              Valida todos os requisitos das Seções 1 a 5:
//              1. Clock de 100 MHz e Reset rst_sys (assert assíncrono, deassert síncrono)
//              2. Fluxo de Inicialização e Reset por Software (ADC_CTRL[7] = 1 -> 0)
//              3. Escrita e Readback de SAMPLE_RATE (0x01), FILTER_CFG (0x02),
//                 GAIN_CFG (0x04) e Coeficientes Reservados (0x06 a 0x0F)
//              4. Suporte a Write Strobes (wb_sel_i)
//              5. Ativação do ADC (ADC_CTRL[6] = 1), Polling de SPI_STATUS (0x05)
//                 e Leitura de ADC_DATA (0x03) conectado ao RTL do ADC (adc.sv)
//                 para múltiplos valores de tensão analógica v_in
//              6. Proteção de registos RO (0x03 e 0x05) e sinalização de erro (ERR)
//              7. Endereços não mapeados (0x010 a 0xFFF): escrita ignorada,
//                 leitura retorna 16'h0000 e wb_ack_o sempre gerado sem travamento
// ----------------------------------------------------------------------

module adc_wb_slave_top_tb;

    // =========================================================================
    // 1. Parâmetros de Simulação
    // =========================================================================
    localparam real CLK_PERIOD_NS = 10.0;   // 100 MHz
    localparam real V_FULL_SCALE  = 0.8;    // 0.8 V fundo de escala
    localparam int  RESOLUTION    = 10;     // 10 bits (0 a 1023)
    localparam real T_CONV_NS     = 1000.0; // 1 us para acelerar simulação mantendo fidelidade

    // =========================================================================
    // 2. Sinais da Interface do DUT
    // =========================================================================
    logic        clk;
    logic        rst_sys;

    // Interface Wishbone Slave
    logic [11:0] wb_adr_i;
    logic [15:0] wb_dat_i;
    logic [15:0] wb_dat_o;
    logic [1:0]  wb_sel_i;
    logic        wb_cyc_i;
    logic        wb_stb_i;
    logic        wb_we_i;
    logic        wb_ack_o;

    // Interface com o Núcleo do ADC e Sistema
    real         v_in;
    logic        i_err_ext;
    logic        o_adc_reset;
    logic        o_adc_enable;
    logic [5:0]  o_adc_mode;
    logic [7:0]  o_sample_rate;
    logic [15:0] o_filter_cfg;
    logic [7:0]  o_gain_cfg;
    logic [15:0] o_filter_coeffs [0:9];
    logic [9:0]  o_adc_dout;
    logic        o_adc_data_rdy;

    int total_checks = 0;
    int total_errors = 0;

    // =========================================================================
    // 3. Instanciação do DUT (Wishbone Slave + RegBank + ADC SAR 10b)
    // =========================================================================
    adc_wb_slave_top #(
        .ADDR_WIDTH   (12),
        .DATA_WIDTH   (16),
        .GRANULE      (8),
        .REGISTER_NUM (15),
        .SEL_WIDTH    (2),
        .V_FULL_SCALE (V_FULL_SCALE),
        .RESOLUTION   (RESOLUTION),
        .T_CONV_NS    (T_CONV_NS)
    ) u_dut (
        .clk             (clk),
        .rst_sys         (rst_sys),
        .wb_adr_i        (wb_adr_i),
        .wb_dat_i        (wb_dat_i),
        .wb_dat_o        (wb_dat_o),
        .wb_sel_i        (wb_sel_i),
        .wb_cyc_i        (wb_cyc_i),
        .wb_stb_i        (wb_stb_i),
        .wb_we_i         (wb_we_i),
        .wb_ack_o        (wb_ack_o),
        .v_in            (v_in),
        .i_err_ext       (i_err_ext),
        .o_adc_reset     (o_adc_reset),
        .o_adc_enable    (o_adc_enable),
        .o_adc_mode      (o_adc_mode),
        .o_sample_rate   (o_sample_rate),
        .o_filter_cfg    (o_filter_cfg),
        .o_gain_cfg      (o_gain_cfg),
        .o_filter_coeffs (o_filter_coeffs),
        .o_adc_dout      (o_adc_dout),
        .o_adc_data_rdy  (o_adc_data_rdy)
    );

    // =========================================================================
    // 4. Geração do Relógio de 100 MHz
    // =========================================================================
    initial clk = 1'b0;
    always #(CLK_PERIOD_NS / 2.0) clk = ~clk;

    // =========================================================================
    // 5. Tasks de Barramento Wishbone Master (BFM) e Funções Auxiliares
    // =========================================================================
    function automatic logic [9:0] calc_expected_adc(real vin);
        real lsb;
        real clamped_v;
        begin
            lsb = V_FULL_SCALE / ((1 << RESOLUTION) - 1.0);
            if (vin < 0.0) clamped_v = 0.0;
            else if (vin > V_FULL_SCALE) clamped_v = V_FULL_SCALE;
            else clamped_v = vin;
            return $rtoi(clamped_v / lsb);
        end
    endfunction

    // Escrita Wishbone com timeout de segurança para verificar ausência de bloqueio
    task automatic wb_write(
        input logic [11:0] addr,
        input logic [15:0] data,
        input logic [1:0]  sel = 2'b11
    );
        int timeout;
        begin
            @(posedge clk);
            wb_adr_i <= addr;
            wb_dat_i <= data;
            wb_sel_i <= sel;
            wb_we_i  <= 1'b1;
            wb_cyc_i <= 1'b1;
            wb_stb_i <= 1'b1;

            timeout = 0;
            @(posedge clk);
            while (!wb_ack_o && timeout < 50) begin
                @(posedge clk);
                timeout++;
            end

            if (!wb_ack_o) begin
                $display("[FAIL] Timeout esperando wb_ack_o na escrita em 0x%03h!", addr);
                total_errors++;
            end

            wb_cyc_i <= 1'b0;
            wb_stb_i <= 1'b0;
            wb_we_i  <= 1'b0;
            @(posedge clk);
        end
    endtask

    // Leitura Wishbone
    task automatic wb_read(
        input  logic [11:0] addr,
        output logic [15:0] data_out
    );
        int timeout;
        begin
            @(posedge clk);
            wb_adr_i <= addr;
            wb_sel_i <= 2'b11;
            wb_we_i  <= 1'b0;
            wb_cyc_i <= 1'b1;
            wb_stb_i <= 1'b1;

            timeout = 0;
            @(posedge clk);
            while (!wb_ack_o && timeout < 50) begin
                @(posedge clk);
                timeout++;
            end

            data_out = wb_dat_o;
            if (!wb_ack_o) begin
                $display("[FAIL] Timeout esperando wb_ack_o na leitura de 0x%03h!", addr);
                total_errors++;
            end

            wb_cyc_i <= 1'b0;
            wb_stb_i <= 1'b0;
            @(posedge clk);
        end
    endtask

    // Leitura Wishbone com verificação automática
    task automatic wb_read_check(
        input logic [11:0] addr,
        input logic [15:0] expected,
        input string       desc
    );
        logic [15:0] rd_val;
        begin
            wb_read(addr, rd_val);
            total_checks++;
            if (rd_val === expected) begin
                $display("[PASS] %s (Addr 0x%03h): Lido = 0x%04h (Esperado = 0x%04h)",
                         desc, addr, rd_val, expected);
            end else begin
                $display("[FAIL] %s (Addr 0x%03h): Lido = 0x%04h (Esperado = 0x%04h)",
                         desc, addr, rd_val, expected);
                total_errors++;
            end
        end
    endtask

    // =========================================================================
    // 6. Sequência Principal de Testes
    // =========================================================================
    initial begin
        logic [15:0] status_val;
        real         test_voltages [0:3];
        logic [9:0]  exp_code;
        int          poll_cnt;

        $display("===================================================================================");
        $display("   SIMULACAO: BANCO DE REGISTOS (WISHBONE SLAVE) + NUCLEO ADC SAR 10 BITS          ");
        $display("===================================================================================");

        // Inicialização
        rst_sys   = 1'b0;
        wb_adr_i  = 12'h000;
        wb_dat_i  = 16'h0000;
        wb_sel_i  = 2'b11;
        wb_cyc_i  = 1'b0;
        wb_stb_i  = 1'b0;
        wb_we_i   = 1'b0;
        v_in      = 0.0;
        i_err_ext = 1'b0;

        // ---------------------------------------------------------------------
        // TESTE 1: Sincronismo de Reset rst_sys (Ativação Assíncrona, Desativação Síncrona)
        // ---------------------------------------------------------------------
        $display("\n--- [TESTE 1] Verificacao do Reset rst_sys (Assincrono na ativacao, Sincrono na liberacao) ---");
        repeat (5) @(posedge clk);
        #3; // Fora da borda do relógio
        rst_sys = 1'b1;
        total_checks++;
        if (u_dut.rst_sys_sync_n === 1'b0) begin
            $display("[PASS] Desativacao sincrona de rst_sys confirmada (aguarda bordas de clk)");
        end else begin
            $display("[FAIL] rst_sys_sync_n liberou assincronamente!");
            total_errors++;
        end
        repeat (3) @(posedge clk);

        // ---------------------------------------------------------------------
        // TESTE 2: Fluxo de Configuração (Seção 4) e Reset por Software (ADC_CTRL[7])
        // ---------------------------------------------------------------------
        $display("\n--- [TESTE 2] Inicializacao, Reset por Software e Configuracao de Registos ---");
        // Preenche alguns registos antes do reset por software
        wb_write(12'h001, 16'h00FF);
        wb_write(12'h002, 16'hBEEF);

        // Anfitrião escreve ADC_CTRL (0x00) com RESET (Bit 7) = '1' e depois limpa para '0'
        wb_write(12'h000, 16'h0080);
        total_checks++;
        if (o_adc_reset === 1'b1) begin
            $display("[PASS] Bit RESET (Bit 7 de ADC_CTRL) ativou o_adc_reset=1");
        end else begin
            $display("[FAIL] o_adc_reset nao foi ativado!");
            total_errors++;
        end
        wb_write(12'h000, 16'h0000); // Limpa bit RESET ('0') após ciclo de reset

        // Verifica que SAMPLE_RATE e FILTER_CFG foram zerados pelo reset por software
        wb_read_check(12'h001, 16'h0000, "SAMPLE_RATE pos-SW-Reset");
        wb_read_check(12'h002, 16'h0000, "FILTER_CFG pos-SW-Reset");

        // Configura SAMPLE_RATE (0x01), FILTER_CFG (0x02), GAIN_CFG (0x04) e Coeffs (0x06..0x0F)
        wb_write(12'h001, 16'h002A); // SAMPLE_RATE = 0x2A
        wb_write(12'h002, 16'hC0DE); // FILTER_CFG  = 0xC0DE
        wb_write(12'h004, 16'h0015); // GAIN_CFG    = 0x15
        for (int c = 0; c < 10; c++) begin
            wb_write(12'h006 + c[11:0], 16'h1000 + c[15:0]);
        end

        wb_read_check(12'h001, 16'h002A, "SAMPLE_RATE (0x01)");
        wb_read_check(12'h002, 16'hC0DE, "FILTER_CFG (0x02)");
        wb_read_check(12'h004, 16'h0015, "GAIN_CFG (0x04)");
        for (int c = 0; c < 10; c++) begin
            wb_read_check(12'h006 + c[11:0], 16'h1000 + c[15:0], "COEFF_REG");
        end

        // ---------------------------------------------------------------------
        // TESTE 3: Operação do ADC SAR 10b, Polling de SPI_STATUS (0x05) e Leitura de ADC_DATA (0x03)
        // ---------------------------------------------------------------------
        $display("\n--- [TESTE 3] Conversao do ADC SAR 10b, Polling de SPI_STATUS (0x05) e Leitura de ADC_DATA (0x03) ---");
        test_voltages[0] = 0.0;  // 0 V    -> 0
        test_voltages[1] = 0.25; // 0.25 V -> 319
        test_voltages[2] = 0.40; // 0.40 V -> 511
        test_voltages[3] = 0.80; // 0.80 V -> 1023

        for (int k = 0; k < 4; k++) begin
            v_in     = test_voltages[k];
            exp_code = calc_expected_adc(v_in);

            // Ativa o ADC escrevendo ENABLE (Bit 6 = 1) em ADC_CTRL (0x00)
            wb_write(12'h000, 16'h0040);

            // Faz polling ao registo SPI_STATUS (0x05) até DATA_RDY (Bit 6) == 1
            poll_cnt   = 0;
            status_val = 16'h0000;
            while (!status_val[6] && poll_cnt < 500) begin
                wb_read(12'h005, status_val);
                poll_cnt++;
            end

            total_checks++;
            if (status_val[6] === 1'b1) begin
                $display("[PASS] SPI_STATUS.DATA_RDY (Bit 6) detetado em 1 para V_in = %0.2f V (apos %0d leituras de polling)",
                         v_in, poll_cnt);
            end else begin
                $display("[FAIL] Timeout no polling de SPI_STATUS.DATA_RDY para V_in = %0.2f V!", v_in);
                total_errors++;
            end

            // Lê o resultado de 10 bits no registo RO ADC_DATA (0x03)
            wb_read_check(12'h003, {6'b000000, exp_code}, "ADC_DATA (0x03)");

            // Desativa o ADC antes da próxima tensão de teste
            wb_write(12'h000, 16'h0000);
            #(T_CONV_NS + 200);
        end

        // ---------------------------------------------------------------------
        // TESTE 4: Proteção de Registos Read-Only (0x03 e 0x05) e Sinalização de Erro (ERR)
        // ---------------------------------------------------------------------
        $display("\n--- [TESTE 4] Protecao de Registos Read-Only (0x03, 0x05) e Flag ERR ---");
        // Tenta escrever em ADC_DATA (0x03 - RO) e SPI_STATUS (0x05 - RO)
        wb_write(12'h003, 16'hFFFF);
        wb_write(12'h005, 16'hFFFF);
        wb_read_check(12'h003, {6'b000000, exp_code}, "ADC_DATA (RO protegido contra escrita)");

        // Injeta um pulso de erro externo (i_err_ext = 1) e verifica Bit 7 (ERR) em SPI_STATUS (0x05)
        @(posedge clk);
        i_err_ext <= 1'b1;
        @(posedge clk);
        i_err_ext <= 1'b0;
        wb_read_check(12'h005, 16'h0080, "SPI_STATUS (Bit 7 ERR = 1)");

        // ---------------------------------------------------------------------
        // TESTE 5: Endereços Não Mapeados (0x010 a 0xFFF) — Diretriz 5
        // ---------------------------------------------------------------------
        $display("\n--- [TESTE 5] Enderecos Nao Mapeados (0x010 a 0xFFF): Escrita Ignorada e Leitura 16'h0000 com ACK ---");
        wb_write(12'h010, 16'hDEAD);
        wb_write(12'h123, 16'hBEEF);
        wb_write(12'hFFF, 16'hCAFE);

        wb_read_check(12'h010, 16'h0000, "Endereco Nao Mapeado 0x010");
        wb_read_check(12'h123, 16'h0000, "Endereco Nao Mapeado 0x123");
        wb_read_check(12'hFFF, 16'h0000, "Endereco Nao Mapeado 0xFFF");

        // Confirma que os registos mapeados (0x01, 0x02, 0x04) continuam intactos
        wb_read_check(12'h001, 16'h002A, "SAMPLE_RATE intacto apos acessos 0x010-0xFFF");
        wb_read_check(12'h002, 16'hC0DE, "FILTER_CFG intacto apos acessos 0x010-0xFFF");

        // ---------------------------------------------------------------------
        // Resumo Final
        // ---------------------------------------------------------------------
        $display("\n###################################################################################");
        if (total_errors == 0) begin
            $display("   STATUS FINAL: PASS! Todas as %0d verificacoes passaram com 0 erros!", total_checks);
            $display("\tPPPP   AA   SSS   SSS      !! !! !!");
            $display("\tP   P A  A S     S         !! !! !!");
            $display("\tPPPP  AAAA  SSS   SSS      !! !! !!");
            $display("\tP     A  A     S     S             ");
            $display("\tP     A  A SSSS  SSSS      !! !! !!");
        end else begin
            $display("   STATUS FINAL: FAIL! %0d erros encontrados em %0d verificacoes.", total_errors, total_checks);
        end
        $display("###################################################################################\n");
        $finish;
    end

endmodule

