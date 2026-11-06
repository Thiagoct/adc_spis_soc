// ----------------------------------------------------------------------
// Copyright (c) 2026 VIRTUS-UFCG. All rights reserved
// VIRTUS-UFCG Confidential Proprietary
// ----------------------------------------------------------------------
// Module     : adc_reg_bank
// Description: Banco de Registos (REG BANK) de 16 bits para configuração,
//              controlo e monitorização do ADC SAR de 10 bits:
//              - 0x00: ADC_CTRL    (R/W) Bit 7: RESET síncrono, Bit 6: ENABLE,
//                                        Bits [15:8],[5:0]: Reservados/Modo
//              - 0x01: SAMPLE_RATE (R/W) Bits [7:0]: Seleção da taxa de amostragem
//              - 0x02: FILTER_CFG  (R/W) Bits [15:0]: Parâmetros do filtro digital
//              - 0x03: ADC_DATA    (RO)  Bits [9:0]: Resultado da conversão ({6'b0, adc_dout[9:0]})
//              - 0x04: GAIN_CFG    (R/W) Bits [7:0]: Valor de controlo de ganho
//              - 0x05: SPI_STATUS  (RO)  Bit 7: ERR, Bit 6: DATA_RDY, restantes: 0
//              - 0x06..0x0F: Coeffs(R/W) Coeficientes programáveis de filtro / expansão
//              - 0x010..0xFFF: Não mapeados (Leitura retorna 16'h0000, escrita ignorada)
// ----------------------------------------------------------------------

module adc_reg_bank #(
    parameter int ADDR_WIDTH = 12,
    parameter int DATA_WIDTH = 16,
    parameter int SEL_WIDTH  = DATA_WIDTH / 8,
    parameter int NUM_REGS   = 16
) (
    input  logic                  i_clk,
    input  logic                  i_rst_n,

    // Interface de acesso vinda do wb_slave
    input  logic                  i_valid,
    input  logic                  i_write,
    input  logic [ADDR_WIDTH-1:0] i_addr,
    input  logic [DATA_WIDTH-1:0] i_data,
    input  logic [SEL_WIDTH-1:0]  i_sel,
    output logic [DATA_WIDTH-1:0] o_data,

    // Entradas de Estado (ADC / Sistema -> Registos)
    input  logic                  i_err_ext,      // Flag de erro externo (ex.: CRC mismatch)
    input  logic                  i_adc_data_rdy, // Sinal DATA_RDY vindo do núcleo do ADC
    input  logic [9:0]            i_adc_dout,     // Palavra de 10 bits convertida pelo ADC

    // Saídas de Configuração e Controlo (Registos -> ADC)
    output logic                  o_adc_reset,    // Bit 7 de ADC_CTRL (reset síncrono por software)
    output logic                  o_adc_enable,   // Bit 6 de ADC_CTRL (ativa/desativa o ADC)
    output logic [5:0]            o_adc_mode,     // Bits [5:0] de ADC_CTRL (seleção de modo)
    output logic [7:0]            o_sample_rate,  // Bits [7:0] de SAMPLE_RATE (0x01)
    output logic [15:0]           o_filter_cfg,   // Bits [15:0] de FILTER_CFG (0x02)
    output logic [7:0]            o_gain_cfg,     // Bits [7:0] de GAIN_CFG (0x04)
    output logic [15:0]           o_filter_coeffs [0:9] // Registos 0x06 a 0x0F
);

    // Endereços do Mapa de Registos (Seção 3 e 5)
    localparam logic [ADDR_WIDTH-1:0] ADDR_ADC_CTRL    = 12'h000;
    localparam logic [ADDR_WIDTH-1:0] ADDR_SAMPLE_RATE = 12'h001;
    localparam logic [ADDR_WIDTH-1:0] ADDR_FILTER_CFG  = 12'h002;
    localparam logic [ADDR_WIDTH-1:0] ADDR_ADC_DATA    = 12'h003;
    localparam logic [ADDR_WIDTH-1:0] ADDR_GAIN_CFG    = 12'h004;
    localparam logic [ADDR_WIDTH-1:0] ADDR_SPI_STATUS  = 12'h005;

    // Banco de registos interno (0x00 a 0x0F)
    logic [DATA_WIDTH-1:0] regs [0:NUM_REGS-1];
    logic                  r_err_latched;
    logic                  r_data_rdy_latched;
    logic [9:0]            r_adc_dout_latched;

    // Função auxiliar para aplicar máscara de bytes (write strobes wb_sel_i)
    function automatic logic [15:0] apply_wstrb (
        input logic [15:0] old_val,
        input logic [15:0] new_val,
        input logic [1:0]  sel
    );
        logic [15:0] res;
        begin
            res[7:0]  = sel[0] ? new_val[7:0]  : old_val[7:0];
            res[15:8] = sel[1] ? new_val[15:8] : old_val[15:8];
            return res;
        end
    endfunction

    wire [15:0] w_masked_data = apply_wstrb((i_addr < NUM_REGS) ? regs[i_addr] : 16'h0000, i_data, i_sel);

    // Lógica sequencial de escrita, captura de status e reset
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            for (int i = 0; i < NUM_REGS; i++) begin
                regs[i] <= 16'h0000;
            end
            r_err_latched      <= 1'b0;
            r_data_rdy_latched <= 1'b0;
            r_adc_dout_latched <= 10'd0;
        end else begin
            // Captura erro vindo da lógica de validação externa (ex.: erro de CRC)
            if (i_err_ext) begin
                r_err_latched <= 1'b1;
            end

            // Captura novo dado pronto proveniente do núcleo do ADC
            if (i_adc_data_rdy && !regs[ADDR_ADC_CTRL][7]) begin
                r_data_rdy_latched <= 1'b1;
                r_adc_dout_latched <= i_adc_dout;
            end

            // Processamento de transações válidas no intervalo 0x000 a 0x00F
            if (i_valid && (i_addr < NUM_REGS)) begin
                if (i_write) begin
                    if (i_addr == ADDR_ADC_CTRL) begin
                        regs[ADDR_ADC_CTRL] <= w_masked_data;
                        // Se o bit 7 (RESET) for escrito com '1', executa reset síncrono
                        if (w_masked_data[7]) begin
                            for (int i = 1; i < NUM_REGS; i++) begin
                                regs[i] <= 16'h0000;
                            end
                            r_err_latched      <= 1'b0;
                            r_data_rdy_latched <= 1'b0;
                            r_adc_dout_latched <= 10'd0;
                        end
                    end else if ((i_addr == ADDR_ADC_DATA) || (i_addr == ADDR_SPI_STATUS)) begin
                        // Registos 0x03 (ADC_DATA) e 0x05 (SPI_STATUS) são Read-Only (RO)
                        // Escritas são ignoradas para preservar integridade
                    end else begin
                        regs[i_addr] <= w_masked_data;
                    end
                end else begin
                    // Leitura de ADC_DATA (0x03) consome a flag de dado pronto se o pulso já encerrou
                    if (i_addr == ADDR_ADC_DATA && !i_adc_data_rdy) begin
                        r_data_rdy_latched <= 1'b0;
                    end
                end
            end
        end
    end

    // Flags dinâmicas e registos de leitura
    wire w_status_err      = r_err_latched | i_err_ext;
    wire w_status_data_rdy = (r_data_rdy_latched | i_adc_data_rdy) && !regs[ADDR_ADC_CTRL][7];
    wire [9:0] w_adc_dout  = i_adc_data_rdy ? i_adc_dout : r_adc_dout_latched;

    // Leitura combinacional (retorna 16'h0000 para qualquer endereço fora de 0x000-0x00F)
    always_comb begin
        if (i_addr < NUM_REGS) begin
            case (i_addr)
                ADDR_ADC_CTRL:    o_data = regs[ADDR_ADC_CTRL];
                ADDR_SAMPLE_RATE: o_data = regs[ADDR_SAMPLE_RATE];
                ADDR_FILTER_CFG:  o_data = regs[ADDR_FILTER_CFG];
                ADDR_ADC_DATA:    o_data = w_status_data_rdy ? {6'b000000, w_adc_dout} : {6'b000000, r_adc_dout_latched};
                ADDR_GAIN_CFG:    o_data = regs[ADDR_GAIN_CFG];
                ADDR_SPI_STATUS:  o_data = {8'h00, w_status_err, w_status_data_rdy, 6'b000000};
                default:          o_data = regs[i_addr];
            endcase
        end else begin
            o_data = 16'h0000;
        end
    end

    // Saídas contínuas de configuração e controlo para o núcleo do ADC
    assign o_adc_reset   = regs[ADDR_ADC_CTRL][7];
    assign o_adc_enable  = regs[ADDR_ADC_CTRL][6] && !regs[ADDR_ADC_CTRL][7];
    assign o_adc_mode    = regs[ADDR_ADC_CTRL][5:0];
    assign o_sample_rate = regs[ADDR_SAMPLE_RATE][7:0];
    assign o_filter_cfg  = regs[ADDR_FILTER_CFG];
    assign o_gain_cfg    = regs[ADDR_GAIN_CFG][7:0];

    genvar g;
    generate
        for (g = 0; g < 10; g++) begin : gen_filter_coeffs
            assign o_filter_coeffs[g] = regs[6 + g];
        end
    endgenerate

endmodule

