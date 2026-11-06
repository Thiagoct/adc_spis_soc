// ----------------------------------------------------------------------
// Copyright (c) 2026 VIRTUS-UFCG. All rights reserved
// VIRTUS-UFCG Confidential Proprietary
// ----------------------------------------------------------------------
// Module     : spi_slave_core
// Description: Núcleo SPI Slave com suporte aos 4 modos (CPOL/CPHA),
//              ordem de bits configurável (MSB/LSB primeiro), operações
//              Single e Burst (Read/Write) e validação CRC8-CCITT.
//              - Opção B (CRC_HEADER_EN = 0, padrão Seção 6.3.4):
//                Cabeçalho de 13 bits [1b Cmd + 12b Addr] + pacotes de
//                24 bits [16b Data + 8b CRC].
//              - Opção A (CRC_HEADER_EN = 1, Seção 6.3.3.14):
//                Cabeçalho de 21 bits [1b Cmd + 12b Addr + 8b CRC] +
//                pacotes de 24 bits [16b Data + 8b CRC].
// ----------------------------------------------------------------------

module spi_slave_core #(
    parameter bit CRC_HEADER_EN = 1'b0
) (
    // Clock e Reset (sincronizado)
    input  logic        clk,
    input  logic        rst_s_n,

    // Sinais de configuração (síncronos a clk)
    input  logic        cpol,
    input  logic        cpha,
    input  logic        bit_order, // 0: LSB primeiro, 1: MSB primeiro

    // Interface SPI Slave
    input  logic        sck,
    input  logic        ss,
    input  logic        mosi,
    output logic        miso,
    output logic        spi_busy,  // Ativo em nível baixo (0 = ocupado, 1 = livre)

    // Interface com o módulo CRC8-CCITT
    output logic [15:0] o_crc_tx_data,
    input  logic [7:0]  i_crc_tx_val,
    output logic [15:0] o_crc_rx_data,
    output logic [7:0]  o_crc_rx_val,
    input  logic        i_crc_rx_data_ok,
    output logic [12:0] o_crc_rx_hdr,
    output logic [7:0]  o_crc_rx_hdr_val,
    input  logic        i_crc_rx_hdr_ok,

    // Interface de controle para o Wishbone Master
    output logic        o_cmd_load_addr,
    output logic [11:0] o_init_addr,
    output logic        o_wb_wr_req,
    output logic [15:0] o_wb_wr_data,
    output logic        o_wb_rd_req,
    output logic        o_wb_addr_skip, // Pulso: pacote de escrita descartado (CRC invalido), mantem auto-incremento
    input  logic        i_wb_rd_valid,
    input  logic [15:0] i_wb_rd_data,
    input  logic        i_wb_busy,

    // Sinalização de erro de CRC
    output logic        o_crc_err
);

    // ------------------------------------------------------------------
    // 1. Sincronizadores de CDC (sck, ss, mosi -> domínio clk)
    // ------------------------------------------------------------------
    logic sck_ff1, sck_ff2, sck_prev;
    logic ss_ff1,  ss_ff2,  ss_prev;
    logic mosi_ff1, mosi_ff2;

    always_ff @(posedge clk or negedge rst_s_n) begin
        if (!rst_s_n) begin
            sck_ff1  <= 1'b0;
            sck_ff2  <= 1'b0;
            sck_prev <= 1'b0;
            ss_ff1   <= 1'b1;
            ss_ff2   <= 1'b1;
            ss_prev  <= 1'b1;
            mosi_ff1 <= 1'b0;
            mosi_ff2 <= 1'b0;
        end else begin
            sck_ff1  <= sck;
            sck_ff2  <= sck_ff1;
            ss_ff1   <= ss;
            ss_ff2   <= ss_ff1;
            ss_prev  <= ss_ff2;
            mosi_ff1 <= mosi;
            mosi_ff2 <= mosi_ff1;

            if (ss_ff2) begin
                sck_prev <= sck_ff2;
            end else begin
                sck_prev <= sck_ff2;
            end
        end
    end

    wire ss_active   = !ss_ff2;
    wire sck_posedge = ss_active && (!ss_prev) && ( sck_ff2 && !sck_prev);
    wire sck_negedge = ss_active && (!ss_prev) && (!sck_ff2 &&  sck_prev);

    // Definição das bordas inicial (leading) e final (trailing) conforme CPOL
    wire leading_edge  = (cpol == 1'b0) ? sck_posedge : sck_negedge;
    wire trailing_edge = (cpol == 1'b0) ? sck_negedge : sck_posedge;

    // Definição das bordas de captura (sample) e lançamento (launch) conforme CPHA
    wire sample_edge   = (cpha == 1'b0) ? leading_edge  : trailing_edge;
    wire launch_edge   = (cpha == 1'b0) ? trailing_edge : leading_edge;

    // ------------------------------------------------------------------
    // 2. Máquina de Estados do Protocolo SPI Slave
    // ------------------------------------------------------------------
    typedef enum logic [2:0] {
        ST_IDLE       = 3'd0,
        ST_CMD        = 3'd1,
        ST_ADDR       = 3'd2,
        ST_HDR_CRC    = 3'd3,
        ST_WRITE_DATA = 3'd4,
        ST_READ_DATA  = 3'd5,
        ST_ERROR      = 3'd6
    } spi_state_t;

    spi_state_t state;

    logic        r_cmd;         // 0: Write, 1: Read
    logic [11:0] r_addr;        // Endereço recebido no cabeçalho
    logic [7:0]  r_hdr_crc;     // CRC do cabeçalho (quando CRC_HEADER_EN = 1)
    logic [15:0] r_rx_data;     // Dados de escrita recebidos
    logic [7:0]  r_rx_crc;      // CRC de dados recebido
    logic [15:0] r_tx_data;     // Dados de leitura para serializar no MISO
    logic [7:0]  r_tx_crc;      // CRC calculado dos dados de leitura
    logic [4:0]  r_bit_cnt;     // Contador de bits de recepção (sample_edge)
    logic [4:0]  r_tx_bit_idx;  // Índice do bit a ser lançado no MISO (launch_edge)

    // Próximos valores combinacionais no momento do último bit de cada campo
    logic [11:0] next_addr;
    logic [7:0]  next_hdr_crc;
    logic [7:0]  next_rx_crc;

    always_comb begin
        next_addr    = r_addr;
        next_hdr_crc = r_hdr_crc;
        next_rx_crc  = r_rx_crc;

        if (bit_order) begin
            next_addr[11 - r_bit_cnt[3:0]]   = mosi_ff2;
            next_hdr_crc[7 - r_bit_cnt[2:0]] = mosi_ff2;
        end else begin
            next_addr[r_bit_cnt[3:0]]        = mosi_ff2;
            next_hdr_crc[r_bit_cnt[2:0]]     = mosi_ff2;
        end

        if (r_bit_cnt >= 5'd16) begin
            if (bit_order)
                next_rx_crc[7 - (r_bit_cnt - 5'd16)] = mosi_ff2;
            else
                next_rx_crc[r_bit_cnt - 5'd16]     = mosi_ff2;
        end
    end

    // Conexões com o módulo de CRC
    assign o_crc_tx_data    = i_wb_rd_data;
    assign o_crc_rx_data    = r_rx_data;
    assign o_crc_rx_val     = next_rx_crc;
    assign o_crc_rx_hdr     = {r_cmd, (state == ST_ADDR) ? next_addr : r_addr};
    assign o_crc_rx_hdr_val = next_hdr_crc;

    // Seleção do bit de transmissão (16 bits de dados seguidos por 8 bits de CRC)
    function automatic logic select_tx_bit (
        input logic [15:0] data_w,
        input logic [7:0]  crc_w,
        input logic [4:0]  bit_idx,
        input logic        msb_first
    );
        logic [2:0] crc_idx;
        begin
            if (bit_idx < 5'd16) begin
                select_tx_bit = msb_first ? data_w[15 - bit_idx[3:0]] : data_w[bit_idx[3:0]];
            end else begin
                crc_idx       = bit_idx[4:0] - 5'd16;
                select_tx_bit = msb_first ? crc_w[7 - crc_idx] : crc_w[crc_idx];
            end
        end
    endfunction

    // Flag spi_busy ativa em nível baixo (0 durante transação SPI ou acesso Wishbone)
    assign spi_busy = !(ss_active || i_wb_busy);

    // ------------------------------------------------------------------
    // 3. Lógica Sequencial da FSM e Datapath
    // ------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_s_n) begin
        if (!rst_s_n) begin
            state           <= ST_IDLE;
            r_cmd           <= 1'b0;
            r_addr          <= 12'h000;
            r_hdr_crc       <= 8'h00;
            r_rx_data       <= 16'h0000;
            r_rx_crc        <= 8'h00;
            r_tx_data       <= 16'h0000;
            r_tx_crc        <= 8'h00;
            r_bit_cnt       <= 5'd0;
            r_tx_bit_idx    <= 5'd0;
            miso            <= 1'b0;
            o_cmd_load_addr <= 1'b0;
            o_init_addr     <= 12'h000;
            o_wb_wr_req     <= 1'b0;
            o_wb_wr_data    <= 16'h0000;
            o_wb_rd_req     <= 1'b0;
            o_wb_addr_skip  <= 1'b0;
            o_crc_err       <= 1'b0;
        end else begin
            // Pulsos de 1 ciclo por padrão
            o_cmd_load_addr <= 1'b0;
            o_wb_wr_req     <= 1'b0;
            o_wb_rd_req     <= 1'b0;
            o_wb_addr_skip  <= 1'b0;

            // Captura os dados lidos do barramento Wishbone quando prontos
            if (i_wb_rd_valid) begin
                r_tx_data <= i_wb_rd_data;
                r_tx_crc  <= i_crc_tx_val;
            end

            // Quando SS está desativado (1), mantém a interface em repouso e MISO em '0'
            if (!ss_active) begin
                state        <= ST_IDLE;
                r_bit_cnt    <= 5'd0;
                r_tx_bit_idx <= 5'd0;
                miso         <= 1'b0;
            end else begin
                case (state)
                    ST_IDLE: begin
                        miso         <= 1'b0;
                        r_bit_cnt    <= 5'd0;
                        r_tx_bit_idx <= 5'd0;
                        o_crc_err    <= 1'b0;
                        // Se a primeira borda de sample já ocorrer neste ciclo, captura o comando
                        if (sample_edge) begin
                            r_cmd     <= mosi_ff2; // 0 = Write, 1 = Read
                            r_bit_cnt <= 5'd0;
                            state     <= ST_ADDR;
                        end else begin
                            state     <= ST_CMD;
                        end
                    end

                    ST_CMD: begin
                        miso <= 1'b0;
                        if (sample_edge) begin
                            r_cmd     <= mosi_ff2; // 0 = Write, 1 = Read
                            r_bit_cnt <= 5'd0;
                            state     <= ST_ADDR;
                        end
                    end

                    ST_ADDR: begin
                        miso <= 1'b0; // MISO constante em '0' durante o campo de endereço
                        if (sample_edge) begin
                            r_addr <= next_addr;
                            if (r_bit_cnt == 5'd11) begin
                                r_bit_cnt <= 5'd0;
                                if (CRC_HEADER_EN) begin
                                    state <= ST_HDR_CRC;
                                end else begin
                                    o_init_addr     <= next_addr;
                                    o_cmd_load_addr <= 1'b1;
                                    if (r_cmd == 1'b1) begin
                                        // Leitura: dispara leitura inicial no Wishbone
                                        o_wb_rd_req  <= 1'b1;
                                        r_tx_bit_idx <= 5'd0;
                                        state        <= ST_READ_DATA;
                                    end else begin
                                        // Escrita: aguarda pacotes de 24 bits (16b Data + 8b CRC)
                                        state        <= ST_WRITE_DATA;
                                    end
                                end
                            end else begin
                                r_bit_cnt <= r_bit_cnt + 5'd1;
                            end
                        end
                    end

                    ST_HDR_CRC: begin
                        miso <= 1'b0;
                        if (sample_edge) begin
                            r_hdr_crc <= next_hdr_crc;
                            if (r_bit_cnt == 5'd7) begin
                                r_bit_cnt <= 5'd0;
                                if (i_crc_rx_hdr_ok) begin
                                    o_init_addr     <= r_addr;
                                    o_cmd_load_addr <= 1'b1;
                                    if (r_cmd == 1'b1) begin
                                        o_wb_rd_req  <= 1'b1;
                                        r_tx_bit_idx <= 5'd0;
                                        state        <= ST_READ_DATA;
                                    end else begin
                                        state        <= ST_WRITE_DATA;
                                    end
                                end else begin
                                    o_crc_err <= 1'b1;
                                    state     <= ST_ERROR;
                                end
                            end else begin
                                r_bit_cnt <= r_bit_cnt + 5'd1;
                            end
                        end
                    end

                    ST_WRITE_DATA: begin
                        miso <= 1'b0; // MISO constante em '0' durante toda operação de escrita
                        if (sample_edge) begin
                            if (r_bit_cnt < 5'd16) begin
                                if (bit_order)
                                    r_rx_data[15 - r_bit_cnt[3:0]] <= mosi_ff2;
                                else
                                    r_rx_data[r_bit_cnt[3:0]]      <= mosi_ff2;
                                r_bit_cnt <= r_bit_cnt + 5'd1;
                            end else begin
                                r_rx_crc <= next_rx_crc;
                                if (r_bit_cnt == 5'd23) begin
                                    // Fim do pacote de 24 bits (16b Data + 8b CRC)
                                    r_bit_cnt <= 5'd0;
                                    if (i_crc_rx_data_ok) begin
                                        o_crc_err    <= 1'b0;
                                        o_wb_wr_data <= r_rx_data;
                                        o_wb_wr_req  <= 1'b1;
                                    end else begin
                                        o_crc_err    <= 1'b1;
                                        o_wb_addr_skip <= 1'b1;
                                    end
                                end else begin
                                    r_bit_cnt <= r_bit_cnt + 5'd1;
                                end
                            end
                        end
                    end

                    ST_READ_DATA: begin
                        // Na borda de lançamento (launch_edge), atualiza o pino MISO
                        if (launch_edge) begin
                            miso <= select_tx_bit(r_tx_data, r_tx_crc, r_tx_bit_idx, bit_order);
                            if (r_tx_bit_idx == 5'd23)
                                r_tx_bit_idx <= 5'd0;
                            else
                                r_tx_bit_idx <= r_tx_bit_idx + 5'd1;
                        end

                        // Na borda de captura (sample_edge), contabiliza o bit recebido pelo Mestre
                        if (sample_edge) begin
                            if (r_bit_cnt == 5'd23) begin
                                // Pacote de 24 bits concluído; se SS continua ativo, busca próximo endereço em burst
                                r_bit_cnt    <= 5'd0;
                                r_tx_bit_idx <= 5'd0;
                                o_wb_rd_req  <= 1'b1;
                            end else begin
                                r_bit_cnt <= r_bit_cnt + 5'd1;
                            end
                        end
                    end

                    ST_ERROR: begin
                        miso <= 1'b0;
                    end

                    default: begin
                        state <= ST_IDLE;
                        miso  <= 1'b0;
                    end
                endcase
            end
        end
    end

endmodule

