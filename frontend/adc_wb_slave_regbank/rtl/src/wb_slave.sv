// ----------------------------------------------------------------------
// Copyright (c) 2026 VIRTUS-UFCG. All rights reserved
// VIRTUS-UFCG Confidential Proprietary
// ----------------------------------------------------------------------
// Module     : wb_slave
// Description: Controlador Wishbone Slave (12 bits de endereço, 16 bits
//              de dados) adaptado para o Banco de Registos do ADC SAR 10b.
//              Suporta máscaras de bytes (wb_sel_i) e garante geração de
//              wb_ack_o em todo o espaço de 12 bits (0x000 a 0xFFF),
//              retornando 16'h0000 em leituras fora de 0x000-0x00F e
//              ignorando silenciosamente escritas fora desse intervalo.
// ----------------------------------------------------------------------

module wb_slave #(
    parameter int ADDR_WIDTH   = 12,
    parameter int DATA_WIDTH   = 16,
    parameter int GRANULE      = 8,
    parameter int REGISTER_NUM = 15,
    parameter int SEL_WIDTH    = DATA_WIDTH / GRANULE
)(
    // --- Interface Wishbone Slave ---
    input  logic                   rst_i,
    input  logic                   i_clk,
    input  logic [ADDR_WIDTH-1:0]  adr_i,
    input  logic [DATA_WIDTH-1:0]  dat_i,
    output logic [DATA_WIDTH-1:0]  dat_o,
    input  logic [SEL_WIDTH-1:0]   sel_i,
    input  logic                   we_i,
    input  logic                   stb_i,
    output logic                   ack_o,
    output logic                   err_o,
    input  logic                   cyc_i,

    // --- Interface Externa para Banco de Registos (com handshake) ---
    output logic                   o_mem_valid,
    output logic                   o_mem_we,
    output logic [ADDR_WIDTH-1:0]  o_mem_addr,
    output logic [DATA_WIDTH-1:0]  o_mem_data_w,
    output logic [SEL_WIDTH-1:0]   o_mem_sel,
    input  logic [DATA_WIDTH-1:0]  i_mem_data_r,
    input  logic                   i_mem_ready
);

    typedef enum logic [2:0] {
        STATE_IDLE,
        STATE_READ_REQ,
        STATE_READ_WAIT,
        STATE_WRITE_REQ,
        STATE_WRITE_WAIT,
        STATE_ACK
    } state_t;

    state_t state;

    // Registos internos da transação
    logic [ADDR_WIDTH-1:0] r_adr;
    logic [DATA_WIDTH-1:0] r_dat_i;
    logic [SEL_WIDTH-1:0]  r_sel_i;
    logic                  r_we_i;
    logic                  r_error_flag;

    always_ff @(posedge i_clk or negedge rst_i) begin
        if (!rst_i) begin
            state        <= STATE_IDLE;
            ack_o        <= 1'b0;
            err_o        <= 1'b0;
            dat_o        <= '0;

            o_mem_valid  <= 1'b0;
            o_mem_we     <= 1'b0;
            o_mem_addr   <= '0;
            o_mem_data_w <= '0;
            o_mem_sel    <= {SEL_WIDTH{1'b1}};

            r_adr        <= '0;
            r_dat_i      <= '0;
            r_sel_i      <= {SEL_WIDTH{1'b1}};
            r_we_i       <= 1'b0;
            r_error_flag <= 1'b0;
        end else begin
            // Valores padrão em cada ciclo
            ack_o       <= 1'b0;
            err_o       <= 1'b0;
            o_mem_valid <= 1'b0;

            case (state)
                STATE_IDLE: begin
                    if (cyc_i && stb_i && !ack_o) begin
                        r_adr   <= adr_i;
                        r_dat_i <= dat_i;
                        r_sel_i <= sel_i;
                        r_we_i  <= we_i;

                        // Diretriz 5: Endereços não mapeados (0x010 a 0xFFF)
                        // Leituras retornam 16'h0000 e escritas são ignoradas silenciosamente,
                        // gerando sempre wb_ack_o para evitar bloqueio do barramento.
                        if (adr_i > REGISTER_NUM) begin
                            r_error_flag <= 1'b0;
                            if (!we_i) begin
                                dat_o <= '0;
                            end
                            state <= STATE_ACK;
                        end else begin
                            r_error_flag <= 1'b0;
                            if (we_i) begin
                                state <= STATE_WRITE_REQ;
                            end else begin
                                state <= STATE_READ_REQ;
                            end
                        end
                    end
                end

                STATE_READ_REQ: begin
                    o_mem_valid <= 1'b1;
                    o_mem_we    <= 1'b0;
                    o_mem_addr  <= r_adr;
                    o_mem_sel   <= r_sel_i;
                    state       <= STATE_READ_WAIT;
                end

                STATE_READ_WAIT: begin
                    if (i_mem_ready) begin
                        dat_o <= i_mem_data_r;
                        state <= STATE_ACK;
                    end
                end

                STATE_WRITE_REQ: begin
                    o_mem_valid  <= 1'b1;
                    o_mem_we     <= 1'b1;
                    o_mem_addr   <= r_adr;
                    o_mem_data_w <= r_dat_i;
                    o_mem_sel    <= r_sel_i;
                    state        <= STATE_WRITE_WAIT;
                end

                STATE_WRITE_WAIT: begin
                    if (i_mem_ready) begin
                        state <= STATE_ACK;
                    end
                end

                STATE_ACK: begin
                    ack_o <= 1'b1;
                    err_o <= r_error_flag;
                    state <= STATE_IDLE;
                end

                default: begin
                    state <= STATE_IDLE;
                end
            endcase
        end
    end

endmodule
