// ----------------------------------------------------------------------
// Copyright (c) 2026 VIRTUS-UFCG. All rights reserved
// VIRTUS-UFCG Confidential Proprietary
// ----------------------------------------------------------------------
// Module     : wb_master_ctrl
// Description: Controlador Wishbone Master clássico (12 bits de endereço,
//              16 bits de dados) com suporte a auto-incremento de endereço
//              para operações de leitura e escrita em modo burst.
// ----------------------------------------------------------------------

module wb_master_ctrl #(
    parameter int ADDR_WIDTH = 12,
    parameter int DATA_WIDTH = 16,
    parameter int SEL_WIDTH  = DATA_WIDTH / 8
) (
    // Clock e Reset sincronizado do domínio do sistema
    input  logic                  clk,
    input  logic                  rst_sys_n,

    // Interface de controle vinda do spi_slave_core
    input  logic                  i_cmd_load_addr,
    input  logic [ADDR_WIDTH-1:0] i_init_addr,
    input  logic                  i_wr_req,
    input  logic [DATA_WIDTH-1:0] i_wr_data,
    input  logic                  i_rd_req,
    input  logic                  i_addr_skip, // Pacote de escrita descartado: apenas auto-incrementa o endereco
    output logic                  o_rd_valid,
    output logic [DATA_WIDTH-1:0] o_rd_data,
    output logic                  o_wb_busy,

    // Interface Wishbone Master (síncrona a clk)
    output logic [ADDR_WIDTH-1:0] wb_adr_o,
    output logic [DATA_WIDTH-1:0] wb_dat_o,
    input  logic [DATA_WIDTH-1:0] wb_dat_i,
    output logic [SEL_WIDTH-1:0]  wb_sel_o,
    output logic                  wb_we_o,
    output logic                  wb_stb_o,
    output logic                  wb_cyc_o,
    input  logic                  wb_ack_i
);

    typedef enum logic [1:0] {
        WB_IDLE  = 2'b00,
        WB_WRITE = 2'b01,
        WB_READ  = 2'b10
    } wb_state_t;

    wb_state_t state;
    logic [ADDR_WIDTH-1:0] r_curr_addr;

    assign o_wb_busy = (state != WB_IDLE);

    always_ff @(posedge clk or negedge rst_sys_n) begin
        if (!rst_sys_n) begin
            state       <= WB_IDLE;
            r_curr_addr <= '0;
            wb_adr_o    <= '0;
            wb_dat_o    <= '0;
            wb_sel_o    <= {SEL_WIDTH{1'b1}};
            wb_we_o     <= 1'b0;
            wb_stb_o    <= 1'b0;
            wb_cyc_o    <= 1'b0;
            o_rd_valid  <= 1'b0;
            o_rd_data   <= '0;
        end else begin
            o_rd_valid <= 1'b0;

            // Carrega o endereço inicial quando o cabeçalho SPI é decodificado
            if (i_cmd_load_addr) begin
                r_curr_addr <= i_init_addr;
            end

            case (state)
                WB_IDLE: begin
                    if (i_addr_skip) begin
                        r_curr_addr <= r_curr_addr + {{(ADDR_WIDTH-1){1'b0}}, 1'b1};
                    end
                    wb_stb_o <= 1'b0;
                    wb_cyc_o <= 1'b0;
                    wb_we_o  <= 1'b0;

                    if (i_wr_req) begin
                        wb_adr_o <= i_cmd_load_addr ? i_init_addr : r_curr_addr;
                        wb_dat_o <= i_wr_data;
                        wb_sel_o <= {SEL_WIDTH{1'b1}}; // Palavra completa de 16 bits
                        wb_we_o  <= 1'b1;
                        wb_stb_o <= 1'b1;
                        wb_cyc_o <= 1'b1;
                        state    <= WB_WRITE;
                    end else if (i_rd_req) begin
                        wb_adr_o <= i_cmd_load_addr ? i_init_addr : r_curr_addr;
                        wb_sel_o <= {SEL_WIDTH{1'b1}};
                        wb_we_o  <= 1'b0;
                        wb_stb_o <= 1'b1;
                        wb_cyc_o <= 1'b1;
                        state    <= WB_READ;
                    end
                end

                WB_WRITE: begin
                    if (wb_ack_i) begin
                        wb_stb_o    <= 1'b0;
                        wb_cyc_o    <= 1'b0;
                        wb_we_o     <= 1'b0;
                        // Auto-incremento de 1 após concluir a escrita no Wishbone
                        r_curr_addr <= wb_adr_o + {{(ADDR_WIDTH-1){1'b0}}, 1'b1};
                        state       <= WB_IDLE;
                    end
                end

                WB_READ: begin
                    if (wb_ack_i) begin
                        wb_stb_o    <= 1'b0;
                        wb_cyc_o    <= 1'b0;
                        wb_we_o     <= 1'b0;
                        o_rd_data   <= wb_dat_i;
                        o_rd_valid  <= 1'b1;
                        // Auto-incremento de 1 assim que o endereço atual é lido no Wishbone
                        r_curr_addr <= wb_adr_o + {{(ADDR_WIDTH-1){1'b0}}, 1'b1};
                        state       <= WB_IDLE;
                    end
                end

                default: begin
                    state    <= WB_IDLE;
                    wb_stb_o <= 1'b0;
                    wb_cyc_o <= 1'b0;
                    wb_we_o  <= 1'b0;
                end
            endcase
        end
    end

endmodule

