// ----------------------------------------------------------------------
// Copyright (c) 2026 VIRTUS-UFCG. All rights reserved
// VIRTUS-UFCG Confidential Proprietary
// ----------------------------------------------------------------------
// Module     : spi_wb_reset_sync
// Description: Sincronizador de reset ativo em nível baixo com ativação
//              (assert) assíncrona e desativação (deassert) síncrona.
// ----------------------------------------------------------------------

module spi_wb_reset_sync (
    input  logic clk,
    input  logic rst_n_async,
    output logic rst_n_sync
);

    logic rst_ff1;
    logic rst_ff2;

    always_ff @(posedge clk or negedge rst_n_async) begin
        if (!rst_n_async) begin
            rst_ff1 <= 1'b0;
            rst_ff2 <= 1'b0;
        end else begin
            rst_ff1 <= 1'b1;
            rst_ff2 <= rst_ff1;
        end
    end

    assign rst_n_sync = rst_ff2;

endmodule

