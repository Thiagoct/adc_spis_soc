`timescale 1ns / 1ps
// ----------------------------------------------------------------------
// Copyright (c) 2026 VIRTUS-UFCG. All rights reserved
// VIRTUS-UFCG Confidential Proprietary
// ----------------------------------------------------------------------
// Module     : adc_digital
// Description: Bloco digital comportamental do ADC SAR de 10 bits.
// ----------------------------------------------------------------------

module adc_digital #(
    parameter real T_CONV_NS = 100000.0 // 100 us para 10 KS/s
) (
    input  logic       rst_n,
    input  logic       enable,
    input  logic [9:0] d_raw,

    output logic       sample,
    output logic [9:0] d_out,
    output logic       data_rdy
);

    always begin
        if (!rst_n) begin
            d_out    <= 10'd0;
            data_rdy <= 1'b0;
            sample   <= 1'b0;
            #1ns;
        end else if (enable) begin
            // Dispara a leitura do valor analógico
            sample <= 1'b1;
            #1ns;
            sample <= 1'b0;

            // Aguarda o tempo de conversão do algoritmo SAR de 10 bits
            #(T_CONV_NS * 1ns);

            if (rst_n && enable) begin
                // Atualiza a saída e levanta a flag (lida pelo SPI_STATUS)
                d_out    <= d_raw;
                data_rdy <= 1'b1;

                // Segura o pulso por um breve período antes da próxima conversão
                #100ns;
                data_rdy <= 1'b0;
            end
        end else begin
            data_rdy <= 1'b0;
            sample   <= 1'b0;
            #1ns;
        end
    end

endmodule
