// ----------------------------------------------------------------------
// Copyright (c) 2026 VIRTUS-UFCG. All rights reserved
// VIRTUS-UFCG Confidential Proprietary
// ----------------------------------------------------------------------
// Module     : spi_crc8_ccitt
// Description: Calculador e verificador CRC8-CCITT (Polinômio x^8 + x^2 + x + 1,
//              0x07, valor inicial 0x00) para pacotes de dados de 16 bits e
//              cabeçalhos de 13 bits (1-bit Comando + 12-bit Endereço).
// ----------------------------------------------------------------------

module spi_crc8_ccitt #(
    parameter logic [7:0] CRC_POLY = 8'h07,
    parameter logic [7:0] CRC_INIT = 8'h00
) (
    // Geração de CRC para transmissão (Leitura: 16 bits de dados -> 8 bits CRC)
    input  logic [15:0] i_tx_data,
    output logic [7:0]  o_tx_crc,

    // Validação de CRC para recepção de dados (Escrita: 16 bits de dados + 8 bits CRC)
    input  logic [15:0] i_rx_data,
    input  logic [7:0]  i_rx_data_crc,
    output logic [7:0]  o_rx_data_calc_crc,
    output logic        o_rx_data_crc_ok,

    // Validação de CRC para cabeçalho (Opção A: 13 bits [cmd, addr] + 8 bits CRC)
    input  logic [12:0] i_rx_hdr,
    input  logic [7:0]  i_rx_hdr_crc,
    output logic [7:0]  o_rx_hdr_calc_crc,
    output logic        o_rx_hdr_crc_ok
);

    // Função auxiliar para avanço de 1 bit no LFSR do CRC8-CCITT
    function automatic logic [7:0] crc8_step (
        input logic [7:0] crc_in,
        input logic       bit_in
    );
        logic feedback;
        begin
            feedback  = crc_in[7] ^ bit_in;
            crc8_step = {crc_in[6:0], 1'b0} ^ (feedback ? CRC_POLY : 8'h00);
        end
    endfunction

    // Cálculo de CRC8-CCITT para palavra de 16 bits (processado do MSB ao LSB lógico)
    function automatic logic [7:0] calc_crc16 (
        input logic [15:0] data_word
    );
        logic [7:0] crc_tmp;
        begin
            crc_tmp = CRC_INIT;
            for (int i = 15; i >= 0; i--) begin
                crc_tmp = crc8_step(crc_tmp, data_word[i]);
            end
            calc_crc16 = crc_tmp;
        end
    endfunction

    // Cálculo de CRC8-CCITT para cabeçalho de 13 bits {cmd, addr[11:0]}
    function automatic logic [7:0] calc_crc13 (
        input logic [12:0] hdr_word
    );
        logic [7:0] crc_tmp;
        begin
            crc_tmp = CRC_INIT;
            for (int i = 12; i >= 0; i--) begin
                crc_tmp = crc8_step(crc_tmp, hdr_word[i]);
            end
            calc_crc13 = crc_tmp;
        end
    endfunction

    always_comb begin
        o_tx_crc           = calc_crc16(i_tx_data);
        o_rx_data_calc_crc = calc_crc16(i_rx_data);
        o_rx_data_crc_ok   = (o_rx_data_calc_crc == i_rx_data_crc);

        o_rx_hdr_calc_crc  = calc_crc13(i_rx_hdr);
        o_rx_hdr_crc_ok    = (o_rx_hdr_calc_crc == i_rx_hdr_crc);
    end

endmodule

