// ----------------------------------------------------------------------
// Copyright (c) 2026 VIRTUS-UFCG. All rights reserved
// VIRTUS-UFCG Confidential Proprietary
// ----------------------------------------------------------------------
// Module     : adc_analog
// Description: Bloco analógico comportamental do ADC SAR de 10 bits.
// ----------------------------------------------------------------------

module adc_analog #(
    parameter real V_FULL_SCALE = 0.8, // Tensão de fundo de escala
    parameter int  RESOLUTION   = 10   // Resolução de 10 bits
) (
    input  real        v_in,    // Entrada analógica
    input  logic       sample,  // Pulso de gatilho vindo do digital
    output logic [9:0] d_raw    // Valor digital bruto amostrado
);

    real LSB_size;
    real clamped_v_in;

    initial begin
        d_raw    = 10'd0;
        LSB_size = V_FULL_SCALE / ((1 << RESOLUTION) - 1.0);
    end

    // Calcula a quantização ideal na borda de subida do sinal de amostragem
    always @(posedge sample) begin
        if (v_in < 0.0)
            clamped_v_in = 0.0;
        else if (v_in > V_FULL_SCALE)
            clamped_v_in = V_FULL_SCALE;
        else
            clamped_v_in = v_in;

        d_raw = $rtoi(clamped_v_in / LSB_size);
    end

endmodule

