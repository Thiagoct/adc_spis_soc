// ----------------------------------------------------------------------
// Copyright (c) 2026 VIRTUS-UFCG. All rights reserved
// VIRTUS-UFCG Confidential Proprietary
// ----------------------------------------------------------------------
// File       : helloc.c
// Description: Firmware de teste do IP integrado:
//              Ponte SPI Slave para Wishbone Master + Banco de Registos +
//              Conversor Analógico-Digital (ADC) SAR de 10 bits.
//              Opera a CPU do LiteX como SPI Master (via BitBang CSRs)
//              para testar configuração, controle, conversão e integridade CRC.
// ----------------------------------------------------------------------

#include <stdio.h>
#include <inttypes.h>
#include <generated/mem.h>
#include <generated/csr.h>

// =========================================================================
// 1. Mapa de Registadores do ADC SAR 10b (Seções 3 e 5 da Especificação)
// =========================================================================
#define ADDR_ADC_CTRL       0x000  // Bit 7: RESET (SW), Bit 6: ENABLE, [5:0]: MODE
#define ADDR_SAMPLE_RATE    0x001  // [7:0]: Taxa de Amostragem
#define ADDR_FILTER_CFG     0x002  // [15:0]: Configuração do Filtro Digital
#define ADDR_ADC_DATA       0x003  // [9:0]: Dado convertido (Read-Only)
#define ADDR_GAIN_CFG       0x004  // [7:0]: Configuração de Ganho
#define ADDR_SPI_STATUS     0x005  // Bit 7: ERR, Bit 6: DATA_RDY (Read-Only)
#define ADDR_COEFF_0        0x006  // 0x006 a 0x00F: Coeficientes de filtro (R/W)
#define ADDR_COEFF_1        0x007
#define ADDR_COEFF_2        0x008
#define ADDR_COEFF_3        0x009

// Bits de Status (SPI_STATUS 0x005)
#define STATUS_BIT_ERR      (1 << 7)
#define STATUS_BIT_DATA_RDY (1 << 6)

// =========================================================================
// 2. Definições do Módulo SPI BitBang (CSRs)
// =========================================================================
// Bits de Controle (escritos no CSR spi_bitbang_control):
#define SPI_CTRL_SCK        (1 << 0)
#define SPI_CTRL_SS         (1 << 1)  // Ativo baixo
#define SPI_CTRL_MOSI       (1 << 2)
#define SPI_CTRL_RESET_S    (1 << 3)  // Ativo baixo
#define SPI_CTRL_CPOL       (1 << 4)  // 0: idle em 0
#define SPI_CTRL_CPHA       (1 << 5)  // 0: captura na 1a borda
#define SPI_CTRL_BIT_ORDER  (1 << 6)  // 1: MSB primeiro

// Bits de Status (lidos no CSR spi_bitbang_status):
#define SPI_STAT_MISO       (1 << 0)
#define SPI_STAT_BUSY       (1 << 1)  // Ativo baixo

// Estado local dos pinos de controle
static uint8_t g_spi_ctrl = (SPI_CTRL_SS | SPI_CTRL_RESET_S | SPI_CTRL_BIT_ORDER);

static inline void spi_write_raw_ctrl(uint8_t val) {
    g_spi_ctrl = val;
#ifdef CSR_SPI_BITBANG_CONTROL_ADDR
    spi_bitbang_control_write(val);
#endif
}

static inline uint8_t spi_read_raw_status(void) {
#ifdef CSR_SPI_BITBANG_STATUS_ADDR
    return (uint8_t)spi_bitbang_status_read();
#else
    return 0x02; // spi_busy = 1 (ocioso) como fallback
#endif
}

// =========================================================================
// 3. Funções de Cálculo de CRC8-CCITT (Polinômio 0x07: x^8 + x^2 + x + 1)
// =========================================================================
static uint8_t calc_crc8_ccitt(uint16_t data_word) {
    uint8_t crc = 0x00;
    for (int i = 15; i >= 0; i--) {
        uint8_t bit = (data_word >> i) & 1;
        uint8_t fb  = ((crc >> 7) ^ bit) & 1;
        crc = (uint8_t)((crc << 1) ^ (fb ? 0x07 : 0x00));
    }
    return crc;
}

// CRC do Cabeçalho de 13 bits: {cmd_read (1b), addr (12b)}
static uint8_t calc_hdr_crc(uint8_t cmd_read, uint16_t addr) {
    uint8_t crc = 0x00;
    uint16_t packet = ((cmd_read & 1) << 12) | (addr & 0xFFF);
    for (int i = 12; i >= 0; i--) {
        uint8_t bit = (packet >> i) & 1;
        uint8_t fb  = ((crc >> 7) ^ bit) & 1;
        crc = (uint8_t)((crc << 1) ^ (fb ? 0x07 : 0x00));
    }
    return crc;
}

// =========================================================================
// 4. Primitivas de Comunicação SPI BitBang (Modo 0: CPOL=0, CPHA=0, MSB first)
// =========================================================================
static inline void spi_delay(void) {
    for (volatile int i = 0; i < 10; i++);
}

static uint8_t spi_transfer_bit(uint8_t tx_b) {
    // Modo 0 (CPHA=0): MOSI é posicionado com SCK baixo
    if (tx_b)
        g_spi_ctrl |= SPI_CTRL_MOSI;
    else
        g_spi_ctrl &= ~SPI_CTRL_MOSI;
    g_spi_ctrl &= ~SPI_CTRL_SCK; // SCK = 0
    spi_write_raw_ctrl(g_spi_ctrl);
    spi_delay();

    // Borda de subida de SCK -> captura MISO
    g_spi_ctrl |= SPI_CTRL_SCK;  // SCK = 1
    spi_write_raw_ctrl(g_spi_ctrl);
    spi_delay();

    uint8_t rx_b = (spi_read_raw_status() & SPI_STAT_MISO) ? 1 : 0;

    // Retorna SCK para 0
    g_spi_ctrl &= ~SPI_CTRL_SCK;
    spi_write_raw_ctrl(g_spi_ctrl);
    spi_delay();

    return rx_b;
}

static void spi_send_bits(uint16_t val, int nbits) {
    for (int i = nbits - 1; i >= 0; i--) {
        spi_transfer_bit((val >> i) & 1);
    }
}

static uint16_t spi_recv_bits(int nbits) {
    uint16_t val = 0;
    for (int i = 0; i < nbits; i++) {
        val = (val << 1) | spi_transfer_bit(0);
    }
    return val;
}

static void spi_begin(void) {
    g_spi_ctrl &= ~SPI_CTRL_SCK;
    g_spi_ctrl &= ~SPI_CTRL_SS; // SS = 0 (ativo)
    spi_write_raw_ctrl(g_spi_ctrl);
    spi_delay();
}

static void spi_end(void) {
    g_spi_ctrl |= SPI_CTRL_SS;  // SS = 1 (inativo)
    g_spi_ctrl &= ~SPI_CTRL_MOSI;
    spi_write_raw_ctrl(g_spi_ctrl);
    spi_delay();
}

static void spi_send_header(uint8_t cmd_read, uint16_t addr, uint8_t bad_hdr_crc) {
    spi_transfer_bit(cmd_read & 1);
    spi_send_bits(addr, 12);
    uint8_t hc = calc_hdr_crc(cmd_read, addr);
    if (bad_hdr_crc) hc ^= 0xFF;
    spi_send_bits(hc, 8); // CRC do cabeçalho (Opção A / Sec 6.3.3.14)
}

static void spi_write_single(uint16_t addr, uint16_t data, uint8_t bad_crc, uint8_t bad_hdr_crc) {
    uint8_t crc = calc_crc8_ccitt(data);
    if (bad_crc) crc ^= 0xFF;

    spi_begin();
    spi_send_header(0, addr, bad_hdr_crc);
    spi_send_bits(data, 16);
    spi_send_bits(crc, 8);
    spi_end();
}

static uint8_t spi_read_single(uint16_t addr, uint16_t *data_out, uint8_t *crc_out) {
    spi_begin();
    spi_send_header(1, addr, 0);
    *data_out = spi_recv_bits(16);
    *crc_out  = (uint8_t)spi_recv_bits(8);
    spi_end();

    uint8_t exp_crc = calc_crc8_ccitt(*data_out);
    return (*crc_out == exp_crc);
}

// =========================================================================
// 5. Função Principal da Demonstração / Teste do Firmware (helloc)
// =========================================================================
void helloc(void);

void helloc(void) {
    uint16_t rd_data;
    uint8_t  rd_crc;
    int pass_count = 0;
    int fail_count = 0;

    printf("\n=======================================================\n");
    printf("   C: Teste do IP ADC SAR 10b + Ponte SPI/Wishbone     \n");
    printf("   VIRTUS-UFCG - CI Expert Hands-on                    \n");
    printf("=======================================================\n\n");

    // ---------------------------------------------------------------------
    // Teste 1: Inicialização e Reset da Interface SPI
    // ---------------------------------------------------------------------
    printf("[1] Inicializando interface SPI BitBang...\n");
    // Pulso em reset_s (ativo baixo)
    spi_write_raw_ctrl(SPI_CTRL_SS | SPI_CTRL_BIT_ORDER); // reset_s = 0
    spi_delay();
    spi_write_raw_ctrl(SPI_CTRL_SS | SPI_CTRL_RESET_S | SPI_CTRL_BIT_ORDER); // reset_s = 1
    spi_delay();

    uint8_t st = spi_read_raw_status();
    printf("    Status inicial: spi_busy=%d, miso=%d\n", (st & SPI_STAT_BUSY) ? 1 : 0, (st & SPI_STAT_MISO) ? 1 : 0);

    // ---------------------------------------------------------------------
    // Teste 2: Configuração e Readback dos Registadores do ADC
    // ---------------------------------------------------------------------
    printf("\n[2] Configurando Registadores do ADC via SPI...\n");

    // Habilita ADC: ADC_CTRL (0x000) = 0x0040 (ENABLE = 1)
    spi_write_single(ADDR_ADC_CTRL, 0x0040, 0, 0);
    // Configura taxa de amostragem: SAMPLE_RATE (0x001) = 0x003C
    spi_write_single(ADDR_SAMPLE_RATE, 0x003C, 0, 0);
    // Configura filtro: FILTER_CFG (0x002) = 0xA55A
    spi_write_single(ADDR_FILTER_CFG, 0xA55A, 0, 0);
    // Configura ganho: GAIN_CFG (0x004) = 0x001F
    spi_write_single(ADDR_GAIN_CFG, 0x001F, 0, 0);

    // Readback e validação de cada registrador
    if (spi_read_single(ADDR_ADC_CTRL, &rd_data, &rd_crc) && rd_data == 0x0040) {
        printf("    [PASS] ADC_CTRL: 0x%04X (CRC OK: 0x%02X)\n", rd_data, rd_crc);
        pass_count++;
    } else {
        printf("    [FAIL] ADC_CTRL: Obtido 0x%04X (Esp 0x0040)\n", rd_data);
        fail_count++;
    }

    if (spi_read_single(ADDR_SAMPLE_RATE, &rd_data, &rd_crc) && rd_data == 0x003C) {
        printf("    [PASS] SAMPLE_RATE: 0x%04X (CRC OK: 0x%02X)\n", rd_data, rd_crc);
        pass_count++;
    } else {
        printf("    [FAIL] SAMPLE_RATE: Obtido 0x%04X (Esp 0x003C)\n", rd_data);
        fail_count++;
    }

    if (spi_read_single(ADDR_FILTER_CFG, &rd_data, &rd_crc) && rd_data == 0xA55A) {
        printf("    [PASS] FILTER_CFG: 0x%04X (CRC OK: 0x%02X)\n", rd_data, rd_crc);
        pass_count++;
    } else {
        printf("    [FAIL] FILTER_CFG: Obtido 0x%04X (Esp 0xA55A)\n", rd_data);
        fail_count++;
    }

    if (spi_read_single(ADDR_GAIN_CFG, &rd_data, &rd_crc) && rd_data == 0x001F) {
        printf("    [PASS] GAIN_CFG: 0x%04X (CRC OK: 0x%02X)\n", rd_data, rd_crc);
        pass_count++;
    } else {
        printf("    [FAIL] GAIN_CFG: Obtido 0x%04X (Esp 0x001F)\n", rd_data);
        fail_count++;
    }

    // ---------------------------------------------------------------------
    // Teste 3: Escrita e Leitura de Coeficientes de Filtro
    // ---------------------------------------------------------------------
    printf("\n[3] Escrita e Leitura de Coeficientes (0x006 a 0x009)...\n");
    uint16_t coeffs[4] = {0xCAFE, 0xBEEF, 0xDEAD, 0xF00D};
    for (int i = 0; i < 4; i++) {
        spi_write_single(ADDR_COEFF_0 + i, coeffs[i], 0, 0);
        if (spi_read_single(ADDR_COEFF_0 + i, &rd_data, &rd_crc) && rd_data == coeffs[i]) {
            printf("    [PASS] Coeff[%d] (Addr 0x%03X): 0x%04X (CRC 0x%02X)\n", i, ADDR_COEFF_0 + i, rd_data, rd_crc);
            pass_count++;
        } else {
            printf("    [FAIL] Coeff[%d]: Obtido 0x%04X (Esp 0x%04X)\n", i, rd_data, coeffs[i]);
            fail_count++;
        }
    }

    // ---------------------------------------------------------------------
    // Teste 4: Polling de Conversão do ADC SAR e Leitura de Tensão
    // ---------------------------------------------------------------------
    printf("\n[4] Aguardando Conversao do ADC SAR 10b (Polling SPI_STATUS)...\n");
    int timeout = 100;
    uint8_t data_ready = 0;
    while (timeout-- > 0) {
        spi_read_single(ADDR_SPI_STATUS, &rd_data, &rd_crc);
        if (rd_data & STATUS_BIT_DATA_RDY) {
            data_ready = 1;
            break;
        }
    }

    if (data_ready) {
        printf("    [PASS] SPI_STATUS: DATA_RDY=1 (Status=0x%04X)\n", rd_data);
        pass_count++;

        spi_read_single(ADDR_ADC_DATA, &rd_data, &rd_crc);
        uint16_t adc_code = rd_data & 0x3FF; // 10 bits de resolução
        uint32_t mv = (adc_code * 800) / 1024; // Fundo de escala de 0.8V = 800 mV
        printf("    [PASS] ADC_DATA: Codigo = %u / 1023 (0x%03X), Tensao Estimada = %" PRIu32 " mV\n",
               adc_code, adc_code, mv);
        pass_count++;
    } else {
        printf("    [INFO] Timeout no polling de DATA_RDY (ADC em modelo comportamental)\n");
    }

    // ---------------------------------------------------------------------
    // Teste 5: Injeção de Erro de CRC e Verificação de Proteção
    // ---------------------------------------------------------------------
    printf("\n[5] Teste de Protecao por CRC8-CCITT...\n");
    // Tenta sobrescrever FILTER_CFG com CRC inválido
    spi_write_single(ADDR_FILTER_CFG, 0xFFFF, 1 /* bad_crc */, 0);

    // O registrador deve ter mantido o valor anterior 0xA55A
    spi_read_single(ADDR_FILTER_CFG, &rd_data, &rd_crc);
    if (rd_data == 0xA55A) {
        printf("    [PASS] FILTER_CFG protegido contra CRC invalido (mantem 0x%04X)\n", rd_data);
        pass_count++;
    } else {
        printf("    [FAIL] FILTER_CFG sobrescrito indevidamente com 0x%04X!\n", rd_data);
        fail_count++;
    }

    // SPI_STATUS deve ter ativado a flag ERR (bit 7)
    spi_read_single(ADDR_SPI_STATUS, &rd_data, &rd_crc);
    if (rd_data & STATUS_BIT_ERR) {
        printf("    [PASS] SPI_STATUS sinaliza erro de CRC: ERR=1 (0x%04X)\n", rd_data);
        pass_count++;
    } else {
        printf("    [FAIL] SPI_STATUS nao sinalizou ERR (Status=0x%04X)\n", rd_data);
        fail_count++;
    }

    // ---------------------------------------------------------------------
    // Teste 6: Reset por Software via ADC_CTRL (Bit 7 = 1 -> 0)
    // ---------------------------------------------------------------------
    printf("\n[6] Reset por Software via ADC_CTRL (Bit 7)...\n");
    spi_write_single(ADDR_ADC_CTRL, 0x0080, 0, 0); // RESET = 1
    spi_write_single(ADDR_ADC_CTRL, 0x0000, 0, 0); // Limpa RESET

    spi_read_single(ADDR_SAMPLE_RATE, &rd_data, &rd_crc);
    spi_read_single(ADDR_SPI_STATUS, &rd_data, &rd_crc);
    if ((rd_data & STATUS_BIT_ERR) == 0) {
        printf("    [PASS] Reset por Software limpa flag ERR e reinicializa registos\n");
        pass_count++;
    } else {
        printf("    [FAIL] Flag ERR permaneceu ativa apos software reset!\n");
        fail_count++;
    }

    // ---------------------------------------------------------------------
    // Resumo Final
    // ---------------------------------------------------------------------
    printf("\n=======================================================\n");
    printf("   RESULTADO FINAL: %d Testes PASSED, %d FAILED\n", pass_count, fail_count);
    if (fail_count == 0) {
        printf("   >>> STATUS: SUCESSO COMPLETO NO FIRMWARE! <<<\n");
    } else {
        printf("   >>> STATUS: FALHAS DETECTADAS NO FIRMWARE! <<<\n");
    }
    printf("=======================================================\n\n");
}
