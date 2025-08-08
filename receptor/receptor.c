#include "receptor.h"

// ============================================================================
// == Função Principal ========================================================
// ============================================================================

int main()
{
    stdio_init_all();
    inicializar_display_i2c();
    ssd1306_fill(&ssd, !cor);                     // limpa o display
    ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor); // Moldura
    ssd1306_draw_string(&ssd, "RECEPTOR", 35, 20);
    ssd1306_draw_string(&ssd, "LORA", 50, 35);
    ssd1306_send_data(&ssd); // Envia os dados para o display
    sleep_ms(2000);

    // Inicialização do hardware
    spi_init(SPI_PORT, 1000 * 1000); // 1 MHz
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    gpio_init(PIN_RST);
    gpio_set_dir(PIN_RST, GPIO_OUT);

    // Reset e verificação do módulo
    rmf95_reset();
    sleep_ms(5000);
    uint8_t version = rmf95_read_reg(REG_VERSION);
    printf("Versao do RFM95: 0x%02X\n", version);

    if (version != 0x12)
    {
        ssd1306_fill(&ssd, !cor);                     // limpa o display
        ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor); // Moldura
        ssd1306_draw_string(&ssd, "Falha na comunicacao", 20, 30);
        ssd1306_send_data(&ssd); // Envia os dados para o display
        while (1)
            ;
    }
    printf("Comunicacao SPI OK! ✅\n");

    // Configura o rádio para operar com LoRa
    lora_init();

    // Coloca o rádio no modo de Recepção Contínua
    printf("Aguardando pacotes...\n");
    rmf95_write_reg(REG_OP_MODE, MODE_RX_CONTINUOUS);

    while (1)
    {
        // Verifica se a flag de 'RxDone' foi acionada
        uint8_t irq_flags = rmf95_read_reg(REG_IRQ_FLAGS);
        if (irq_flags & IRQ_RX_DONE_MASK)
        {

            // Limpa a flag de interrupção escrevendo 1 no bit correspondente
            rmf95_write_reg(REG_IRQ_FLAGS, IRQ_RX_DONE_MASK);

            // Verifica se houve erro de CRC
            if (irq_flags & IRQ_PAYLOAD_CRC_ERROR_MASK)
            {
                printf("Erro de CRC no pacote recebido!\n");
            }
            else
            {
                // Lê o número de bytes recebidos
                uint8_t received_len = rmf95_read_reg(REG_RX_NB_BYTES);

                // Obtém o endereço inicial do pacote na FIFO
                uint8_t current_addr = rmf95_read_reg(REG_FIFO_RX_CURRENT_ADDR);
                rmf95_write_reg(REG_FIFO_ADDR_PTR, current_addr);

                // Lê os dados da FIFO
                uint8_t buffer[256];
                memset(buffer, 0, sizeof(buffer));
                rmf95_read_fifo(buffer, received_len);

                // Lê RSSI e SNR
                int8_t rssi = rmf95_read_reg(REG_PKT_RSSI_VALUE) - 157; // Para a porta HF
                int8_t snr = (int8_t)rmf95_read_reg(REG_PKT_SNR_VALUE) / 4;

                printf("--------------------------------\n");
                printf("Pacote Recebido!\n");
                printf("  Mensagem: '%s'\n", buffer);
                printf("  Bytes: %d\n", received_len);
                printf("  RSSI: %d dBm\n", rssi);
                printf("  SNR: %d dB\n", snr);
                printf("--------------------------------\n");

                ssd1306_fill(&ssd, 0); // Limpa o display (0 = preto)

                // Moldura
                ssd1306_rect(&ssd, 0, 0, 127, 63, 1, 0);

                // Cabeçalho
                ssd1306_draw_string(&ssd, "Msg recebida!", 20, 2);

                // Mensagem
                char msg_line[22];
                snprintf(msg_line, sizeof(msg_line), "Msg: %.12s", buffer); // Mostra até 12 caracteres
                ssd1306_draw_string(&ssd, msg_line, 2, 16);

                // Bytes recebidos
                char bytes_line[22];
                snprintf(bytes_line, sizeof(bytes_line), "Bytes: %d", received_len);
                ssd1306_draw_string(&ssd, bytes_line, 2, 26);

                // RSSI
                char rssi_line[22];
                snprintf(rssi_line, sizeof(rssi_line), "RSSI: %d dBm", rssi);
                ssd1306_draw_string(&ssd, rssi_line, 2, 36);

                // SNR
                char snr_line[22];
                snprintf(snr_line, sizeof(snr_line), "SNR: %d dB", snr);
                ssd1306_draw_string(&ssd, snr_line, 2, 46);

                // Atualiza o display
                ssd1306_send_data(&ssd);
            }
        }
    }

    return 0;
}

// ============================================================================
// == Funções Básicas de Comunicação SPI ======================================
// ============================================================================

void rmf95_reset()
{
    gpio_put(PIN_RST, 1);
    sleep_ms(1);
    gpio_put(PIN_RST, 0);
    sleep_ms(1);
    gpio_put(PIN_RST, 1);
    sleep_ms(5);
}

void rmf95_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx_data[] = {reg | 0x80, value};
    gpio_put(PIN_CS, 0);
    spi_write_blocking(SPI_PORT, tx_data, 2);
    gpio_put(PIN_CS, 1);
}

uint8_t rmf95_read_reg(uint8_t reg)
{
    uint8_t tx_data[] = {reg & 0x7F, 0x00};
    uint8_t rx_data[2];
    gpio_put(PIN_CS, 0);
    spi_write_read_blocking(SPI_PORT, tx_data, rx_data, 2);
    gpio_put(PIN_CS, 1);
    return rx_data[1];
}

void rmf95_read_fifo(uint8_t *buffer, uint8_t length)
{
    uint8_t tx_data = REG_FIFO & 0x7F;
    gpio_put(PIN_CS, 0);
    spi_write_blocking(SPI_PORT, &tx_data, 1);
    spi_read_blocking(SPI_PORT, 0, buffer, length);
    gpio_put(PIN_CS, 1);
}

// ============================================================================
// == Funções de Configuração LoRa ============================================
// ============================================================================

void lora_set_frequency(long frequency)
{
    uint64_t frf = ((uint64_t)frequency << 19) / RF_CRYSTAL_FREQ_HZ;
    rmf95_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    rmf95_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
    rmf95_write_reg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void lora_init()
{
    // 1. Colocar em modo Sleep + LoRa
    rmf95_write_reg(REG_OP_MODE, MODE_SLEEP);
    sleep_ms(10);

    // 2. Configurar a frequência
    lora_set_frequency(LORA_FREQUENCY_HZ);

    // 3. Configurar os ponteiros da FIFO
    rmf95_write_reg(REG_FIFO_TX_BASE_ADDR, 0x80);
    rmf95_write_reg(REG_FIFO_RX_BASE_ADDR, 0x00);
    rmf95_write_reg(REG_FIFO_ADDR_PTR, 0x00);

    // 4. Configurar LNA (Low Noise Amplifier) para melhor sensibilidade
    rmf95_write_reg(REG_LNA, 0x23); // Ganho máximo e boost de HF

    // 5. Configurar o modem
    rmf95_write_reg(REG_MODEM_CONFIG_1, 0x72); // BW 125kHz, Coding Rate 4/5, Explicit Header
    rmf95_write_reg(REG_MODEM_CONFIG_2, 0x74); // SF7, CRC On
    rmf95_write_reg(REG_PAYLOAD_LENGTH, 0x10); // Payload de 16 bytes (ajuste se necessário)
    rmf95_write_reg(REG_MAX_PAYLOAD_LENGTH, 0xFF);
    rmf95_write_reg(REG_PREAMBLE_MSB, 0x00);
    rmf95_write_reg(REG_PREAMBLE_LSB, 0x08); // Preâmbulo de 8 símbolos

    // 6. Colocar em modo Standby para finalizar
    rmf95_write_reg(REG_OP_MODE, MODE_STDBY);
    sleep_ms(10);

    printf("RFM95 configurado para LoRa RX em %ld Hz\n", LORA_FREQUENCY_HZ);
}

void inicializar_display_i2c()
{ // FUNÇÃO PARA INICIALIZAR O I2C DO DISPLAY
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);
}
