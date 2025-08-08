#include "transmissor.h"

// ============================================================================
// == Função Principal ========================================================
// ============================================================================

int main()
{
    stdio_init_all();
    sleep_ms(2000);
    inicializar_display_i2c();

    ssd1306_fill(&ssd, !cor);                     // limpa o display
    ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor); // Moldura
    ssd1306_draw_string(&ssd, "TRANSMISSOR", 20, 20);
    ssd1306_draw_string(&ssd, "LORA", 50, 35);
    ssd1306_send_data(&ssd); // Envia os dados para o display
    printf("Iniciando Transmissor LoRa (TX)...\n");

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
    uint8_t version = rmf95_read_reg(REG_VERSION);
    printf("Versao do RFM95: 0x%02X\n", version);

    if (version != 0x12)
    {
        printf("Falha na comunicacao SPI. Travando. ❌\n");
        while (1)
            ;
    }
    printf("Comunicacao SPI OK! ✅\n");

    // Configura o rádio para operar com LoRa
    lora_init();

    int counter = 0;
    char message_buffer[50];

    while (1)
    {
        // Monta a mensagem a ser enviada
        snprintf(message_buffer, sizeof(message_buffer), "Ola do Pico! Cont: %d", counter);

        // Envia o pacote
        lora_send_packet(message_buffer);

        counter++;
        sleep_ms(5000); // Espera 5 segundos antes de enviar o próximo pacote
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

void rmf95_write_fifo(const uint8_t *buffer, uint8_t length)
{
    uint8_t tx_data = REG_FIFO | 0x80;
    gpio_put(PIN_CS, 0);
    spi_write_blocking(SPI_PORT, &tx_data, 1);    // Envia o endereço do FIFO com o bit de escrita
    spi_write_blocking(SPI_PORT, buffer, length); // Envia os dados do pacote
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

void lora_set_power(uint8_t power)
{
    // Configura para usar o pino PA_BOOST, necessário para potências > 14 dBm
    rmf95_write_reg(REG_PA_CONFIG, PA_BOOST_PIN | (power - 2));
}

void lora_init()
{
    // 1. Colocar em modo Sleep + LoRa
    rmf95_write_reg(REG_OP_MODE, MODE_SLEEP);
    sleep_ms(10);

    // 2. Configurar a frequência
    lora_set_frequency(LORA_FREQUENCY_HZ);

    // 3. Configurar a potência de saída
    lora_set_power(OUTPUT_POWER_DBM);

    // 4. Configurar os ponteiros da FIFO
    rmf95_write_reg(REG_FIFO_TX_BASE_ADDR, 0x00); // TX usa a primeira metade da FIFO
    rmf95_write_reg(REG_FIFO_RX_BASE_ADDR, 0x80); // RX usa a segunda metade
    rmf95_write_reg(REG_FIFO_ADDR_PTR, 0x00);

    // 5. Configurar LNA (não crítico para TX, mas bom manter consistente)
    rmf95_write_reg(REG_LNA, 0x23);

    // 6. Configurar o modem (DEVE ser idêntico ao receptor)
    rmf95_write_reg(REG_MODEM_CONFIG_1, 0x72); // BW 125kHz, Coding Rate 4/5, Explicit Header
    rmf95_write_reg(REG_MODEM_CONFIG_2, 0x74); // SF7, CRC On
    rmf95_write_reg(REG_PREAMBLE_MSB, 0x00);
    rmf95_write_reg(REG_PREAMBLE_LSB, 0x08); // Preâmbulo de 8 símbolos

    // 7. Colocar em modo Standby para finalizar
    rmf95_write_reg(REG_OP_MODE, MODE_STDBY);
    sleep_ms(10);

    printf("RFM95 configurado para LoRa TX em %ld Hz\n", LORA_FREQUENCY_HZ);
}

// ============================================================================
// == Função de Transmissão LoRa ==============================================
// ============================================================================

void lora_send_packet(const char *message)
{
    // 1. Mudar para o modo Standby para preparar
    rmf95_write_reg(REG_OP_MODE, MODE_STDBY);

    // 2. Apontar o FIFO para o início da área de TX e carregar os dados
    rmf95_write_reg(REG_FIFO_ADDR_PTR, 0x00);
    rmf95_write_fifo((const uint8_t *)message, strlen(message));

    // 3. Definir o tamanho do payload que será enviado
    rmf95_write_reg(REG_PAYLOAD_LENGTH, strlen(message));

    // 4. Acionar a transmissão
    rmf95_write_reg(REG_OP_MODE, MODE_TX);

    // 5. Aguardar o fim da transmissão (polling na flag TxDone)
    while ((rmf95_read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0)
    {
        sleep_ms(10);
    }

    // 6. Limpar a flag de interrupção
    rmf95_write_reg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);

    printf("Pacote enviado: '%s'\n", message);
}

// ============================================================================
// == Inicialização do display oled ===========================================
// ============================================================================
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
