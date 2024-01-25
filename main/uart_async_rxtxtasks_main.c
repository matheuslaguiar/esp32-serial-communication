/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

// UART
static const int RX_BUF_SIZE = 1024;
#define TXD_PIN (GPIO_NUM_43)
#define RXD_PIN (GPIO_NUM_44)

#define TXD1_PIN (GPIO_NUM_17)
#define RXD1_PIN (GPIO_NUM_18)

#define TXD2_PIN (GPIO_NUM_4)
#define RXD2_PIN (GPIO_NUM_5)

// USB
#define TAG_USB_RX_CB "USB RX CALLBACK"
#include "driver/usb_serial_jtag.h"

void init_uart(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD1_PIN, RXD1_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

#if defined(CONFIG_IDF_TARGET_ESP32S3)
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TXD2_PIN, RXD2_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
#endif
}

int sendData(const char* logName, uart_port_t port, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(port, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        // sendData(TX_TASK_TAG, UART_NUM_0, "Hello world from uart0");
        // vTaskDelay(2000 / portTICK_PERIOD_MS);

        sendData(TX_TASK_TAG, UART_NUM_1, "Hello world from uart1");
        vTaskDelay(2000 / portTICK_PERIOD_MS);

#if defined(CONFIG_IDF_TARGET_ESP32S3)
        sendData(TX_TASK_TAG, UART_NUM_2, "Hello world from uart2");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
#endif
    }
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes1 = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes1 > 0) {
            data[rxBytes1] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes (uart1): '%s'", rxBytes1, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes1, ESP_LOG_INFO);
        }

#if defined(CONFIG_IDF_TARGET_ESP32S3)
        const int rxBytes2 = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes2 > 0) {
            data[rxBytes2] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes (uart2): '%s'", rxBytes2, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes2, ESP_LOG_INFO);
        }
#endif
    }
    free(data);
}

static uint32_t usb_serial_jtag_read_until(uint8_t *data, uint32_t len, char target) {
    uint8_t new_char;
    uint32_t size = 0;

    // Empty or null data buffer
    if(len == 0 || data == NULL)
        return 0;

    while(usb_serial_jtag_read_bytes(&new_char, 1, pdMS_TO_TICKS(10))) {
        if(new_char == '\n')
            break;
        data[size++] = new_char;
        if(size == len)
            break;
    }
    return size;
}

static void usb_task(void *pvParameters) {
    uint8_t data[256];
    int dataLen = 0;
    while(1){
        dataLen = usb_serial_jtag_read_until(data, 256, '\n');
        if(dataLen){
            if(dataLen == 2 && data[0] == 'O' && data[1] == 'N'){ // ON
                ESP_LOGW("LED", "LED ON");
                gpio_set_level(GPIO_NUM_48,1);
            }
            if(dataLen == 3 && data[0] == 'O' && data[1] == 'F' && data[2] == 'F'){ // OFF
                ESP_LOGW("LED", "LED OFF");
                gpio_set_level(GPIO_NUM_48,0);
            }
            ESP_LOGW(TAG_USB_RX_CB, "Received from USB: %.*s", dataLen, data);
        }
        else
            ESP_LOGE(TAG_USB_RX_CB, "Nothing Received from USB");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void init_usb_jtag(void) {
    usb_serial_jtag_driver_config_t config = {
        .rx_buffer_size = 256,
        .tx_buffer_size = 256
    };
    usb_serial_jtag_driver_install(&config);
    xTaskCreate(usb_task, "USB_task", 1024*8, NULL, configMAX_PRIORITIES, NULL);
}

void app_main(void)
{
    gpio_config_t config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1LL<<GPIO_NUM_48),
    };
    
    gpio_config(&config);
    init_uart();
    init_usb_jtag();
    gpio_config(&config);
    xTaskCreate(rx_task, "uart_rx_task", 1024*8, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024*8, NULL, configMAX_PRIORITIES-2, NULL);
}
