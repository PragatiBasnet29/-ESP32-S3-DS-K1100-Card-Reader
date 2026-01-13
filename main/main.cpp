// ============================================================================
// File: main/main.cpp
// ESP32-S3 DS-K1100 Card Reader Controller - ESP-IDF Version
// ============================================================================

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "esp_timer.h"

// LVGL for GUI
#include "lvgl.h"
#include "lvgl_helpers.h"

static const char *TAG = "CardReader";

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// RS-485 Configuration
#define RS485_UART_NUM      UART_NUM_1
#define RS485_TX_PIN        GPIO_NUM_17
#define RS485_RX_PIN        GPIO_NUM_18
#define RS485_DE_RE_PIN     GPIO_NUM_16
#define RS485_BAUD_RATE     9600

// Wiegand Configuration
#define WIEGAND_D0_PIN      GPIO_NUM_19
#define WIEGAND_D1_PIN      GPIO_NUM_20
#define LED_CONTROL_PIN     GPIO_NUM_21
#define BEEP_CONTROL_PIN    GPIO_NUM_22
#define CASE_SENSOR_PIN     GPIO_NUM_23

// Touch and Display (adjust based on your hardware)
#define LCD_MOSI            GPIO_NUM_11
#define LCD_MISO            GPIO_NUM_13
#define LCD_CLK             GPIO_NUM_12
#define LCD_CS              GPIO_NUM_10
#define LCD_DC              GPIO_NUM_14
#define LCD_RST             GPIO_NUM_9
#define LCD_BCKL            GPIO_NUM_15
#define TOUCH_CS            GPIO_NUM_33

// ============================================================================
// COMMUNICATION MODE
// ============================================================================

typedef enum {
    MODE_RS485,
    MODE_WIEGAND
} comm_mode_t;

static comm_mode_t current_mode = MODE_RS485;

// ============================================================================
// CARD READER STATUS
// ============================================================================

typedef struct {
    bool online;
    uint8_t address;
    char last_card_id[32];
    bool door_open;
    bool tamper_alarm;
    uint32_t last_activity;
} card_reader_status_t;

static card_reader_status_t reader_status = {
    .online = false,
    .address = 1,
    .last_card_id = "",
    .door_open = false,
    .tamper_alarm = false,
    .last_activity = 0
};

// ============================================================================
// WIEGAND DATA
// ============================================================================

static volatile uint32_t wiegand_data = 0;
static volatile uint8_t wiegand_bit_count = 0;
static volatile uint32_t last_wiegand_time = 0;

// ============================================================================
// LVGL OBJECTS
// ============================================================================

static lv_obj_t *main_screen;
static lv_obj_t *btn_rs485;
static lv_obj_t *btn_wiegand;
static lv_obj_t *btn_open_door;
static lv_obj_t *btn_reset_alarm;
static lv_obj_t *btn_read_card;
static lv_obj_t *label_status;
static lv_obj_t *label_mode;
static lv_obj_t *label_card;

// ============================================================================
// RS-485 FUNCTIONS
// ============================================================================

static void rs485_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = RS485_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(RS485_UART_NUM, 1024, 1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RS485_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(RS485_UART_NUM, RS485_TX_PIN, RS485_RX_PIN, 
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Configure DE/RE pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RS485_DE_RE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(RS485_DE_RE_PIN, 0); // RX mode

    ESP_LOGI(TAG, "RS-485 initialized");
}

static void rs485_send_command(uint8_t addr, uint8_t cmd, uint8_t *data, uint8_t len)
{
    // Enable transmit mode
    gpio_set_level(RS485_DE_RE_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(1));

    // Build packet: [START][ADDR][CMD][LEN][DATA...][CHECKSUM]
    uint8_t buffer[128];
    uint8_t idx = 0;
    uint8_t checksum = 0;

    buffer[idx++] = 0xAA; // Start byte
    buffer[idx++] = addr;
    checksum += addr;
    
    buffer[idx++] = cmd;
    checksum += cmd;
    
    buffer[idx++] = len;
    checksum += len;

    for (int i = 0; i < len; i++) {
        buffer[idx++] = data[i];
        checksum += data[i];
    }

    buffer[idx++] = checksum;

    uart_write_bytes(RS485_UART_NUM, (const char *)buffer, idx);
    uart_wait_tx_done(RS485_UART_NUM, pdMS_TO_TICKS(100));

    // Switch back to receive mode
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(RS485_DE_RE_PIN, 0);

    ESP_LOGI(TAG, "RS-485 sent: Addr=%d, Cmd=0x%02X", addr, cmd);
}

static void rs485_receive_task(void *arg)
{
    uint8_t buffer[128];
    
    while (1) {
        int len = uart_read_bytes(RS485_UART_NUM, buffer, sizeof(buffer), pdMS_TO_TICKS(100));
        
        if (len > 0 && buffer[0] == 0xAA) {
            reader_status.online = true;
            reader_status.last_activity = esp_timer_get_time() / 1000;

            if (len >= 5) {
                uint8_t addr = buffer[1];
                uint8_t status = buffer[2];
                
                // Parse card ID if present
                if (len > 5) {
                    memset(reader_status.last_card_id, 0, sizeof(reader_status.last_card_id));
                    int card_len = len - 5;
                    if (card_len > 15) card_len = 15;
                    
                    for (int i = 0; i < card_len; i++) {
                        sprintf(reader_status.last_card_id + strlen(reader_status.last_card_id), 
                                "%02X", buffer[4 + i]);
                    }
                }

                ESP_LOGI(TAG, "RS-485 Response: Addr=%d, Status=0x%02X, Card=%s", 
                         addr, status, reader_status.last_card_id);
            }
        }

        // Check timeout
        if ((esp_timer_get_time() / 1000 - reader_status.last_activity) > 5000) {
            reader_status.online = false;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ============================================================================
// WIEGAND FUNCTIONS
// ============================================================================

static void IRAM_ATTR wiegand_d0_isr_handler(void *arg)
{
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    if (current_time - last_wiegand_time > 100) {
        wiegand_bit_count = 0;
        wiegand_data = 0;
    }
    
    wiegand_data <<= 1;
    wiegand_bit_count++;
    last_wiegand_time = current_time;
}

static void IRAM_ATTR wiegand_d1_isr_handler(void *arg)
{
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    if (current_time - last_wiegand_time > 100) {
        wiegand_bit_count = 0;
        wiegand_data = 0;
    }
    
    wiegand_data <<= 1;
    wiegand_data |= 1;
    wiegand_bit_count++;
    last_wiegand_time = current_time;
}

static void wiegand_init(void)
{
    // Configure D0 and D1 pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << WIEGAND_D0_PIN) | (1ULL << WIEGAND_D1_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);

    // Configure control pins
    gpio_config_t ctrl_conf = {
        .pin_bit_mask = (1ULL << LED_CONTROL_PIN) | (1ULL << BEEP_CONTROL_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&ctrl_conf);

    // Configure case sensor
    gpio_config_t sensor_conf = {
        .pin_bit_mask = (1ULL << CASE_SENSOR_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&sensor_conf);

    // Install ISR service and add handlers
    gpio_install_isr_service(0);
    gpio_isr_handler_add(WIEGAND_D0_PIN, wiegand_d0_isr_handler, NULL);
    gpio_isr_handler_add(WIEGAND_D1_PIN, wiegand_d1_isr_handler, NULL);

    ESP_LOGI(TAG, "Wiegand initialized");
}

static void wiegand_process_task(void *arg)
{
    while (1) {
        uint32_t current_time = esp_timer_get_time() / 1000;
        
        if (wiegand_bit_count > 0 && (current_time - last_wiegand_time) > 100) {
            if (wiegand_bit_count == 26 || wiegand_bit_count == 34) {
                reader_status.online = true;
                reader_status.last_activity = current_time;

                // Extract card number
                uint32_t card_num = 0;
                if (wiegand_bit_count == 26) {
                    card_num = (wiegand_data >> 1) & 0xFFFFFF;
                } else {
                    card_num = (wiegand_data >> 1) & 0xFFFFFFFF;
                }

                sprintf(reader_status.last_card_id, "%08X", card_num);
                
                ESP_LOGI(TAG, "Wiegand Card: %d bits, ID: %s", wiegand_bit_count, reader_status.last_card_id);

                // Visual feedback
                gpio_set_level(LED_CONTROL_PIN, 1);
                gpio_set_level(BEEP_CONTROL_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(100));
                gpio_set_level(BEEP_CONTROL_PIN, 0);
                vTaskDelay(pdMS_TO_TICKS(100));
                gpio_set_level(BEEP_CONTROL_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(100));
                gpio_set_level(BEEP_CONTROL_PIN, 0);
                gpio_set_level(LED_CONTROL_PIN, 0);
            }

            wiegand_bit_count = 0;
            wiegand_data = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ============================================================================
// LVGL GUI CALLBACKS
// ============================================================================

static void btn_rs485_event_cb(lv_event_t *e)
{
    if (current_mode != MODE_RS485) {
        current_mode = MODE_RS485;
        ESP_LOGI(TAG, "Switched to RS-485 mode");
        lv_label_set_text(label_mode, "Mode: RS-485");
    }
}

static void btn_wiegand_event_cb(lv_event_t *e)
{
    if (current_mode != MODE_WIEGAND) {
        current_mode = MODE_WIEGAND;
        ESP_LOGI(TAG, "Switched to Wiegand mode");
        lv_label_set_text(label_mode, "Mode: Wiegand");
    }
}

static void btn_open_door_event_cb(lv_event_t *e)
{
    ESP_LOGI(TAG, "Open door requested");
    
    if (current_mode == MODE_RS485) {
        uint8_t cmd = 0x40; // Door open command
        rs485_send_command(reader_status.address, cmd, NULL, 0);
    } else {
        gpio_set_level(LED_CONTROL_PIN, 1);
        gpio_set_level(BEEP_CONTROL_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(BEEP_CONTROL_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(BEEP_CONTROL_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(BEEP_CONTROL_PIN, 0);
        gpio_set_level(LED_CONTROL_PIN, 0);
    }
}

static void btn_reset_alarm_event_cb(lv_event_t *e)
{
    reader_status.tamper_alarm = false;
    ESP_LOGI(TAG, "Alarm reset");
}

static void btn_read_card_event_cb(lv_event_t *e)
{
    ESP_LOGI(TAG, "Read card status requested");
    
    if (current_mode == MODE_RS485) {
        uint8_t cmd = 0x20; // Status query command
        rs485_send_command(reader_status.address, cmd, NULL, 0);
    }
}

// ============================================================================
// LVGL GUI SETUP
// ============================================================================

static void create_gui(void)
{
    main_screen = lv_obj_create(NULL);
    lv_scr_load(main_screen);

    // Header
    lv_obj_t *header = lv_obj_create(main_screen);
    lv_obj_set_size(header, LV_HOR_RES, 50);
    lv_obj_set_pos(header, 0, 0);
    lv_obj_set_style_bg_color(header, lv_color_hex(0x0000FF), 0);

    lv_obj_t *header_label = lv_label_create(header);
    lv_label_set_text(header_label, "DS-K1100 Card Reader Controller");
    lv_obj_set_style_text_color(header_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(header_label);

    // RS-485 Mode Button
    btn_rs485 = lv_btn_create(main_screen);
    lv_obj_set_size(btn_rs485, 180, 50);
    lv_obj_set_pos(btn_rs485, 10, 60);
    lv_obj_add_event_cb(btn_rs485, btn_rs485_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label_rs485 = lv_label_create(btn_rs485);
    lv_label_set_text(label_rs485, "RS-485 Mode");
    lv_obj_center(label_rs485);

    // Wiegand Mode Button
    btn_wiegand = lv_btn_create(main_screen);
    lv_obj_set_size(btn_wiegand, 180, 50);
    lv_obj_set_pos(btn_wiegand, 200, 60);
    lv_obj_add_event_cb(btn_wiegand, btn_wiegand_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label_wiegand = lv_label_create(btn_wiegand);
    lv_label_set_text(label_wiegand, "Wiegand Mode");
    lv_obj_center(label_wiegand);

    // Open Door Button
    btn_open_door = lv_btn_create(main_screen);
    lv_obj_set_size(btn_open_door, 180, 50);
    lv_obj_set_pos(btn_open_door, 10, 120);
    lv_obj_add_event_cb(btn_open_door, btn_open_door_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label_open = lv_label_create(btn_open_door);
    lv_label_set_text(label_open, "Open Door");
    lv_obj_center(label_open);

    // Reset Alarm Button
    btn_reset_alarm = lv_btn_create(main_screen);
    lv_obj_set_size(btn_reset_alarm, 180, 50);
    lv_obj_set_pos(btn_reset_alarm, 200, 120);
    lv_obj_add_event_cb(btn_reset_alarm, btn_reset_alarm_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label_reset = lv_label_create(btn_reset_alarm);
    lv_label_set_text(label_reset, "Reset Alarm");
    lv_obj_center(label_reset);

    // Read Card Button
    btn_read_card = lv_btn_create(main_screen);
    lv_obj_set_size(btn_read_card, 370, 50);
    lv_obj_set_pos(btn_read_card, 10, 180);
    lv_obj_add_event_cb(btn_read_card, btn_read_card_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label_read = lv_label_create(btn_read_card);
    lv_label_set_text(label_read, "Read Card Status");
    lv_obj_center(label_read);

    // Status Labels
    label_mode = lv_label_create(main_screen);
    lv_label_set_text(label_mode, "Mode: RS-485");
    lv_obj_set_pos(label_mode, 10, 250);

    label_status = lv_label_create(main_screen);
    lv_label_set_text(label_status, "Status: Offline");
    lv_obj_set_pos(label_status, 10, 270);

    label_card = lv_label_create(main_screen);
    lv_label_set_text(label_card, "Last Card: None");
    lv_obj_set_pos(label_card, 10, 290);
}

static void gui_update_task(void *arg)
{
    char buf[64];
    
    while (1) {
        // Update status
        sprintf(buf, "Status: %s", reader_status.online ? "Online" : "Offline");
        lv_label_set_text(label_status, buf);

        // Update card ID
        if (strlen(reader_status.last_card_id) > 0) {
            sprintf(buf, "Last Card: %s", reader_status.last_card_id);
            lv_label_set_text(label_card, buf);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ============================================================================
// MAIN APPLICATION
// ============================================================================

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting DS-K1100 Card Reader Controller");

    // Initialize LVGL
    lv_init();
    lvgl_driver_init();

    // Initialize RS-485
    rs485_init();

    // Initialize Wiegand
    wiegand_init();

    // Create GUI
    create_gui();

    // Create tasks
    xTaskCreate(rs485_receive_task, "rs485_rx", 4096, NULL, 5, NULL);
    xTaskCreate(wiegand_process_task, "wiegand", 4096, NULL, 5, NULL);
    xTaskCreate(gui_update_task, "gui_update", 4096, NULL, 4, NULL);

    // LVGL task
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        lv_task_handler();
    }
}
    