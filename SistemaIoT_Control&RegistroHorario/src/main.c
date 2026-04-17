#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#include "driver/ledc.h"
#include "driver/gpio.h"

#include "rc522.h"
#include "driver/rc522_spi.h"
#include "rc522_picc.h"

#include "HD44780.h"

static const char *TAG = "ACCESS_SYSTEM";

// =====================================================
// LCD I2C
// =====================================================
#define LCD_ADDR 0x27
#define LCD_SDA_PIN 21
#define LCD_SCL_PIN 22
#define LCD_COLS 20
#define LCD_ROWS 4

// =====================================================
// RC522 RFID
// =====================================================
#define RC522_PIN_MISO 34
#define RC522_PIN_MOSI 13
#define RC522_PIN_SCK  14
#define RC522_PIN_CS   27
#define RC522_PIN_RST  26

// =====================================================
// BUZZER
// =====================================================
#define BUZZER_PIN 25

#define BUZZER_LEDC_MODE       LEDC_LOW_SPEED_MODE
#define BUZZER_LEDC_TIMER      LEDC_TIMER_0
#define BUZZER_LEDC_CHANNEL    LEDC_CHANNEL_0
#define BUZZER_LEDC_DUTY_RES   LEDC_TIMER_8_BIT
#define BUZZER_FREQ_HZ         2000
#define BUZZER_DUTY            128

// =====================================================
// SERVO
// =====================================================
#define SERVO_PIN              23
#define SERVO_LEDC_MODE        LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_TIMER       LEDC_TIMER_1
#define SERVO_LEDC_CHANNEL     LEDC_CHANNEL_1
#define SERVO_MIN_DUTY         205
#define SERVO_MAX_DUTY         1024

// =====================================================
// HC-SR04
// =====================================================
#define TRIG_PIN GPIO_NUM_32
#define ECHO_PIN GPIO_NUM_33
#define SOUND_SPEED_CM_PER_US 0.0343f
#define DIST_THRESHOLD_CM     20.0f

// Tiempo máximo esperando que alguien se acerque al sensor
#define ACCESS_WAIT_TIMEOUT_US 5000000LL   // 5 segundos

// =====================================================
// UIDs autorizados
// =====================================================
static const uint8_t UID_AUT_1[] = {0x59, 0xEF, 0xA0, 0x03};
static const uint8_t UID_AUT_2[] = {0x49, 0x35, 0x21, 0x7A};

// =====================================================
// Tipos y variables globales
// =====================================================
typedef enum {
    SYSTEM_IDLE = 0,
    SYSTEM_AUTH_WAIT_SENSOR
} system_state_t;

typedef struct {
    bool authorized;
    char uid_str[RC522_PICC_UID_STR_BUFFER_SIZE_MAX];
} access_event_t;

static rc522_driver_handle_t driver;
static rc522_handle_t scanner;
static QueueHandle_t access_queue = NULL;

// =====================================================
// Helpers LCD
// =====================================================
static void lcd_write_line(int row, const char *text)
{
    char buffer[LCD_COLS + 1];
    snprintf(buffer, sizeof(buffer), "%-20.20s", text);
    LCD_setCursor(0, row);
    LCD_writeStr(buffer);
}

static void lcd_show_idle(void)
{
    LCD_clearScreen();
    lcd_write_line(0, "Sistema listo");
    lcd_write_line(1, "Acerque tarjeta");
    lcd_write_line(2, "Esperando RFID...");
    lcd_write_line(3, "Servo: 0 grados");
}

static void lcd_show_denied(const char *uid)
{
    LCD_clearScreen();
    lcd_write_line(0, "ACCESO DENEGADO");
    lcd_write_line(1, uid);
    lcd_write_line(2, "Tarjeta no valida");
    lcd_write_line(3, "Intente de nuevo");
}

static void lcd_show_authorized_wait(const char *uid, float dist)
{
    char line2[21];
    char line3[21];

    LCD_clearScreen();
    lcd_write_line(0, "ACCESO CONCEDIDO");
    lcd_write_line(1, uid);

    if (dist < 0.0f) {
        snprintf(line2, sizeof(line2), "Dist: fuera rango");
    } else {
        snprintf(line2, sizeof(line2), "Dist: %.2f cm", dist);
    }

    snprintf(line3, sizeof(line3), "< %.0f cm para abrir", DIST_THRESHOLD_CM);

    lcd_write_line(2, line2);
    lcd_write_line(3, line3);
}

static void lcd_show_servo_open(const char *uid, float dist)
{
    char line2[21];

    LCD_clearScreen();
    lcd_write_line(0, "ACCESO CONCEDIDO");
    lcd_write_line(1, "Puerta abierta");
    snprintf(line2, sizeof(line2), "Dist: %.2f cm", dist);
    lcd_write_line(2, line2);
    lcd_write_line(3, uid);
}

static void lcd_show_timeout(void)
{
    LCD_clearScreen();
    lcd_write_line(0, "Tiempo agotado");
    lcd_write_line(1, "No hubo deteccion");
    lcd_write_line(2, "Vuelve a intentar");
    lcd_write_line(3, "Estado inicial");
}

// =====================================================
// Buzzer
// =====================================================
static void buzzer_init(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode      = BUZZER_LEDC_MODE,
        .duty_resolution = BUZZER_LEDC_DUTY_RES,
        .timer_num       = BUZZER_LEDC_TIMER,
        .freq_hz         = BUZZER_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t channel_conf = {
        .gpio_num   = BUZZER_PIN,
        .speed_mode = BUZZER_LEDC_MODE,
        .channel    = BUZZER_LEDC_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = BUZZER_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));
}

static void buzzer_on(void)
{
    ESP_ERROR_CHECK(ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, BUZZER_DUTY));
    ESP_ERROR_CHECK(ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL));
}

static void buzzer_off(void)
{
    ESP_ERROR_CHECK(ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0));
    ESP_ERROR_CHECK(ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL));
}

static void beep_access_granted(void)
{
    buzzer_on();
    vTaskDelay(pdMS_TO_TICKS(120));
    buzzer_off();
}

static void beep_access_denied(void)
{
    for (int i = 0; i < 2; i++) {
        buzzer_on();
        vTaskDelay(pdMS_TO_TICKS(90));
        buzzer_off();
        vTaskDelay(pdMS_TO_TICKS(90));
    }
}

// =====================================================
// Servo
// =====================================================
static void servo_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode      = SERVO_LEDC_MODE,
        .timer_num       = SERVO_LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz         = 50,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode = SERVO_LEDC_MODE,
        .channel    = SERVO_LEDC_CHANNEL,
        .timer_sel  = SERVO_LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = SERVO_PIN,
        .duty       = 0,
        .hpoint     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

static void set_servo_angle(uint32_t angle)
{
    if (angle > 180) angle = 180;

    uint32_t duty = SERVO_MIN_DUTY +
                    (((SERVO_MAX_DUTY - SERVO_MIN_DUTY) * angle) / 180);

    ESP_ERROR_CHECK(ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL));
}

// =====================================================
// Ultrasonido
// =====================================================
static void ultrasonic_init(void)
{
    gpio_reset_pin(TRIG_PIN);
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(TRIG_PIN, 0);

    gpio_reset_pin(ECHO_PIN);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
}

static float medir_distancia_cm(void)
{
    gpio_set_level(TRIG_PIN, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_PIN, 0);

    int64_t timeout_start = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 0) {
        if ((esp_timer_get_time() - timeout_start) > 30000) {
            return -1.0f;
        }
    }

    int64_t echo_start = esp_timer_get_time();

    while (gpio_get_level(ECHO_PIN) == 1) {
        if ((esp_timer_get_time() - echo_start) > 30000) {
            return -1.0f;
        }
    }

    int64_t echo_end = esp_timer_get_time();

    float distance = (float)(echo_end - echo_start) * SOUND_SPEED_CM_PER_US / 2.0f;
    return distance;
}

// =====================================================
// RFID autorización
// =====================================================
static bool uid_equals(const rc522_picc_uid_t *uid, const uint8_t *ref_uid, uint8_t ref_len)
{
    if (uid->length != ref_len) {
        return false;
    }

    for (uint8_t i = 0; i < ref_len; i++) {
        if (uid->value[i] != ref_uid[i]) {
            return false;
        }
    }

    return true;
}

static bool is_authorized(const rc522_picc_uid_t *uid)
{
    if (uid_equals(uid, UID_AUT_1, sizeof(UID_AUT_1))) {
        return true;
    }

    if (uid_equals(uid, UID_AUT_2, sizeof(UID_AUT_2))) {
        return true;
    }

    return false;
}

// =====================================================
// Callback RFID
// =====================================================
static void on_picc_state_changed(void *arg, esp_event_base_t base, int32_t event_id, void *data)
{
    rc522_picc_state_changed_event_t *event = (rc522_picc_state_changed_event_t *)data;
    rc522_picc_t *picc = event->picc;

    if (picc->state == RC522_PICC_STATE_ACTIVE) {
        access_event_t access_event = {0};

        rc522_picc_uid_to_str(&picc->uid, access_event.uid_str, sizeof(access_event.uid_str));
        access_event.authorized = is_authorized(&picc->uid);

        if (access_queue != NULL) {
            xQueueSend(access_queue, &access_event, 0);
        }
    } else if (picc->state == RC522_PICC_STATE_IDLE &&
               event->old_state >= RC522_PICC_STATE_ACTIVE) {
        ESP_LOGI(TAG, "Tarjeta retirada");
    }
}

// =====================================================
// RFID init
// =====================================================
static void rfid_init(void)
{
    rc522_spi_config_t driver_config = {
        .host_id = SPI3_HOST,
        .bus_config = &(spi_bus_config_t) {
            .miso_io_num = RC522_PIN_MISO,
            .mosi_io_num = RC522_PIN_MOSI,
            .sclk_io_num = RC522_PIN_SCK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
        },
        .dev_config = {
            .spics_io_num = RC522_PIN_CS,
        },
        .rst_io_num = RC522_PIN_RST,
    };

    rc522_spi_create(&driver_config, &driver);
    rc522_driver_install(driver);

    rc522_config_t scanner_config = {
        .driver = driver,
    };

    rc522_create(&scanner_config, &scanner);
    rc522_register_events(scanner, RC522_EVENT_PICC_STATE_CHANGED, on_picc_state_changed, NULL);
    rc522_start(scanner);
}

// =====================================================
// Reset del sistema al estado inicial
// =====================================================
static void reset_to_idle(system_state_t *state, bool *servo_open, char *current_uid)
{
    set_servo_angle(0);
    *servo_open = false;
    memset(current_uid, 0, RC522_PICC_UID_STR_BUFFER_SIZE_MAX);
    *state = SYSTEM_IDLE;
    lcd_show_idle();
}

// =====================================================
// app_main
// =====================================================
void app_main(void)
{
    system_state_t state = SYSTEM_IDLE;
    bool servo_open = false;
    char current_uid[RC522_PICC_UID_STR_BUFFER_SIZE_MAX] = {0};
    int64_t auth_start_time_us = 0;

    ESP_LOGI(TAG, "Iniciando sistema integrado...");

    access_queue = xQueueCreate(5, sizeof(access_event_t));
    if (access_queue == NULL) {
        ESP_LOGE(TAG, "No se pudo crear la cola de acceso");
        return;
    }

    LCD_init(LCD_ADDR, LCD_SDA_PIN, LCD_SCL_PIN, LCD_COLS, LCD_ROWS);
    buzzer_init();
    servo_init();
    ultrasonic_init();
    rfid_init();

    set_servo_angle(0);
    vTaskDelay(pdMS_TO_TICKS(300));
    lcd_show_idle();

    ESP_LOGI(TAG, "Sistema listo. Acerque una tarjeta.");

    while (1) {
        access_event_t evt;

        if (xQueueReceive(access_queue, &evt, pdMS_TO_TICKS(100)) == pdPASS) {

            // Si ya había un acceso concedido esperando sensor, lo cancelamos
            // y procesamos la nueva tarjeta desde cero.
            if (state == SYSTEM_AUTH_WAIT_SENSOR) {
                ESP_LOGW(TAG, "Nueva tarjeta detectada. Cancelando intento anterior.");
                reset_to_idle(&state, &servo_open, current_uid);
                vTaskDelay(pdMS_TO_TICKS(150));
            }

            ESP_LOGI(TAG, "Tarjeta detectada");
            ESP_LOGI(TAG, "UID: %s", evt.uid_str);

            if (evt.authorized) {
                ESP_LOGI(TAG, ">>> ACCESO CONCEDIDO <<<");

                strncpy(current_uid, evt.uid_str, sizeof(current_uid) - 1);
                current_uid[sizeof(current_uid) - 1] = '\0';

                beep_access_granted();
                lcd_show_authorized_wait(current_uid, -1.0f);

                state = SYSTEM_AUTH_WAIT_SENSOR;
                servo_open = false;
                set_servo_angle(0);
                auth_start_time_us = esp_timer_get_time();
            } else {
                ESP_LOGW(TAG, ">>> ACCESO DENEGADO <<<");

                beep_access_denied();
                set_servo_angle(0);
                servo_open = false;

                lcd_show_denied(evt.uid_str);
                vTaskDelay(pdMS_TO_TICKS(1500));

                reset_to_idle(&state, &servo_open, current_uid);
            }
        }

        if (state == SYSTEM_AUTH_WAIT_SENSOR) {
            float dist = medir_distancia_cm();
            int64_t now_us = esp_timer_get_time();

            // Si no se abrió el servo y ya pasaron más de 5 segundos, cancelar acceso
            if (!servo_open && (now_us - auth_start_time_us > ACCESS_WAIT_TIMEOUT_US)) {
                ESP_LOGW(TAG, "Tiempo de espera agotado. Volviendo a estado inicial.");
                lcd_show_timeout();
                vTaskDelay(pdMS_TO_TICKS(1200));
                reset_to_idle(&state, &servo_open, current_uid);
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }

            if (dist > 0.0f && dist < DIST_THRESHOLD_CM) {
                if (!servo_open) {
                    ESP_LOGI(TAG, "Distancia %.2f cm < %.2f cm -> Servo a 180 grados",
                             dist, DIST_THRESHOLD_CM);
                    set_servo_angle(180);
                    servo_open = true;
                }

                lcd_show_servo_open(current_uid, dist);
            } else {
                if (servo_open) {
                    ESP_LOGI(TAG, "Condicion no cumplida -> Servo a 0 grados");
                    set_servo_angle(0);
                    servo_open = false;
                    reset_to_idle(&state, &servo_open, current_uid);
                } else {
                    lcd_show_authorized_wait(current_uid, dist);
                }
            }

            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}