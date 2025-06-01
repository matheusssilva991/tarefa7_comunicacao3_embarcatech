#include "pico/stdlib.h"     // Biblioteca da Raspberry Pi Pico para funções padrão (GPIO, temporização, etc.)
#include "pico/cyw43_arch.h" // Biblioteca para arquitetura Wi-Fi da Pico com CYW43
#include "pico/unique_id.h"  // Biblioteca com recursos para trabalhar com os pinos GPIO do Raspberry Pi Pico

#include "hardware/gpio.h" // Biblioteca de hardware de GPIO
#include "hardware/irq.h"  // Biblioteca de hardware de interrupções

#include "lwip/apps/mqtt.h"      // Biblioteca LWIP MQTT -  fornece funções e recursos para conexão MQTT
#include "lwip/apps/mqtt_priv.h" // Biblioteca que fornece funções e recursos para Geração de Conexões
#include "lwip/dns.h"            // Biblioteca que fornece funções e recursos suporte DNS:
#include "lwip/altcp_tls.h"      // Biblioteca que fornece funções e recursos para conexões seguras usando TLS:

#include "lib/ssd1306/ssd1306.h"
#include "lib/ssd1306/display.h"
#include "lib/led/led.h"
#include "lib/button/button.h"
#include "lib/ws2812b/ws2812b.h"
#include "lib/buzzer/buzzer.h"
#include "config/credential_config.h" // Inclua suas credenciais de configuração

#ifndef MQTT_SERVER
#error Need to define MQTT_SERVER
#endif

// This file includes your client certificate for client server authentication
#ifdef MQTT_CERT_INC
#include MQTT_CERT_INC
#endif

#ifndef MQTT_TOPIC_LEN
#define MQTT_TOPIC_LEN 100
#endif

// Dados do cliente MQTT
typedef struct
{
    mqtt_client_t *mqtt_client_inst;
    struct mqtt_connect_client_info_t mqtt_client_info;
    char data[MQTT_OUTPUT_RINGBUF_SIZE];
    char topic[MQTT_TOPIC_LEN];
    uint32_t len;
    ip_addr_t mqtt_server_address;
    bool connect_done;
    int subscribe_count;
    bool stop_client;
} MQTT_CLIENT_DATA_T;

#ifndef DEBUG_printf
#ifndef NDEBUG
#define DEBUG_printf printf
#else
#define DEBUG_printf(...)
#endif
#endif

#ifndef INFO_printf
#define INFO_printf printf
#endif

#ifndef ERROR_printf
#define ERROR_printf printf
#endif

#define TEMP_WORKER_TIME_S 10

// Manter o programa ativo - keep alive in seconds
#define MQTT_KEEP_ALIVE_S 60

// QoS - mqtt_subscribe
// At most once (QoS 0)
// At least once (QoS 1)
// Exactly once (QoS 2)
#define MQTT_SUBSCRIBE_QOS 1
#define MQTT_PUBLISH_QOS 1
#define MQTT_PUBLISH_RETAIN 0

// Tópico usado para: last will and testament
#define MQTT_WILL_TOPIC "/online"
#define MQTT_WILL_MSG "0"
#define MQTT_WILL_QOS 1

#ifndef MQTT_DEVICE_NAME
#define MQTT_DEVICE_NAME "pico"
#endif

// Definir como 1 para adicionar o nome do cliente aos tópicos, para suportar vários dispositivos que utilizam o mesmo servidor
#ifndef MQTT_UNIQUE_TOPIC
#define MQTT_UNIQUE_TOPIC 0
#endif

#define CYW43_LED_PIN CYW43_WL_GPIO_LED_PIN // GPIO do CI CYW43
#define PARKING_LOT_SIZE 4                  // Tamanho do estacionamento
#define LED_MATRIX_PIN 7                    // GPIO da matriz de LEDs

typedef struct parking_lot
{
    uint8_t id;                             // ID do estacionamento
    uint8_t status;                         // Status do estacionamento (0 - livre, 1 - ocupado, 2 - reservado)
    absolute_time_t reservation_start_time; // Hora de início da reserva
} parking_lot_t;

// Prototipos de funções
// Inicializa o estacionamento
void init_parking_lots(void);

// Atualiza o LED RGB de acordo com a quantidade de vagas livres
void update_led_rgb();

// Atualiza a matriz de LEDs
void update_led_matrix(void);

// Atualiza o display OLED
void update_display();

// Atualiza o buzzer
void update_buzzer();

// Atualiza os sinais de saída
void update_outputs();

// Função de callback para os botões GPIO
void gpio_callback_handler(uint gpio, uint32_t events);

// Requisição para publicar
static void pub_request_cb(__unused void *arg, err_t err);

// Topico MQTT
static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name);

// Publicar status do estacionamento
static void publish_parking_status(MQTT_CLIENT_DATA_T *state);

// Requisição de Assinatura - subscribe
static void sub_request_cb(void *arg, err_t err);

// Requisição para encerrar a assinatura
static void unsub_request_cb(void *arg, err_t err);

// Tópicos de assinatura
static void sub_unsub_topics(MQTT_CLIENT_DATA_T *state, bool sub);

// Dados de entrada MQTT
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags);

// Dados de entrada publicados
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len);

// Worker para publicar o status do estacionamento
static void parking_status_worker_fn(async_context_t *context, async_at_time_worker_t *worker);
static async_at_time_worker_t parking_status_worker = {.do_work = parking_status_worker_fn};

// Conexão MQTT
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);

// Inicializar o cliente MQTT
static void start_client(MQTT_CLIENT_DATA_T *state);

// Call back com o resultado do DNS
static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg);

static volatile parking_lot_t parking_lots[PARKING_LOT_SIZE]; // Array de estruturas para armazenar o status do estacionamento
static volatile int8_t current_parking_lot = 0;               // Vaga de estacionamento atual
static volatile int last_a = 0, last_b = 0, last_sw = 0;
const int debounce = 270;                         // Tempo de debounce para os botões
volatile bool publish_parking_status_flag = true; // Sinaliza para publicar o status do estacionamento
static volatile int free_parking_lots = 0;
static volatile int parking_lot_status[PARKING_LOT_SIZE] = {0};
ssd1306_t ssd;

int main(void)
{

    // Inicializa todos os tipos de bibliotecas stdio padrão presentes que estão ligados ao binário.
    stdio_init_all();
    init_parking_lots();  // Inicializa o estacionamento
    init_btns();          // Inicializa os botões
    init_btn(BTN_SW_PIN); // Inicializa o botão do joystick
    init_leds();          // Inicializa os LEDs
    ws2812b_init(LED_MATRIX_PIN); // Inicializa a matriz de LEDs
    init_display(&ssd); // Inicializa o display OLED
    init_buzzer(BUZZER_A_PIN, 4.0); // Inicializa o buzzer


    update_outputs(); // Atualiza os LEDs e a matriz de LEDs

    INFO_printf("mqtt client starting\n");

    // Cria registro com os dados do cliente
    static MQTT_CLIENT_DATA_T state;

    // Inicializa a arquitetura do cyw43
    if (cyw43_arch_init())
    {
        panic("Failed to inizialize CYW43");
    }

    // Usa identificador único da placa
    char unique_id_buf[5];
    pico_get_unique_board_id_string(unique_id_buf, sizeof(unique_id_buf));
    for (int i = 0; i < sizeof(unique_id_buf) - 1; i++)
    {
        unique_id_buf[i] = tolower(unique_id_buf[i]);
    }

    // Gera nome único, Ex: pico1234
    char client_id_buf[sizeof(MQTT_DEVICE_NAME) + sizeof(unique_id_buf) - 1];
    memcpy(&client_id_buf[0], MQTT_DEVICE_NAME, sizeof(MQTT_DEVICE_NAME) - 1);
    memcpy(&client_id_buf[sizeof(MQTT_DEVICE_NAME) - 1], unique_id_buf, sizeof(unique_id_buf) - 1);
    client_id_buf[sizeof(client_id_buf) - 1] = 0;
    INFO_printf("Device name %s\n", client_id_buf);

    state.mqtt_client_info.client_id = client_id_buf;
    state.mqtt_client_info.keep_alive = MQTT_KEEP_ALIVE_S; // Keep alive in sec
#if defined(MQTT_USERNAME) && defined(MQTT_PASSWORD)
    state.mqtt_client_info.client_user = MQTT_USERNAME;
    state.mqtt_client_info.client_pass = MQTT_PASSWORD;
#else
    state.mqtt_client_info.client_user = NULL;
    state.mqtt_client_info.client_pass = NULL;
#endif
    static char will_topic[MQTT_TOPIC_LEN];
    strncpy(will_topic, full_topic(&state, MQTT_WILL_TOPIC), sizeof(will_topic));
    state.mqtt_client_info.will_topic = will_topic;
    state.mqtt_client_info.will_msg = MQTT_WILL_MSG;
    state.mqtt_client_info.will_qos = MQTT_WILL_QOS;
    state.mqtt_client_info.will_retain = true;
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    // TLS enabled
#ifdef MQTT_CERT_INC
    static const uint8_t ca_cert[] = TLS_ROOT_CERT;
    static const uint8_t client_key[] = TLS_CLIENT_KEY;
    static const uint8_t client_cert[] = TLS_CLIENT_CERT;
    // This confirms the indentity of the server and the client
    state.mqtt_client_info.tls_config = altcp_tls_create_config_client_2wayauth(ca_cert, sizeof(ca_cert),
                                                                                client_key, sizeof(client_key), NULL, 0, client_cert, sizeof(client_cert));
#if ALTCP_MBEDTLS_AUTHMODE != MBEDTLS_SSL_VERIFY_REQUIRED
    WARN_printf("Warning: tls without verification is insecure\n");
#endif
#else
    state->client_info.tls_config = altcp_tls_create_config_client(NULL, 0);
    WARN_printf("Warning: tls without a certificate is insecure\n");
#endif
#endif

    // Conectar à rede WiFI - fazer um loop até que esteja conectado
    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000))
    {
        panic("Failed to connect");
    }
    INFO_printf("\nConnected to Wifi\n");

    // Faz um pedido de DNS para o endereço IP do servidor MQTT
    cyw43_arch_lwip_begin();
    int err = dns_gethostbyname(MQTT_SERVER, &state.mqtt_server_address, dns_found, &state);
    cyw43_arch_lwip_end();

    // Se tiver o endereço, inicia o cliente
    if (err == ERR_OK)
    {
        start_client(&state);
    }
    else if (err != ERR_INPROGRESS)
    { // ERR_INPROGRESS means expect a callback
        panic("dns request failed");
    }

    gpio_set_irq_enabled_with_callback(BTN_A_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback_handler);
    gpio_set_irq_enabled(BTN_B_PIN, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(BTN_SW_PIN, GPIO_IRQ_EDGE_FALL, true);

    // Loop condicionado a conexão mqtt
    while (!state.connect_done || mqtt_client_is_connected(state.mqtt_client_inst))
    {
        cyw43_arch_poll();
        cyw43_arch_wait_for_work_until(make_timeout_time_ms(10000));

        if (publish_parking_status_flag)
        {
            update_outputs(); // Atualiza os LEDs e a matriz de LEDs
            publish_parking_status(&state);
            publish_parking_status_flag = false;
        }
    }

    INFO_printf("mqtt client exiting\n");
    return 0;
}

// Inicializa o estacionamento
void init_parking_lots()
{
    for (int i = 0; i < PARKING_LOT_SIZE; i++)
    {
        parking_lots[i].id = i + 1;                 // ID do estacionamento
        parking_lots[i].status = 0;                 // Status do estacionamento (0 - livre)
        parking_lots[i].reservation_start_time = 0; // Hora de início da reserva
    }
}

// Atualiza o LED RGB de acordo com a quantidade de vagas livres
void update_led_rgb()
{
    free_parking_lots = 0; // Reseta a quantidade de vagas livres

    // Verifica a quantidade de vagas livres
    for (int i = 0; i < PARKING_LOT_SIZE; i++)
    {
        if (parking_lots[i].status == 0)
            free_parking_lots++;
    }

    // Acende uma cor no LED RGB de acordo com a quantidade de vagas livres
    if (free_parking_lots == 0)
        set_led_red();
    else if (free_parking_lots > PARKING_LOT_SIZE / 2)
    {
        set_led_green();
    }
    else
        set_led_yellow();
}

// Atualiza a matriz de LEDs
void update_led_matrix()
{
    INFO_printf("Updating LED matrix\n");
    int parking_lot_positions[PARKING_LOT_SIZE][4] = {
        {15, 16, 23, 24},
        {18, 19, 20, 21},
        {3, 4, 5, 6},
        {0, 1, 8, 9},
    };

    int color[3] = {0, 0, 0};

    for (int i = 0; i < PARKING_LOT_SIZE; i++)
    {
        color[0] = 0; // Vermelho
        color[1] = 0; // Verde
        color[2] = 0; // Azul

        if (parking_lots[i].status == 0)
            color[1] = 8; // Verde
        else if (parking_lots[i].status == 1)
            color[0] = 8; // Vermelho
        else if (parking_lots[i].status == 2)
        {
            color[0] = 4; // Amarelo
            color[1] = 8;
        }

        for (int j = 0; j < 4; j++)
            ws2812b_draw_point(parking_lot_positions[i][j], color);
    }

    ws2812b_write();
}

// Atualiza o display OLED
void update_display()
{
    ssd1306_fill(&ssd, false); // Limpa a tela
    draw_centered_text(&ssd, "Estacionamento", 0);
    ssd1306_draw_string(&ssd, "Vagas:", 0, 15);

    for (int i = 0; i < PARKING_LOT_SIZE; i++)
    {
        const char *status_text = (parking_lots[i].status == 0) ? "Livre" : (parking_lots[i].status == 1) ? "Ocupada"
                                                                        : (parking_lots[i].status == 2)   ? "Reservada"
                                                                                                            : "Indefinida";

        char buffer[20];

        snprintf(buffer, sizeof(buffer), "%d: %s", i + 1, status_text);
        ssd1306_draw_string(&ssd, buffer, 5, (i * 10) + 25);
    }

    ssd1306_send_data(&ssd);        // Envia os dados para o display
}

// Atualiza o buzzer
void update_buzzer()
{
    // Verifica qual foi a mudança de status
    for (int i = 0; i < PARKING_LOT_SIZE; i++)
    {
        if (parking_lots[i].status != parking_lot_status[i])
        {
            parking_lot_status[i] = parking_lots[i].status;

            // Toca o buzzer se a vaga estiver ocupada
            if (parking_lots[i].status == 1)
            {
                play_tone(BUZZER_A_PIN, 300);
                sleep_ms(250); // Toca o buzzer por 250ms
                stop_tone(BUZZER_A_PIN);
            }
            // Toca o buzzer se a vaga estiver livre
            else if (parking_lots[i].status == 0)
            {
                play_tone(BUZZER_A_PIN, 2000);
                sleep_ms(250); // Toca o buzzer por 250ms
                stop_tone(BUZZER_A_PIN);
            }
            // Toca o buzzer se a vaga estiver reservada
            else if (parking_lots[i].status == 2)
            {
                play_tone(BUZZER_A_PIN, 900);
                sleep_ms(250); // Toca o buzzer por 250ms
                stop_tone(BUZZER_A_PIN);
            }
        }
    }
}

// Atualiza os sinais de saída
void update_outputs()
{
    // Atualiza o LED RGB
    update_led_rgb();

    // Atualiza a matriz de LEDs
    update_led_matrix();

    // Atualiza o display OLED
    update_display();

    // Atualiza o buzzer
    update_buzzer();
    INFO_printf("Outputs updated: Free parking lots: %d\n", free_parking_lots);
}

// Função de callback para os botões GPIO
void gpio_callback_handler(uint gpio, uint32_t events)
{
    int now = to_ms_since_boot(get_absolute_time()); // Obtém o tempo atual em milissegundos

    // Verifica se o botão A está pressionado
    if (btn_is_pressed(BTN_A_PIN) && (now - last_a) > debounce)
    {
        last_a = now; // Atualiza o último tempo em que o botão A foi pressionado

        if (current_parking_lot > 0)
            current_parking_lot--;
    }
    else if (btn_is_pressed(BTN_B_PIN) && (now - last_b) > debounce)
    {
        last_b = now; // Atualiza o último tempo em que o botão B foi pressionado

        if (current_parking_lot < PARKING_LOT_SIZE - 1)
            current_parking_lot++;
    }
    else if (btn_is_pressed(BTN_SW_PIN) && (now - last_sw) > debounce)
    {
        last_sw = now;
        if (parking_lots[current_parking_lot].status == 0 || parking_lots[current_parking_lot].status == 2)
            parking_lots[current_parking_lot].status = 1;
        else if (parking_lots[current_parking_lot].status == 1)
            parking_lots[current_parking_lot].status = 0;

        publish_parking_status_flag = true; // Sinaliza para publicar depois
        INFO_printf("Parking lot %d status: %d\n", parking_lots[current_parking_lot].id, parking_lots[current_parking_lot].status);
    }
}

// Requisição para publicar
static void pub_request_cb(__unused void *arg, err_t err)
{
    if (err != 0)
    {
        ERROR_printf("pub_request_cb failed %d", err);
    }
}

// Topico MQTT
static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name)
{
#if MQTT_UNIQUE_TOPIC
    static char full_topic[MQTT_TOPIC_LEN];
    snprintf(full_topic, sizeof(full_topic), "/%s%s", state->mqtt_client_info.client_id, name);
    return full_topic;
#else
    return name;
#endif
}

// Publicar status do estacionamento
static void publish_parking_status(MQTT_CLIENT_DATA_T *state)
{
    char topic[MQTT_TOPIC_LEN];
    char msg[32];
    for (int i = 0; i < PARKING_LOT_SIZE; i++)
    {
        snprintf(topic, sizeof(topic), "%s%d", full_topic(state, "/parking/status/"), parking_lots[i].id);
        snprintf(msg, sizeof(msg), "%d", parking_lots[i].status);
        mqtt_publish(state->mqtt_client_inst, topic, msg, strlen(msg), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }
}

// Requisição de Assinatura - subscribe
static void sub_request_cb(void *arg, err_t err)
{
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T *)arg;
    if (err != 0)
    {
        panic("subscribe request failed %d", err);
    }
    state->subscribe_count++;
}

// Requisição para encerrar a assinatura
static void unsub_request_cb(void *arg, err_t err)
{
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T *)arg;
    if (err != 0)
    {
        panic("unsubscribe request failed %d", err);
    }
    state->subscribe_count--;
    assert(state->subscribe_count >= 0);

    // Stop if requested
    if (state->subscribe_count <= 0 && state->stop_client)
    {
        mqtt_disconnect(state->mqtt_client_inst);
    }
}

// Tópicos de assinatura
static void sub_unsub_topics(MQTT_CLIENT_DATA_T *state, bool sub)
{
    mqtt_request_cb_t cb = sub ? sub_request_cb : unsub_request_cb;
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/print"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/ping"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/exit"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/parking/+/reservation"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
}

// Dados de entrada MQTT
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T *)arg;
#if MQTT_UNIQUE_TOPIC
    const char *basic_topic = state->topic + strlen(state->mqtt_client_info.client_id) + 1;
#else
    const char *basic_topic = state->topic;
#endif
    strncpy(state->data, (const char *)data, len);
    state->len = len;
    state->data[len] = '\0';

    DEBUG_printf("Topic: %s, Message: %s\n", state->topic, state->data);

    if (strcmp(basic_topic, "/print") == 0)
    {
        INFO_printf("%.*s\n", len, data);
    }
    else if (strcmp(basic_topic, "/ping") == 0)
    {
        char buf[11];
        snprintf(buf, sizeof(buf), "%u", to_ms_since_boot(get_absolute_time()) / 1000);
        mqtt_publish(state->mqtt_client_inst, full_topic(state, "/uptime"), buf, strlen(buf), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }
    else if (strcmp(basic_topic, "/exit") == 0)
    {
        state->stop_client = true;      // stop the client when ALL subscriptions are stopped
        sub_unsub_topics(state, false); // unsubscribe
    }
    else if (strncmp(basic_topic, "/parking/", 9) == 0)
    {
        int id;
        if (sscanf(basic_topic, "/parking/%d/reservation", &id) == 1)
        {
            if (id >= 1 && id <= PARKING_LOT_SIZE)
            {
                int index = id - 1;
                if (parking_lots[index].status == 0)
                {
                    parking_lots[index].status = 2; // 2 = reservado
                    parking_lots[index].reservation_start_time = get_absolute_time();
                    update_outputs(); // Atualiza os LEDs e a matriz de LEDs
                    INFO_printf("Reserva recebida para vaga %d\n", id);

                   // Publique imediatamente o novo status
                    publish_parking_status(state);
                }
                else
                {
                    INFO_printf("Vaga %d já está ocupada\n", id);
                }
            }
            else
            {
                INFO_printf("ID de vaga inválido: %d\n", id);
            }
        }
    }


}

// Dados de entrada publicados
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T *)arg;
    strncpy(state->topic, topic, sizeof(state->topic));
}

// Publicar status do estacionamento
static void parking_status_worker_fn(async_context_t *context, async_at_time_worker_t *worker)
{
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T *)worker->user_data;
    publish_parking_status(state);
    async_context_add_at_time_worker_in_ms(context, worker, TEMP_WORKER_TIME_S * 1000);
}

// Conexão MQTT
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T *)arg;
    if (status == MQTT_CONNECT_ACCEPTED)
    {
        state->connect_done = true;
        sub_unsub_topics(state, true); // subscribe;

        // indicate online
        if (state->mqtt_client_info.will_topic)
        {
            mqtt_publish(state->mqtt_client_inst, state->mqtt_client_info.will_topic, "1", 1, MQTT_WILL_QOS, true, pub_request_cb, state);
        }

        // Publish parking status every 10 sec
        parking_status_worker.user_data = state;
        async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(), &parking_status_worker, 0);
    }
    else if (status == MQTT_CONNECT_DISCONNECTED)
    {
        if (!state->connect_done)
        {
            panic("Failed to connect to mqtt server");
        }
    }
    else
    {
        panic("Unexpected status");
    }
}

// Inicializar o cliente MQTT
static void start_client(MQTT_CLIENT_DATA_T *state)
{
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    const int port = MQTT_TLS_PORT;
    INFO_printf("Using TLS\n");
#else
    const int port = MQTT_PORT;
    INFO_printf("Warning: Not using TLS\n");
#endif

    state->mqtt_client_inst = mqtt_client_new();
    if (!state->mqtt_client_inst)
    {
        panic("MQTT client instance creation error");
    }
    INFO_printf("IP address of this device %s\n", ipaddr_ntoa(&(netif_list->ip_addr)));
    INFO_printf("Connecting to mqtt server at %s\n", ipaddr_ntoa(&state->mqtt_server_address));

    cyw43_arch_lwip_begin();
    if (mqtt_client_connect(state->mqtt_client_inst, &state->mqtt_server_address, port, mqtt_connection_cb, state, &state->mqtt_client_info) != ERR_OK)
    {
        panic("MQTT broker connection error");
    }
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    // This is important for MBEDTLS_SSL_SERVER_NAME_INDICATION
    mbedtls_ssl_set_hostname(altcp_tls_context(state->mqtt_client_inst->conn), MQTT_SERVER);
#endif
    mqtt_set_inpub_callback(state->mqtt_client_inst, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, state);
    cyw43_arch_lwip_end();
}

// Call back com o resultado do DNS
static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg)
{
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T *)arg;
    if (ipaddr)
    {
        state->mqtt_server_address = *ipaddr;
        start_client(state);
    }
    else
    {
        panic("dns request failed");
    }
}