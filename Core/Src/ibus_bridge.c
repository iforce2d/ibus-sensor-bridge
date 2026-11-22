#include "main.h"
#include "ibus_bridge.h"

#define IBUS_SENSOR_ID_MAX 15U
#define IBUS_FRAME_MAX_LENGTH 8U
#define IBUS_FRAME_REQUEST_LENGTH 4U
#define IBUS_FC_QUERY_INTERVAL_MS 8U
#define IBUS_FC_REPLY_TIMEOUT_MS 6U
#define IBUS_FC_RECOVERY_TIMEOUT_MS 1500U
#define IBUS_FC_RECOVERY_RETRY_MS 2000U
#define IBUS_FC_LINK_LED_TIMEOUT_MS 100U

typedef struct {
  uint8_t bytes[2][IBUS_FRAME_MAX_LENGTH];
  uint8_t length[2];
  volatile uint8_t active;
  volatile uint8_t valid;
} IBusCachedFrame;

typedef struct {
  volatile uint8_t present;
  IBusCachedFrame discovery;
  IBusCachedFrame type;
  IBusCachedFrame value;
  volatile uint8_t sensor_length;
} IBusSensorCache;

typedef enum {
  FC_SCAN_DISCOVER,
  FC_SCAN_TYPE,
  FC_SCAN_VALUE,
  FC_REFRESH_VALUE,
  FC_SCAN_FINISHED
} FCScanStage;

static UART_HandleTypeDef huart_fc;
static UART_HandleTypeDef huart_receiver;
static IBusSensorCache sensor_cache[IBUS_SENSOR_ID_MAX + 1U];

static volatile uint32_t last_fc_reply_ms;
static volatile uint8_t fc_valid_reply_seen;
static uint32_t fc_last_recovery_ms;
static uint8_t fc_recovery_started;

static uint8_t fc_rx_byte;
static uint8_t fc_rx_frame[IBUS_FRAME_MAX_LENGTH];
static volatile uint8_t fc_rx_index;
static volatile uint8_t fc_rx_expected_length;
static volatile uint8_t fc_response_ready;
static volatile uint8_t fc_response_length;
static uint8_t fc_query_command;
static uint8_t fc_query_sensor_id;
static uint8_t fc_waiting;
static uint8_t fc_scan_sensor_id = 1U;
static uint8_t fc_sensor_count;
static FCScanStage fc_scan_stage = FC_SCAN_DISCOVER;
static uint32_t fc_query_started_ms;
static uint32_t fc_next_query_ms;

static uint8_t receiver_rx_byte;
static uint8_t receiver_request[IBUS_FRAME_REQUEST_LENGTH];
static uint8_t receiver_request_index;
static uint8_t receiver_tx_frame[IBUS_FRAME_MAX_LENGTH];
static uint8_t receiver_tx_length;

static void note_fc_reply(void)
{
  last_fc_reply_ms = HAL_GetTick();
  fc_valid_reply_seen = 1U;
  fc_recovery_started = 0U;
}

uint8_t ibus_bridge_fc_link_healthy(void)
{
  return (uint8_t)((fc_valid_reply_seen != 0U) &&
                   ((uint32_t)(HAL_GetTick() - last_fc_reply_ms) <=
                    IBUS_FC_LINK_LED_TIMEOUT_MS));
}

static void restart_fc_link(uint32_t now)
{
  /* Reset the FC UART and parser state, but resume the existing sensor scan. */
  HAL_NVIC_DisableIRQ(USART1_IRQn);
  fc_waiting = 0U;
  fc_response_ready = 0U;
  fc_rx_index = 0U;
  fc_rx_expected_length = 0U;

  (void)HAL_UART_AbortReceive(&huart_fc);
  (void)HAL_UART_AbortTransmit(&huart_fc);
  __HAL_UART_CLEAR_PEFLAG(&huart_fc);

  if (HAL_HalfDuplex_Init(&huart_fc) == HAL_OK) {
    (void)HAL_HalfDuplex_EnableReceiver(&huart_fc);
  }
  NVIC_ClearPendingIRQ(USART1_IRQn);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  fc_next_query_ms = now + 10U;
}

static uint16_t ibus_checksum(const uint8_t *bytes, uint8_t length)
{
  uint16_t sum = 0U;
  for (uint8_t i = 0U; i < length; i++) {
    sum += bytes[i];
  }
  return (uint16_t)(0xFFFFU - sum);
}

static IBusCachedFrame *cache_for_command(IBusSensorCache *sensor, uint8_t command)
{
  if (command == 0x80U) {
    return &sensor->discovery;
  }
  if (command == 0x90U) {
    return &sensor->type;
  }
  if (command == 0xA0U) {
    return &sensor->value;
  }
  return NULL;
}

static void cache_frame(IBusSensorCache *sensor, uint8_t command,
                        const uint8_t *bytes, uint8_t length)
{
  IBusCachedFrame *cache = cache_for_command(sensor, command);
  if ((cache == NULL) || (length > IBUS_FRAME_MAX_LENGTH)) {
    return;
  }

  const uint8_t next = (uint8_t)(cache->active ^ 1U);
  for (uint8_t i = 0U; i < length; i++) {
    cache->bytes[next][i] = bytes[i];
  }
  cache->length[next] = length;
  cache->active = next;
  cache->valid = 1U;
}

static uint8_t copy_cached_frame(uint8_t sensor_id, uint8_t command)
{
  if ((sensor_id == 0U) || (sensor_id > IBUS_SENSOR_ID_MAX)) {
    return 0U;
  }

  IBusSensorCache *sensor = &sensor_cache[sensor_id];
  if ((command == 0x80U) && (sensor->present == 0U)) {
    return 0U;
  }

  IBusCachedFrame *cache = cache_for_command(sensor, command);
  if ((cache == NULL) || (cache->valid == 0U)) {
    return 0U;
  }

  const uint8_t active = cache->active;
  const uint8_t length = cache->length[active];
  if ((length < 4U) || (length > IBUS_FRAME_MAX_LENGTH)) {
    return 0U;
  }
  for (uint8_t i = 0U; i < length; i++) {
    receiver_tx_frame[i] = cache->bytes[active][i];
  }
  return length;
}

static void arm_receiver_uart(void)
{
  if (HAL_HalfDuplex_EnableReceiver(&huart_receiver) == HAL_OK) {
    (void)HAL_UART_Receive_IT(&huart_receiver, &receiver_rx_byte, 1U);
  }
}

static void start_receiver_reply(void)
{
  const uint16_t request_checksum = (uint16_t)receiver_request[2] |
                                    ((uint16_t)receiver_request[3] << 8);
  if (ibus_checksum(receiver_request, 2U) != request_checksum) {
    arm_receiver_uart();
    return;
  }

  const uint8_t command = receiver_request[1] & 0xF0U;
  const uint8_t sensor_id = receiver_request[1] & 0x0FU;
  receiver_tx_length = copy_cached_frame(sensor_id, command);
  if (receiver_tx_length == 0U) {
    arm_receiver_uart();
    return;
  }

  if ((HAL_HalfDuplex_EnableTransmitter(&huart_receiver) != HAL_OK) ||
      (HAL_UART_Transmit_IT(&huart_receiver, receiver_tx_frame,
                           receiver_tx_length) != HAL_OK)) {
    arm_receiver_uart();
    return;
  }
}

static void receive_receiver_byte(void)
{
  if (receiver_request_index == 0U) {
    if (receiver_rx_byte != IBUS_FRAME_REQUEST_LENGTH) {
      (void)HAL_UART_Receive_IT(&huart_receiver, &receiver_rx_byte, 1U);
      return;
    }
  }

  receiver_request[receiver_request_index++] = receiver_rx_byte;
  if (receiver_request_index >= IBUS_FRAME_REQUEST_LENGTH) {
    receiver_request_index = 0U;
    start_receiver_reply();
  } else {
    (void)HAL_UART_Receive_IT(&huart_receiver, &receiver_rx_byte, 1U);
  }
}

static void receive_fc_byte(void)
{
  if (fc_rx_index == 0U) {
    if ((fc_rx_byte < 4U) || (fc_rx_byte > IBUS_FRAME_MAX_LENGTH)) {
      (void)HAL_UART_Receive_IT(&huart_fc, &fc_rx_byte, 1U);
      return;
    }
    fc_rx_expected_length = fc_rx_byte;
  }

  fc_rx_frame[fc_rx_index++] = fc_rx_byte;
  if (fc_rx_index >= fc_rx_expected_length) {
    fc_response_length = fc_rx_expected_length;
    fc_response_ready = 1U;
  } else {
    (void)HAL_UART_Receive_IT(&huart_fc, &fc_rx_byte, 1U);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
  if (uart == &huart_receiver) {
    receive_receiver_byte();
  } else if (uart == &huart_fc) {
    receive_fc_byte();
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
  if (uart == &huart_receiver) {
    arm_receiver_uart();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
  if (uart == &huart_receiver) {
    (void)HAL_UART_AbortReceive(uart);
    (void)HAL_UART_AbortTransmit(uart);
    receiver_request_index = 0U;
    arm_receiver_uart();
  } else if (uart == &huart_fc) {
    (void)HAL_UART_AbortReceive(uart);
    __HAL_UART_CLEAR_PEFLAG(uart);
    fc_rx_index = 0U;
    fc_rx_expected_length = 0U;
    fc_response_ready = 0U;
    if (HAL_HalfDuplex_EnableReceiver(uart) == HAL_OK) {
      (void)HAL_UART_Receive_IT(uart, &fc_rx_byte, 1U);
    }
  }
}

static void finish_fc_scan(uint8_t sensor_count)
{
  fc_sensor_count = sensor_count;
  fc_scan_sensor_id = 1U;
  if (sensor_count == 0U) {
    /* The FC may still be booting; retry discovery instead of giving up. */
    fc_scan_stage = FC_SCAN_DISCOVER;
    fc_next_query_ms = HAL_GetTick() + 1000U;
  } else {
    fc_scan_stage = FC_REFRESH_VALUE;
    fc_next_query_ms = HAL_GetTick() + IBUS_FC_QUERY_INTERVAL_MS;
  }
}

static void advance_fc_scan_after_value(void)
{
  if (fc_scan_sensor_id >= IBUS_SENSOR_ID_MAX) {
    finish_fc_scan(IBUS_SENSOR_ID_MAX);
  } else {
    fc_scan_sensor_id++;
    fc_scan_stage = FC_SCAN_DISCOVER;
  }
}

static void advance_fc_refresh(void)
{
  if (fc_scan_sensor_id >= fc_sensor_count) {
    fc_scan_sensor_id = 1U;
  } else {
    fc_scan_sensor_id++;
  }
}

static uint8_t expected_fc_command(void)
{
  switch (fc_scan_stage) {
  case FC_SCAN_DISCOVER:
    return 0x80U;
  case FC_SCAN_TYPE:
    return 0x90U;
  case FC_SCAN_VALUE:
  case FC_REFRESH_VALUE:
    return 0xA0U;
  default:
    return 0U;
  }
}

static void process_fc_response(void)
{
  const uint8_t length = fc_response_length;
  fc_response_ready = 0U;
  fc_rx_index = 0U;

  if ((length < 4U) || (length > IBUS_FRAME_MAX_LENGTH)) {
    return;
  }
  if (ibus_checksum(fc_rx_frame, (uint8_t)(length - 2U)) !=
      ((uint16_t)fc_rx_frame[length - 2U] |
       ((uint16_t)fc_rx_frame[length - 1U] << 8))) {
    return;
  }

  const uint8_t command = fc_rx_frame[1] & 0xF0U;
  const uint8_t sensor_id = fc_rx_frame[1] & 0x0FU;
  if ((command != fc_query_command) || (sensor_id != fc_query_sensor_id)) {
    return;
  }

  IBusSensorCache *sensor = &sensor_cache[sensor_id];
  if (command == 0x80U) {
    if (length != 4U) {
      return;
    }
    cache_frame(sensor, command, fc_rx_frame, length);
    sensor->present = 1U;
    fc_scan_stage = FC_SCAN_TYPE;
    note_fc_reply();
  } else if (command == 0x90U) {
    if ((length != 6U) ||
        ((fc_rx_frame[3] != 2U) && (fc_rx_frame[3] != 4U))) {
      return;
    }
    cache_frame(sensor, command, fc_rx_frame, length);
    sensor->sensor_length = fc_rx_frame[3];
    fc_scan_stage = FC_SCAN_VALUE;
    note_fc_reply();
  } else if ((command == 0xA0U) &&
             ((fc_scan_stage == FC_SCAN_VALUE) ||
              (fc_scan_stage == FC_REFRESH_VALUE))) {
    const uint8_t expected_length = (uint8_t)(sensor->sensor_length + 4U);
    if (length != expected_length) {
      return;
    }
    cache_frame(sensor, command, fc_rx_frame, length);
    note_fc_reply();
    if (fc_scan_stage == FC_SCAN_VALUE) {
      advance_fc_scan_after_value();
    } else {
      advance_fc_refresh();
    }
  }
}

static void handle_fc_timeout(void)
{
  (void)HAL_UART_AbortReceive(&huart_fc);
  __HAL_UART_CLEAR_PEFLAG(&huart_fc);
  fc_rx_index = 0U;
  fc_rx_expected_length = 0U;
  fc_response_ready = 0U;

  if (fc_scan_stage == FC_SCAN_DISCOVER) {
    finish_fc_scan((uint8_t)(fc_scan_sensor_id - 1U));
  } else if (fc_scan_stage == FC_SCAN_TYPE) {
    finish_fc_scan((uint8_t)(fc_scan_sensor_id - 1U));
  } else if (fc_scan_stage == FC_SCAN_VALUE) {
    advance_fc_scan_after_value();
  } else if (fc_scan_stage == FC_REFRESH_VALUE) {
    advance_fc_refresh();
  }
}

static void send_fc_query(uint32_t now)
{
  uint8_t query[IBUS_FRAME_REQUEST_LENGTH];
  fc_query_command = expected_fc_command();
  fc_query_sensor_id = fc_scan_sensor_id;
  query[0] = IBUS_FRAME_REQUEST_LENGTH;
  query[1] = (uint8_t)(fc_query_command | fc_query_sensor_id);
  const uint16_t checksum = ibus_checksum(query, 2U);
  query[2] = (uint8_t)(checksum & 0xFFU);
  query[3] = (uint8_t)(checksum >> 8);

  if (HAL_HalfDuplex_EnableTransmitter(&huart_fc) != HAL_OK) {
    fc_next_query_ms = now + IBUS_FC_QUERY_INTERVAL_MS;
    return;
  }
  if (HAL_UART_Transmit(&huart_fc, query, sizeof(query), 2U) != HAL_OK) {
    (void)HAL_HalfDuplex_EnableReceiver(&huart_fc);
    fc_next_query_ms = now + IBUS_FC_QUERY_INTERVAL_MS;
    return;
  }
  if (HAL_HalfDuplex_EnableReceiver(&huart_fc) != HAL_OK) {
    fc_next_query_ms = now + IBUS_FC_QUERY_INTERVAL_MS;
    return;
  }

  fc_rx_index = 0U;
  fc_rx_expected_length = 0U;
  fc_response_ready = 0U;
  fc_waiting = 1U;
  fc_query_started_ms = now;
  fc_next_query_ms = now + IBUS_FC_QUERY_INTERVAL_MS;
  (void)HAL_UART_Receive_IT(&huart_fc, &fc_rx_byte, 1U);
}

void ibus_bridge_init(void)
{
  huart_fc.Instance = USART1;
  huart_fc.Init.BaudRate = 115200U;
  huart_fc.Init.WordLength = UART_WORDLENGTH_8B;
  huart_fc.Init.StopBits = UART_STOPBITS_1;
  huart_fc.Init.Parity = UART_PARITY_NONE;
  huart_fc.Init.Mode = UART_MODE_TX_RX;
  huart_fc.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart_fc.Init.OverSampling = UART_OVERSAMPLING_16;
  if ((HAL_HalfDuplex_Init(&huart_fc) != HAL_OK) ||
      (HAL_HalfDuplex_EnableReceiver(&huart_fc) != HAL_OK)) {
    Error_Handler();
  }

  huart_receiver.Instance = USART2;
  huart_receiver.Init.BaudRate = 115200U;
  huart_receiver.Init.WordLength = UART_WORDLENGTH_8B;
  huart_receiver.Init.StopBits = UART_STOPBITS_1;
  huart_receiver.Init.Parity = UART_PARITY_NONE;
  huart_receiver.Init.Mode = UART_MODE_TX_RX;
  huart_receiver.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart_receiver.Init.OverSampling = UART_OVERSAMPLING_16;
  if ((HAL_HalfDuplex_Init(&huart_receiver) != HAL_OK) ||
      (HAL_HalfDuplex_EnableReceiver(&huart_receiver) != HAL_OK)) {
    Error_Handler();
  }

  (void)HAL_UART_Receive_IT(&huart_receiver, &receiver_rx_byte, 1U);
  fc_next_query_ms = HAL_GetTick() + 10U;
}

void ibus_bridge_service(void)
{
  const uint32_t now = HAL_GetTick();

  if (fc_response_ready != 0U) {
    process_fc_response();
    fc_waiting = 0U;
  } else if ((fc_waiting != 0U) &&
             ((uint32_t)(now - fc_query_started_ms) >= IBUS_FC_REPLY_TIMEOUT_MS)) {
    fc_waiting = 0U;
    handle_fc_timeout();
  }

  const uint32_t stall_check_ms = HAL_GetTick();
  if ((fc_valid_reply_seen != 0U) &&
      ((uint32_t)(stall_check_ms - last_fc_reply_ms) >= IBUS_FC_RECOVERY_TIMEOUT_MS) &&
      ((fc_recovery_started == 0U) ||
       ((uint32_t)(stall_check_ms - fc_last_recovery_ms) >= IBUS_FC_RECOVERY_RETRY_MS))) {
    fc_recovery_started = 1U;
    fc_last_recovery_ms = stall_check_ms;
    restart_fc_link(stall_check_ms);
  }

  if ((fc_waiting == 0U) && (fc_scan_stage != FC_SCAN_FINISHED) &&
      ((int32_t)(now - fc_next_query_ms) >= 0)) {
    send_fc_query(now);
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef *uart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();

  if (uart->Instance == USART1) {
    __HAL_RCC_USART1_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    HAL_NVIC_SetPriority(USART1_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  } else if (uart->Instance == USART2) {
    __HAL_RCC_USART2_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    HAL_NVIC_SetPriority(USART2_IRQn, 0U, 0U);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  } else {
    return;
  }

  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void ibus_bridge_usart1_irq(void)
{
  HAL_UART_IRQHandler(&huart_fc);
}

void ibus_bridge_usart2_irq(void)
{
  HAL_UART_IRQHandler(&huart_receiver);
}
