/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * need CONFIG_COMPILER_CXX_EXCEPTIONS=y
 */

#include <lwip/netdb.h>
#include <string.h>
#include <sys/param.h>

#define LGFX_M5ATOMS3

#include <LGFX_AUTODETECT.hpp>
#include <LovyanGFX.hpp>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "usb/cdc_acm_host.h"
#include "usb/usb_host.h"
#include "usb/vcp.hpp"
#include "usb/vcp_ch34x.hpp"
#include "usb/vcp_cp210x.hpp"
#include "usb/vcp_ftdi.hpp"

#define EXAMPLE_BAUDRATE (57600)
#define EXAMPLE_STOP_BITS (0)  // 0: 1 stopbit, 1: 1.5 stopbits, 2: 2 stopbits
#define EXAMPLE_PARITY (0)     // 0: None, 1: Odd, 2: Even, 3: Mark, 4: Space
#define EXAMPLE_DATA_BITS (8)
#define INVALID_SOCK (-1)
#define YIELD_TO_ALL_MS 50

using namespace esp_usb;

static const char *TAG = "cdc-forward";
static SemaphoreHandle_t device_disconnected_sem;
static std::string response =
    "HTTP/1.1 200 OK\r\nContent-Type: text/csv\r\n\r\n";
static LGFX lcd;

QueueHandle_t queue;

/**
 * @brief Utility to log socket errors
 *
 * @param[in] tag Logging tag
 * @param[in] sock Socket number
 * @param[in] err Socket errno
 * @param[in] message Message to print
 */
static void log_socket_error(const char *tag, const int sock, const int err,
                             const char *message) {
  ESP_LOGE(tag,
           "[sock=%d]: %s\n"
           "errorHandle=%d: %s",
           sock, message, err, strerror(err));
}

/**
 * @brief Sends the specified data to the socket. This function blocks until all
 * bytes got sent.
 *
 * @param[in] tag Logging tag
 * @param[in] sock Socket to write data
 * @param[in] data Data to be written
 * @param[in] len Length of the data
 * @return
 *          >0 : Size the written data
 *          -1 : Error occurred during socket write operation
 */
static int socket_send(const char *tag, const int sock, const char *data,
                       const size_t len) {
  int to_write = len;
  while (to_write > 0) {
    int written = send(sock, data + (len - to_write), to_write, 0);
    if (written < 0 && errno != EINPROGRESS && errno != EAGAIN &&
        errno != EWOULDBLOCK) {
      log_socket_error(tag, sock, errno, "Error occurred during sending");
      return -1;
    }
    to_write -= written;
  }
  return len;
}

/**
 * @brief Returns the string representation of client's address (accepted on
 * this server)
 */
static inline char *get_clients_address(struct sockaddr_storage *source_addr) {
  static char address_str[128];
  char *res = NULL;
  // Convert ip address to string
  if (source_addr->ss_family == PF_INET) {
    res = inet_ntoa_r(((struct sockaddr_in *)source_addr)->sin_addr,
                      address_str, sizeof(address_str) - 1);
  }
#ifdef CONFIG_LWIP_IPV6
  else if (source_addr->ss_family == PF_INET6) {
    res = inet6_ntoa_r(((struct sockaddr_in6 *)source_addr)->sin6_addr,
                       address_str, sizeof(address_str) - 1);
  }
#endif
  if (!res) {
    address_str[0] = '\0';  // Returns empty string if conversion didn't succeed
  }
  return address_str;
}

static void wifi_got_ip(void *arg, esp_event_base_t base, int32_t event_id,
                        void *data) {
  const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)data;
  lcd.setCursor(10, 20);
  lcd.printf("My IP: " IPSTR "\n", IP2STR(&event->ip_info.ip));
  lcd.printf("My GW: " IPSTR "\n", IP2STR(&event->ip_info.gw));
  lcd.printf("My NETMASK: " IPSTR "\n", IP2STR(&event->ip_info.netmask));
}

static void tcp_server_task(void *pvParameters) {
  static const char *TAG = "socket-server";

  vTaskDelay(5000 / portTICK_PERIOD_MS);

  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                             wifi_got_ip, NULL));

  /* This helper function configures Wi-Fi or Ethernet, as selected in
   * menuconfig. Read "Establishing Wi-Fi or Ethernet Connection" section in
   * examples/protocols/README.md for more information about this function.
   */
  ESP_ERROR_CHECK(example_connect());

  // SemaphoreHandle_t *server_ready = (SemaphoreHandle_t *)(pvParameters);
  struct addrinfo hints = {.ai_socktype = SOCK_STREAM};
  struct addrinfo *address_info;
  int listen_sock = INVALID_SOCK;
  int err;

  // Translating the hostname or a string representation of an IP to
  // address_info
  int res =
      getaddrinfo(CONFIG_EXAMPLE_TCP_SERVER_BIND_ADDRESS,
                  CONFIG_EXAMPLE_TCP_SERVER_BIND_PORT, &hints, &address_info);
  if (res != 0 || address_info == NULL) {
    ESP_LOGE(TAG,
             "couldn't get hostname for `%s` "
             "getaddrinfo() returns %d, addrinfo=%p",
             CONFIG_EXAMPLE_TCP_SERVER_BIND_ADDRESS, res, address_info);
    goto errorHandle;
  }

  // Creating a listener socket
  listen_sock = socket(address_info->ai_family, address_info->ai_socktype,
                       address_info->ai_protocol);

  if (listen_sock < 0) {
    log_socket_error(TAG, listen_sock, errno, "Unable to create socket");
    goto errorHandle;
  }
  ESP_LOGI(TAG, "Listener socket created");

  // Binding socket to the given address
  err = bind(listen_sock, address_info->ai_addr, address_info->ai_addrlen);
  if (err != 0) {
    log_socket_error(TAG, listen_sock, errno, "Socket unable to bind");
    goto errorHandle;
  }
  ESP_LOGI(TAG, "Socket bound on %s:%s", CONFIG_EXAMPLE_TCP_SERVER_BIND_ADDRESS,
           CONFIG_EXAMPLE_TCP_SERVER_BIND_PORT);

  // Set queue (backlog) of pending connections to one (can be more)
  err = listen(listen_sock, 1);
  if (err != 0) {
    log_socket_error(TAG, listen_sock, errno, "Error occurred during listen");
    goto errorHandle;
  }
  ESP_LOGI(TAG, "Socket listening");
  // xSemaphoreGive(*server_ready);

  // Main loop for accepting new connections and serving all connected clients
  while (1) {
    struct sockaddr_storage source_addr;  // Large enough for both IPv4 or IPv6
    socklen_t addr_len = sizeof(source_addr);

    int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
    if (sock < 0) {
      ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
      break;
    }

    // We have a new client connected -> print it's address
    ESP_LOGI(TAG, "Connection accepted from IP:%s",
             get_clients_address(&source_addr));
    socket_send(TAG, sock, response.c_str(), response.length());

    while (1) {
      uint8_t data;
      BaseType_t result =
          xQueueReceive(queue, &data, pdMS_TO_TICKS(YIELD_TO_ALL_MS));
      if (result != pdPASS) continue;
      // We serve all the connected clients in this loop
      int len = socket_send(TAG, sock, (const char *)(&data), sizeof(uint8_t));
      if (len < 0) {
        // Error occurred on write to this socket -> close it and mark
        // invalid
        ESP_LOGI(TAG, "socket_send() returned %d -> closing the socket", len);
        close(sock);
        break;
      }
    }
  }

errorHandle:
  if (listen_sock != INVALID_SOCK) {
    close(listen_sock);
  }

  free(address_info);
  vTaskDelete(NULL);
}

/**
 * @brief Data received callback
 *
 * Just pass received data to stdout
 *
 * @param[in] data     Pointer to received data
 * @param[in] data_len Length of received data in bytes
 * @param[in] arg      Argument we passed to the device open function
 * @return
 *   true:  We have processed the received data
 *   false: We expect more data
 */
static bool handle_rx(const uint8_t *data, size_t data_len, void *arg) {
  // printf("%.*s", data_len, data);
  for (size_t i = 0; i < data_len; i++) {
    if (data[i] != '`') {
      xQueueSend(queue, &(data[i]), (TickType_t)0);
      printf("%c", data[i]);
    }
  }
  return true;
}

/**
 * @brief Device event callback
 *
 * Apart from handling device disconnection it doesn't do anything useful
 *
 * @param[in] event    Device event type and data
 * @param[in] user_ctx Argument we passed to the device open function
 */
static void handle_event(const cdc_acm_host_dev_event_data_t *event,
                         void *user_ctx) {
  switch (event->type) {
    case CDC_ACM_HOST_ERROR:
      ESP_LOGE(TAG, "CDC-ACM errorHandle has occurred, err_no = %d",
               event->data.error);
      break;
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
      ESP_LOGI(TAG, "Device suddenly disconnected");
      xSemaphoreGive(device_disconnected_sem);
      break;
    case CDC_ACM_HOST_SERIAL_STATE:
      ESP_LOGI(TAG, "Serial state notif 0x%04X", event->data.serial_state.val);
      break;
    case CDC_ACM_HOST_NETWORK_CONNECTION:
    default:
      break;
  }
}

/**
 * @brief USB Host library handling task
 *
 * @param arg Unused
 */
static void usb_lib_task(void *arg) {
  while (1) {
    // Start handling system events
    uint32_t event_flags;
    usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
    if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
      ESP_ERROR_CHECK(usb_host_device_free_all());
    }
    if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
      ESP_LOGI(TAG, "USB: All devices freed");
      // Continue handling USB events to allow device reconnection
    }
  }
}

/**
 * @brief Main application
 *
 * This function shows how you can use Virtual COM Port drivers
 */
extern "C" void app_main(void) {
  lcd.init();
  queue = xQueueCreate(4096, sizeof(uint8_t));
  if (queue == NULL) {
    ESP_LOGE(TAG, "queue init failed.");
  }

  device_disconnected_sem = xSemaphoreCreateBinary();
  assert(device_disconnected_sem);

  // Install USB Host driver. Should only be called once in entire application
  ESP_LOGI(TAG, "Installing USB Host");
  const usb_host_config_t host_config = {
      .skip_phy_setup = false,
      .intr_flags = ESP_INTR_FLAG_LEVEL1,
  };
  ESP_ERROR_CHECK(usb_host_install(&host_config));

  // Create a task that will handle USB library events
  BaseType_t task_created =
      xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 10, NULL);
  assert(task_created == pdTRUE);

  ESP_LOGI(TAG, "Installing CDC-ACM driver");
  ESP_ERROR_CHECK(cdc_acm_host_install(NULL));

  // Register VCP drivers to VCP service
  VCP::register_driver<FT23x>();
  VCP::register_driver<CP210x>();
  VCP::register_driver<CH34x>();

  // SemaphoreHandle_t server_ready = xSemaphoreCreateBinary();
  // assert(server_ready);
  // xTaskCreate(tcp_server_task, "tcp_server", 4096, &server_ready, 5, NULL);
  // xSemaphoreTake(server_ready, portMAX_DELAY);
  // vSemaphoreDelete(server_ready);
  xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);

  // Do everything else in a loop, so we can demonstrate USB device
  // reconnections
  while (true) {
    const cdc_acm_host_device_config_t dev_config = {
        .connection_timeout_ms = 5000,  // 5 seconds, enough time to plug the
                                        // device in or experiment with timeout
        .out_buffer_size = 512,
        .in_buffer_size = 4096,
        .event_cb = handle_event,
        .data_cb = handle_rx,
        .user_arg = NULL,
    };

    // You don't need to know the device's VID and PID. Just plug in any device
    // and the VCP service will load correct (already registered) driver for the
    // device
    ESP_LOGI(TAG, "Opening any VCP device...");
    auto vcp = std::unique_ptr<CdcAcmDevice>(VCP::open(&dev_config));

    if (vcp == nullptr) {
      ESP_LOGI(TAG, "Failed to open VCP device");
      continue;
    }
    vTaskDelay(10);

    ESP_LOGI(TAG, "Setting up line coding");
    cdc_acm_line_coding_t line_coding = {
        .dwDTERate = EXAMPLE_BAUDRATE,
        .bCharFormat = EXAMPLE_STOP_BITS,
        .bParityType = EXAMPLE_PARITY,
        .bDataBits = EXAMPLE_DATA_BITS,
    };
    ESP_ERROR_CHECK(vcp->line_coding_set(&line_coding));

    /*
    Now the USB-to-UART converter is configured and receiving data.
    You can use standard CDC-ACM API to interact with it. E.g.

    ESP_ERROR_CHECK(vcp->set_control_line_state(false, true));
    ESP_ERROR_CHECK(vcp->tx_blocking((uint8_t *)"Test string", 12));
    */

    // Send some dummy data
    ESP_LOGI(TAG, "Sending data through CdcAcmDevice");
    uint8_t data[] = "test_string";
    ESP_ERROR_CHECK(vcp->tx_blocking(data, sizeof(data)));
    ESP_ERROR_CHECK(vcp->set_control_line_state(true, true));

    // We are done. Wait for device disconnection and start over
    ESP_LOGI(TAG, "Done. You can reconnect the VCP device to run again.");
    xSemaphoreTake(device_disconnected_sem, portMAX_DELAY);
  }
}