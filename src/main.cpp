/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * need CONFIG_COMPILER_CXX_EXCEPTIONS=y
 */

#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "nvs_flash.h"
#include "usb/cdc_acm_host.h"
#include "usb/usb_host.h"
#include "usb/vcp.hpp"
#include "usb/vcp_ch34x.hpp"
#include "usb/vcp_cp210x.hpp"
#include "usb/vcp_ftdi.hpp"

using namespace esp_usb;

// Change these values to match your needs
#define EXAMPLE_ESP_WIFI_SSID CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY 3
#define EXAMPLE_BAUDRATE (57600)
#define EXAMPLE_STOP_BITS (0)  // 0: 1 stopbit, 1: 1.5 stopbits, 2: 2 stopbits
#define EXAMPLE_PARITY (0)     // 0: None, 1: Odd, 2: Even, 3: Mark, 4: Space
#define EXAMPLE_DATA_BITS (8)

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

/* The event group allows multiple bits for each event, but we only care about
 * two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

namespace {

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

static const char *TAG = "VCP example";
static SemaphoreHandle_t device_disconnected_sem;

static int s_retry_num = 0;

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(TAG, "retry to connect to the AP");
    } else {
      xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
    ESP_LOGI(TAG, "connect to the AP fail");
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

void wifi_init_sta(void) {
  s_wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());

  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

  wifi_config_t wifi_config = {
      .sta =
          {
              .ssid = EXAMPLE_ESP_WIFI_SSID,
              .password = EXAMPLE_ESP_WIFI_PASS,
              /* Authmode threshold resets to WPA2 as default if password
               * matches WPA2 standards (pasword len => 8). If you want to
               * connect the device to deprecated WEP/WPA networks, Please set
               * the threshold value to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set
               * the password with length and format matching to
               * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
               */
              //.threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
              .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
              .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
          },
  };
  wifi_config.sta.threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD;
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "wifi_init_sta finished.");

  /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or
   * connection failed for the maximum number of re-tries (WIFI_FAIL_BIT). The
   * bits are set by event_handler() (see above) */
  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE, pdFALSE, portMAX_DELAY);

  /* xEventGroupWaitBits() returns the bits before the call returned, hence we
   * can test which event actually happened. */
  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", EXAMPLE_ESP_WIFI_SSID,
             EXAMPLE_ESP_WIFI_PASS);
  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
  } else {
    ESP_LOGE(TAG, "UNEXPECTED EVENT");
  }
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
      ESP_LOGE(TAG, "CDC-ACM error has occurred, err_no = %d",
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
}  // namespace

/**
 * @brief Main application
 *
 * This function shows how you can use Virtual COM Port drivers
 */
extern "C" void app_main(void) {
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