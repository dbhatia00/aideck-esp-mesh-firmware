// File: main/mesh_network.c

#include "mesh_network.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_mesh_internal.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include <string.h>
#include "cpx_receive.h"
#include "telemetry.h"

static const char *TAG = "Mesh_Network";

#define MESH_ID {0x7D, 0x0A, 0x2C, 0x9E, 0x33, 0x56} // Example Mesh ID

void mesh_event_handler(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case MESH_EVENT_STARTED:
        ESP_LOGI(TAG, "Mesh network started");
        break;
    case MESH_EVENT_PARENT_CONNECTED:
        ESP_LOGI(TAG, "Connected to parent node");
        break;
    default:
        ESP_LOGI(TAG, "Mesh event ID: %d", event_id);
        break;
    }
}

void mesh_recv_task(void *arg)
{
    mesh_addr_t from;
    mesh_data_t data;
    uint8_t rx_buf[1500];
    data.data = rx_buf;
    data.size = sizeof(rx_buf);

    while (1)
    {
        int flag = 0;
        int len = esp_mesh_recv(&from, &data, portMAX_DELAY, &flag, NULL, 0);
        if (len < 0)
        {
            ESP_LOGE(TAG, "Error receiving mesh data: %d", len);
            continue;
        }

        ESP_LOGI(TAG, "Received data from mesh, length: %d", len);

        if (len == sizeof(TelemetryData_t))
        {
            TelemetryData_t receivedData;
            memcpy(&receivedData, data.data, sizeof(TelemetryData_t));

            ESP_LOGI(TAG, "Received telemetry from mesh: Voltage=%.2fV, Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°",
                     receivedData.batteryVoltage,
                     receivedData.roll,
                     receivedData.pitch,
                     receivedData.yaw);

            // Send the telemetry data down to the Crazyflie via CPX
            sendTelemetryToCF(&receivedData);
        }
        else
        {
            ESP_LOGW(TAG, "Received data of unexpected length from mesh: %d", len);
        }
    }
}

void mesh_network_init(void)
{
    esp_err_t err;

    // Initialize TCP/IP
    ESP_ERROR_CHECK(esp_netif_init());

    // Initialize Wi-Fi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Set Wi-Fi to station mode
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // Initialize mesh network
    mesh_cfg_t mesh_cfg = MESH_INIT_CONFIG_DEFAULT();
    uint8_t mesh_id[6] = MESH_ID;
    memcpy(mesh_cfg.mesh_id.addr, mesh_id, 6);

    // Set mesh channel
    mesh_cfg.channel = 6; // Choose an appropriate channel

    // Configure the mesh to operate without connecting to an external router
    mesh_cfg.router.ssid_len = 0;
    memset(mesh_cfg.router.ssid, 0, sizeof(mesh_cfg.router.ssid));
    memset(mesh_cfg.router.password, 0, sizeof(mesh_cfg.router.password));

    // Optional: Set mesh network password and max connections
    //strcpy((char *)mesh_cfg.mesh_ap.password, "mesh_password");
    //mesh_cfg.mesh_ap.max_connection = 6; // Adjust as needed
    //mesh_cfg.mesh_ap.authmode = WIFI_AUTH_WPA2_PSK;

    // Initialize mesh
    ESP_ERROR_CHECK(esp_mesh_init());
    ESP_ERROR_CHECK(esp_mesh_set_config(&mesh_cfg));

    // Start the mesh network
    ESP_ERROR_CHECK(esp_mesh_start());

    // Register mesh event handler
    ESP_ERROR_CHECK(esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID, &mesh_event_handler, NULL));

    // Create a task to handle receiving data over the mesh network
    xTaskCreate(mesh_recv_task, "mesh_recv_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Mesh network initialized");
}

void mesh_send_data(const uint8_t *data, size_t len)
{
    mesh_data_t mesh_data;
    mesh_data.data = (uint8_t *)data;
    mesh_data.size = len;
    mesh_data.proto = MESH_PROTO_BIN;
    mesh_data.tos = MESH_TOS_P2P;

    mesh_addr_t dest_addr;
    memset(dest_addr.addr, 0, 6); // Broadcast address

    esp_err_t err = esp_mesh_send(&dest_addr, &mesh_data, 0, NULL, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error sending mesh data: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(TAG, "Telemetry data forwarded over mesh network, length: %d", (int)len);
    }
}
