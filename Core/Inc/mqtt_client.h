#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

typedef enum {
    MQTT_STATE_DISCONNECTED = 0,
    MQTT_STATE_CONNECTING,
    MQTT_STATE_CONNECTED,
    MQTT_STATE_ERROR
} mqtt_state_t;

int mqtt_client_connect(void);
int mqtt_client_disconnect(void);
int mqtt_client_publish(const char *topic, const char *payload, int qos);
mqtt_state_t mqtt_client_get_state(void);

#endif // MQTT_CLIENT_H
