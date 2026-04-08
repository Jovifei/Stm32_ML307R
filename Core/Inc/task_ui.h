#ifndef TASK_UI_H
#define TASK_UI_H

#include "cmsis_os.h"

void UI_Task(void *argument);

// UI更新接口
void ui_update_network_status(const char *status);
void ui_update_signal(int rssi);
void ui_update_ota_progress(int progress);

#endif // TASK_UI_H
