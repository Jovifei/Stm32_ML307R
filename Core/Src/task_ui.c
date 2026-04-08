#include "task_ui.h"
// #include "lvgl/lvgl.h"  // LVGL暂未集成
#include "cmsis_os2.h"
#include <stdio.h>

// 注释掉LVGL相关代码，等待LCD驱动适配
#if 0
static lv_obj_t *label_status;
static lv_obj_t *label_signal;
static lv_obj_t *progress_bar;

void ui_create_main_screen(void) {
    lv_obj_t *scr = lv_scr_create();

    // 状态标签
    label_status = lv_label_create(scr);
    lv_label_set_text(label_status, "Connecting...");
    lv_obj_align(label_status, LV_ALIGN_TOP_MID, 0, 20);

    // 信号强度
    label_signal = lv_label_create(scr);
    lv_label_set_text(label_signal, "Signal: --");
    lv_obj_align(label_signal, LV_ALIGN_TOP_RIGHT, -10, 20);

    // 进度条 (OTA用)
    progress_bar = lv_bar_create(scr);
    lv_obj_set_width(progress_bar, 200);
    lv_obj_align(progress_bar, LV_ALIGN_BOTTOM_MID, 0, -40);
    lv_bar_set_value(progress_bar, 0, LV_ANIM_OFF);
}
#endif

void UI_Task(void *argument) {
    // LVGL暂未集成，等待LCD驱动适配
    // lv_init();
    // disp_drv.init();
    // lv_obj_t *scr = lv_scr_create();
    // ui_create_main_screen();

    while (1) {
        // lv_task_handler();
        osDelay(10);
    }
}

void ui_update_network_status(const char *status) {
    // LVGL暂未集成
    // lv_label_set_text(label_status, status);
}

void ui_update_signal(int rssi) {
    // LVGL暂未集成
}

void ui_update_ota_progress(int progress) {
    // LVGL暂未集成
    // lv_bar_set_value(progress_bar, progress, LV_ANIM_ON);
}
