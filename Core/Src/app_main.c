#include "app_main.h"
#include "cmsis_os2.h"

#include "uart_at.h"
#include "uart_debug.h"
#include "flash_spi.h"
#include "task_cloud.h"
#include "task_ui.h"
#include "task_ota.h"
#include "task_mcu.h"
#include "config.h"

// 任务栈大小
#define TASK_STACK_SIZE     2048

// 任务优先级
#define CLOUD_TASK_PRIO     osPriority_t(osPriorityNormal + 5)
#define MCU_TASK_PRIO       osPriority_t(osPriorityNormal + 3)
#define UI_TASK_PRIO        osPriority_t(osPriorityNormal + 4)
#define OTA_TASK_PRIO       osPriority_t(osPriorityNormal + 2)

// 任务ID
osThreadId_t g_cloud_task_handle;
osThreadId_t g_mcu_task_handle;
osThreadId_t g_ui_task_handle;
osThreadId_t g_ota_task_handle;

// ==================== Cloud <-> OTA 同步 ====================
// Cloud_Task 初始化完成后设置此标志，OTA_Task 等待此标志后才开始运行
static osEventFlagsId_t g_cloud_ready_flag = NULL;
#define CLOUD_READY_FLAG_ID  (1U << 0)

osEventFlagsId_t task_sync_get_cloud_ready_flag(void) {
    return g_cloud_ready_flag;
}

void app_main_init(void) {
    // 初始化驱动层
    uart_debug_init();
    uart_at_init();
    flash_spi_init();

    // 创建同步事件标志（Cloud_Task设置，OTA_Task等待）
    osEventFlagsAttr_t flag_attr = { .name = "cloud_ready" };
    g_cloud_ready_flag = osEventFlagsNew(&flag_attr);

    // 注意: osKernelInitialize() 和 osKernelStart() 已在main()中调用
    // 此处只创建应用任务

    // Cloud Task - 云端通信
    osThreadAttr_t cloud_attr = {
        .name = "Cloud",
        .stack_size = TASK_STACK_SIZE,
        .priority = CLOUD_TASK_PRIO,
    };
    g_cloud_task_handle = osThreadNew(Cloud_Task, NULL, &cloud_attr);

    // MCU Task - 属性管理和云端消息处理
    osThreadAttr_t mcu_attr = {
        .name = "MCU",
        .stack_size = TASK_STACK_SIZE,
        .priority = MCU_TASK_PRIO,
    };
    g_mcu_task_handle = osThreadNew(MCU_Task, NULL, &mcu_attr);

    // UI Task - LVGL界面
    osThreadAttr_t ui_attr = {
        .name = "UI",
        .stack_size = TASK_STACK_SIZE,
        .priority = UI_TASK_PRIO,
    };
    g_ui_task_handle = osThreadNew(UI_Task, NULL, &ui_attr);

    // OTA Task - OTA管理
    osThreadAttr_t ota_attr = {
        .name = "OTA",
        .stack_size = TASK_STACK_SIZE,
        .priority = OTA_TASK_PRIO,
    };
    g_ota_task_handle = osThreadNew(OTA_Task, NULL, &ota_attr);
}
