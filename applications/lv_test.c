/*
 * Copyright 2025 NXP
 * NXP Proprietary. This software is owned or controlled by NXP and may only be used strictly in
 * accordance with the applicable license terms.
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "lvgl.h"
#include "touch_800x480.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define DBG_TAG "LVGL.app"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/* 外部函数声明 */
extern void lv_port_disp_init(void);
extern void lv_port_indev_init(void);

/* 串口通信函数声明 */
static int esp32_uart_init(void);
static rt_err_t esp32_uart_rx_callback(rt_device_t dev, rt_size_t size);
static void send_command_to_esp32(const char* command);
static void process_esp32_packet(const char* packet);
static void update_task_list_from_esp32(const char* response);
static void parse_comma_separated_tasks(const char* task_data);
static void uart_msg_process_thread_entry(void *parameter);
static int verify_checksum(const char* type, const char* data, int received_checksum);
static void extract_packet_field(const char* packet, const char* field_name, char* output, int max_len);

/* UI更新函数声明 */
static void update_task_display(void);
static void update_selected_index_display(void);

/* 事件处理函数声明 */
static void btn_up_event_handler(lv_event_t *e);
static void btn_down_event_handler(lv_event_t *e);
static void btn_finish_event_handler(lv_event_t *e);
static void btn_delete_event_handler(lv_event_t *e);
static void btn_get_event_handler(lv_event_t *e);

/* RT-Thread相关定义 */
static struct rt_thread lvgl_thread;
static rt_uint8_t lvgl_thread_stack[PKG_LVGL_THREAD_STACK_SIZE];

/* UART消息处理线程 */
static struct rt_thread uart_msg_thread;
static rt_uint8_t uart_msg_thread_stack[2048];

/* 消息队列定义 - 减小缓冲区 */
#define UART_MSG_MAX_SIZE 1024
#define UART_MSG_QUEUE_SIZE 4

/* 数据包协议定义 */
#define PKT_START "<PKT_START>"
#define PKT_END "<PKT_END>"
#define PKT_DELIMITER "|"
#define DATA_DELIMITER ","

typedef struct {
    char data[UART_MSG_MAX_SIZE];
    rt_size_t len;
} uart_msg_t;

static rt_mq_t uart_msg_queue = RT_NULL;

/* 互斥锁保护共享资源 */
static rt_mutex_t ui_mutex = RT_NULL;

/* 任务信息结构体 */
typedef struct {
    char title[128];        /* 任务标题 */
    char list_name[64];     /* 列表名称 */
    int list_num;           /* 列表编号 */
    int task_num;           /* 任务编号 */
    bool is_valid;          /* 是否有效 */
} task_info_t;

/* UI对象结构体 */
typedef struct {
    lv_obj_t *screen;
    lv_obj_t *task_list_cont;      /* 左侧任务列表容器 */
    lv_obj_t *task_label;          /* 任务显示标签 */
    lv_obj_t *control_panel;       /* 右侧控制面板 */
    lv_obj_t *btn_up;              /* 上键 */
    lv_obj_t *btn_down;            /* 下键 */
    lv_obj_t *index_label;         /* 索引显示标签 */
    lv_obj_t *btn_finish;          /* Finish按钮 */
    lv_obj_t *btn_delete;          /* Delete按钮 */
    lv_obj_t *btn_get;             /* Get按钮 */
} lv_ui;

/* 串口通信相关定义 - 减小缓冲区 */
#define ESP32_UART_NAME    "uart4"    /* 串口设备名称 */
#define ESP32_UART_BAUD    115200     /* 波特率 */
#define UART_RX_BUFFER_SIZE 2048      /* 减小接收缓冲区大小 */

/* 串口通信变量 */
static rt_device_t esp32_uart_dev = RT_NULL;
static char uart_rx_buffer[UART_RX_BUFFER_SIZE];
static rt_size_t uart_rx_index = 0;
static bool in_packet = false;  /* 是否在接收数据包 */

/* 任务管理变量 */
#define MAX_TASK_COUNT 20  /* 减少最大任务数量 */
static task_info_t task_info_array[MAX_TASK_COUNT];
static int current_task_count = 0;
static int selected_task_index = 1;  /* 当前选中的任务索引（从1开始） */

/* 全局UI对象 */
static lv_ui guider_ui;

/* 字体声明 */
LV_FONT_DECLARE(lv_font_montserratMedium_16)
LV_FONT_DECLARE(lv_font_montserratMedium_12)

/* ==================== 校验和验证函数 ==================== */

/* 计算校验和 */
static int calculate_checksum(const char* data)
{
    int sum = 0;
    while (*data) {
        sum += *data++;
    }
    return sum % 256;
}

/* 验证校验和 */
static int verify_checksum(const char* type, const char* data, int received_checksum)
{
    char combined[1024];
    rt_snprintf(combined, sizeof(combined), "%s%s", type, data);
    int calculated = calculate_checksum(combined);
    return (calculated == received_checksum);
}

/* ==================== 数据包解析函数 ==================== */

/* 从数据包中提取字段 */
static void extract_packet_field(const char* packet, const char* field_name, char* output, int max_len)
{
    char search_str[32];
    rt_snprintf(search_str, sizeof(search_str), "%s:", field_name);

    const char* start = strstr(packet, search_str);
    if (start == NULL) {
        output[0] = '\0';
        return;
    }

    start += rt_strlen(search_str);
    const char* end = strstr(start, PKT_DELIMITER);
    if (end == NULL) {
        end = strstr(start, PKT_END);
    }

    if (end == NULL) {
        output[0] = '\0';
        return;
    }

    int len = end - start;
    if (len >= max_len) len = max_len - 1;

    rt_strncpy(output, start, len);
    output[len] = '\0';
}

/* ==================== UART消息处理线程 ==================== */

/* UART消息处理线程入口函数 */
static void uart_msg_process_thread_entry(void *parameter)
{
    uart_msg_t msg;

    LOG_I("UART message processing thread started");

    while (1)
    {
        /* 从消息队列接收数据 */
        if (rt_mq_recv(uart_msg_queue, &msg, sizeof(uart_msg_t), RT_WAITING_FOREVER) == RT_EOK)
        {
            /* 获取UI互斥锁 */
            if (rt_mutex_take(ui_mutex, RT_WAITING_FOREVER) == RT_EOK)
            {
                /* 处理接收到的数据包 */
                LOG_D("Processing packet (len=%d)", msg.len);
                process_esp32_packet(msg.data);

                /* 释放UI互斥锁 */
                rt_mutex_release(ui_mutex);
            }
        }
    }
}

/* ==================== UI更新函数 ==================== */

/* 更新任务显示 */
static void update_task_display(void)
{
    if (guider_ui.task_label == NULL) return;

    char display_text[1536] = {0};  /* 减小显示缓冲区 */
    int pos = 0;

    /* 构建显示文本 */
    for (int i = 0; i < current_task_count && i < MAX_TASK_COUNT; i++)
    {
        if (task_info_array[i].is_valid)
        {
            pos += rt_snprintf(display_text + pos, sizeof(display_text) - pos,
                             "%d. %s [%s]\n",
                             i + 1,
                             task_info_array[i].title,
                             task_info_array[i].list_name);
        }
    }

    if (current_task_count == 0)
    {
        rt_strcpy(display_text, "No tasks available\nPress GET to load tasks");
    }

    /* 更新标签文本 */
    lv_label_set_text(guider_ui.task_label, display_text);

    LOG_I("Task display updated with %d tasks", current_task_count);
}

/* 更新选中索引显示 */
static void update_selected_index_display(void)
{
    if (guider_ui.index_label == NULL) return;

    char index_text[16];
    rt_snprintf(index_text, sizeof(index_text), "%d", selected_task_index);
    lv_label_set_text(guider_ui.index_label, index_text);
}

/* ==================== 任务列表解析函数 ==================== */

/* 解析逗号分隔的任务数据 */
static void parse_comma_separated_tasks(const char* task_data)
{
    if (task_data == RT_NULL || rt_strlen(task_data) == 0)
    {
        LOG_W("Empty task data received");
        current_task_count = 0;
        update_task_display();
        return;
    }

    LOG_I("Parsing comma-separated task data (length=%d)", rt_strlen(task_data));

    /* 清空现有任务 */
    for (int i = 0; i < MAX_TASK_COUNT; i++)
    {
        rt_memset(&task_info_array[i], 0, sizeof(task_info_t));
        task_info_array[i].is_valid = false;
    }
    current_task_count = 0;

    /* 特殊处理无任务的情况 */
    if (rt_strcmp(task_data, "NO_TASKS") == 0)
    {
        LOG_I("No tasks available");
        update_task_display();
        return;
    }

    /* 复制数据以便处理 */
    char *data_copy = rt_malloc(rt_strlen(task_data) + 1);
    if (data_copy == RT_NULL)
    {
        LOG_E("Failed to allocate memory for task data parsing");
        return;
    }
    rt_strcpy(data_copy, task_data);

    /* 用逗号分割数据 */
    char *token = strtok(data_copy, DATA_DELIMITER);
    int task_index = 0;
    char current_list_name[64] = {0};
    int current_list_num = 0;

    while (token != RT_NULL && task_index < MAX_TASK_COUNT)
    {
        /* 去除前后空白 */
        while (*token == ' ' || *token == '\t') token++;
        char *end = token + rt_strlen(token) - 1;
        while (end > token && (*end == ' ' || *end == '\t')) *end-- = '\0';

        LOG_D("Processing token: [%s]", token);

        /* 检查是否是列表名（格式: "1.ListName"） */
        if (token[0] >= '1' && token[0] <= '9' && token[1] == '.' &&
            strchr(token + 2, '.') == NULL)  /* 确保后面没有其他点号 */
        {
            current_list_num = token[0] - '0';
            rt_strncpy(current_list_name, token + 2, sizeof(current_list_name) - 1);
            LOG_D("Found list %d: %s", current_list_num, current_list_name);
        }
        /* 检查是否是任务（格式: "1.1.TaskTitle"） */
        else if (token[0] >= '1' && token[0] <= '9' && token[1] == '.')
        {
            char *second_dot = strchr(token + 2, '.');
            if (second_dot != NULL)
            {
                /* 解析列表编号和任务编号 */
                int list_num = token[0] - '0';
                int task_num = atoi(token + 2);
                char *task_title = second_dot + 1;

                /* 存储任务信息 */
                task_info_t *task_info = &task_info_array[task_index];
                rt_strncpy(task_info->title, task_title, sizeof(task_info->title) - 1);
                rt_strncpy(task_info->list_name, current_list_name, sizeof(task_info->list_name) - 1);
                task_info->list_num = list_num;
                task_info->task_num = task_num;
                task_info->is_valid = true;

                task_index++;
                current_task_count++;

                LOG_D("Parsed task %d.%d: %s", list_num, task_num, task_title);
            }
        }

        token = strtok(RT_NULL, DATA_DELIMITER);
    }

    rt_free(data_copy);

    /* 更新显示 */
    update_task_display();

    /* 调整选中索引 */
    if (selected_task_index > current_task_count && current_task_count > 0)
    {
        selected_task_index = current_task_count;
        update_selected_index_display();
    }
    else if (current_task_count == 0)
    {
        selected_task_index = 1;
        update_selected_index_display();
    }

    LOG_I("Task parsing completed. Total tasks: %d", current_task_count);
}

/* ==================== 数据包处理函数 ==================== */

/* 处理ESP32数据包 */
static void process_esp32_packet(const char* packet)
{
    LOG_I("Processing ESP32 packet");

    /* 验证包头 */
    if (strstr(packet, PKT_START) != packet)
    {
        LOG_E("Invalid packet start");
        return;
    }

    /* 验证包尾 */
    if (strstr(packet, PKT_END) == NULL)
    {
        LOG_E("Invalid packet end");
        return;
    }

    /* 提取字段 */
    char type[64], data[1024], checksum_str[16];  /* 减小数据缓冲区 */
    extract_packet_field(packet, "TYPE", type, sizeof(type));
    extract_packet_field(packet, "DATA", data, sizeof(data));
    extract_packet_field(packet, "CHECKSUM", checksum_str, sizeof(checksum_str));

    /* 验证校验和 */
    int checksum = atoi(checksum_str);
    if (!verify_checksum(type, data, checksum))
    {
        LOG_E("Checksum verification failed");
        return;
    }

    LOG_I("Packet type: %s", type);

    /* 根据类型处理 */
    if (rt_strcmp(type, "TASKS") == 0)
    {
        LOG_I("Received task list");
        parse_comma_separated_tasks(data);
    }
    else if (rt_strcmp(type, "RESULT") == 0)
    {
        LOG_I("Operation result: %s", data);
        /* 延时后可选择手动获取任务列表 */
        rt_thread_mdelay(500);
    }
    else if (rt_strcmp(type, "ERROR") == 0)
    {
        LOG_E("Error: %s", data);
    }
    else if (rt_strcmp(type, "STATUS") == 0)
    {
        LOG_I("Status: %s", data);
    }
    else if (rt_strcmp(type, "HELP") == 0)
    {
        LOG_I("Help: %s", data);
    }
    else if (rt_strcmp(type, "TEST") == 0)
    {
        LOG_I("Test response: %s", data);
    }
    else
    {
        LOG_W("Unknown packet type: %s", type);
    }
}

/* ==================== 事件处理函数 ==================== */

/* 上键事件处理 */
static void btn_up_event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *btn = lv_event_get_target(e);

    switch (code) {
    case LV_EVENT_PRESSED:
        /* 按下时变红 */
        lv_obj_set_style_bg_color(btn, lv_color_hex(0xFF0000), LV_PART_MAIN|LV_STATE_DEFAULT);
        break;

    case LV_EVENT_RELEASED:
        /* 松开时恢复颜色并执行操作 */
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);

        /* 获取UI互斥锁 */
        if (rt_mutex_take(ui_mutex, 100) == RT_EOK)
        {
            if (selected_task_index > 1)
            {
                selected_task_index--;
                update_selected_index_display();
            }
            rt_mutex_release(ui_mutex);
        }
        break;

    default:
        break;
    }
}

/* 下键事件处理 */
static void btn_down_event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *btn = lv_event_get_target(e);

    switch (code) {
    case LV_EVENT_PRESSED:
        /* 按下时变红 */
        lv_obj_set_style_bg_color(btn, lv_color_hex(0xFF0000), LV_PART_MAIN|LV_STATE_DEFAULT);
        break;

    case LV_EVENT_RELEASED:
        /* 松开时恢复颜色并执行操作 */
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);

        /* 获取UI互斥锁 */
        if (rt_mutex_take(ui_mutex, 100) == RT_EOK)
        {
            if (selected_task_index < current_task_count && current_task_count > 0)
            {
                selected_task_index++;
                update_selected_index_display();
            }
            rt_mutex_release(ui_mutex);
        }
        break;

    default:
        break;
    }
}

/* Finish按钮事件处理 */
static void btn_finish_event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *btn = lv_event_get_target(e);

    switch (code) {
    case LV_EVENT_PRESSED:
        /* 按下时变红 */
        lv_obj_set_style_bg_color(btn, lv_color_hex(0xFF0000), LV_PART_MAIN|LV_STATE_DEFAULT);
        break;

    case LV_EVENT_RELEASED:
        /* 松开时恢复颜色并执行操作 */
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x4CAF50), LV_PART_MAIN|LV_STATE_DEFAULT);

        /* 获取UI互斥锁 */
        if (rt_mutex_take(ui_mutex, 100) == RT_EOK)
        {
            if (selected_task_index > 0 && selected_task_index <= current_task_count)
            {
                task_info_t *task = &task_info_array[selected_task_index - 1];
                if (task->is_valid)
                {
                    char cmd[64];
                    rt_snprintf(cmd, sizeof(cmd), "finish %d.%d",
                              task->list_num, task->task_num);
                    send_command_to_esp32(cmd);
                    LOG_I("Finish task %d: %s", selected_task_index, cmd);
                }
            }
            rt_mutex_release(ui_mutex);
        }
        break;

    default:
        break;
    }
}

/* Delete按钮事件处理 */
static void btn_delete_event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *btn = lv_event_get_target(e);

    switch (code) {
    case LV_EVENT_PRESSED:
        /* 按下时变红 */
        lv_obj_set_style_bg_color(btn, lv_color_hex(0xFF0000), LV_PART_MAIN|LV_STATE_DEFAULT);
        break;

    case LV_EVENT_RELEASED:
        /* 松开时恢复颜色并执行操作 */
        lv_obj_set_style_bg_color(btn, lv_color_hex(0xF44336), LV_PART_MAIN|LV_STATE_DEFAULT);

        /* 获取UI互斥锁 */
        if (rt_mutex_take(ui_mutex, 100) == RT_EOK)
        {
            if (selected_task_index > 0 && selected_task_index <= current_task_count)
            {
                task_info_t *task = &task_info_array[selected_task_index - 1];
                if (task->is_valid)
                {
                    char cmd[64];
                    rt_snprintf(cmd, sizeof(cmd), "delete %d.%d",
                              task->list_num, task->task_num);
                    send_command_to_esp32(cmd);
                    LOG_I("Delete task %d: %s", selected_task_index, cmd);
                }
            }
            rt_mutex_release(ui_mutex);
        }
        break;

    default:
        break;
    }
}

/* GET按钮事件处理 */
static void btn_get_event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *btn = lv_event_get_target(e);

    switch (code) {
    case LV_EVENT_PRESSED:
        /* 按下时变红 */
        lv_obj_set_style_bg_color(btn, lv_color_hex(0xFF0000), LV_PART_MAIN|LV_STATE_DEFAULT);
        break;

    case LV_EVENT_RELEASED:
        /* 松开时恢复颜色并执行操作 */
        lv_obj_set_style_bg_color(btn, lv_color_hex(0xFF9800), LV_PART_MAIN|LV_STATE_DEFAULT);

        /* 发送get命令 */
        LOG_I("Manual GET button pressed");
        send_command_to_esp32("get");
        break;

    default:
        break;
    }
}

/* ==================== 串口通信函数 ==================== */

/* 初始化ESP32串口通信 */
static int esp32_uart_init(void)
{
    /* 查找串口设备 */
    esp32_uart_dev = rt_device_find(ESP32_UART_NAME);
    if (esp32_uart_dev == RT_NULL)
    {
        LOG_E("Cannot find ESP32 UART device: %s", ESP32_UART_NAME);
        return -1;
    }

    /* 配置串口参数 */
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = ESP32_UART_BAUD;
    config.data_bits = DATA_BITS_8;
    config.stop_bits = STOP_BITS_1;
    config.parity = PARITY_NONE;

    rt_device_control(esp32_uart_dev, RT_DEVICE_CTRL_CONFIG, &config);

    /* 打开串口设备 */
    if (rt_device_open(esp32_uart_dev, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX) != RT_EOK)
    {
        LOG_E("Failed to open ESP32 UART device");
        return -1;
    }

    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(esp32_uart_dev, esp32_uart_rx_callback);

    LOG_I("ESP32 UART initialized successfully");
    LOG_I("UART Device: %s, Baud: %d", ESP32_UART_NAME, ESP32_UART_BAUD);
    return 0;
}

/* 串口接收回调函数 - 处理数据包格式 */
static rt_err_t esp32_uart_rx_callback(rt_device_t dev, rt_size_t size)
{
    char ch;
    static uart_msg_t msg;

    while (rt_device_read(dev, -1, &ch, 1) == 1)
    {
        /* 检查缓冲区溢出 */
        if (uart_rx_index >= UART_RX_BUFFER_SIZE - 1)
        {
            LOG_W("UART buffer overflow, resetting");
            uart_rx_index = 0;
            in_packet = false;
            continue;
        }

        uart_rx_buffer[uart_rx_index++] = ch;
        uart_rx_buffer[uart_rx_index] = '\0';

        /* 检查包头 */
        if (!in_packet)
        {
            /* 查找包头 */
            char *pkt_start = strstr(uart_rx_buffer, PKT_START);
            if (pkt_start != NULL)
            {
                /* 找到包头，移动缓冲区内容 */
                int offset = pkt_start - uart_rx_buffer;
                if (offset > 0)
                {
                    memmove(uart_rx_buffer, pkt_start, uart_rx_index - offset + 1);
                    uart_rx_index -= offset;
                }
                in_packet = true;
                LOG_D("Packet start detected");
            }
            else
            {
                /* 如果缓冲区太大但没有包头，清空部分缓冲区 */
                if (uart_rx_index > 100)
                {
                    uart_rx_index = 0;
                }
            }
        }

        /* 检查包尾 */
        if (in_packet)
        {
            char *pkt_end = strstr(uart_rx_buffer, PKT_END);
            if (pkt_end != NULL)
            {
                /* 找到完整的数据包 */
                int pkt_len = pkt_end - uart_rx_buffer + rt_strlen(PKT_END);

                /* 复制数据包到消息 */
                rt_strncpy(msg.data, uart_rx_buffer, pkt_len);
                msg.data[pkt_len] = '\0';
                msg.len = pkt_len;

                /* 发送到消息队列 */
                rt_mq_send(uart_msg_queue, &msg, sizeof(uart_msg_t));
                LOG_D("Complete packet received (len=%d)", pkt_len);

                /* 移除已处理的数据包 */
                if (uart_rx_index > pkt_len)
                {
                    memmove(uart_rx_buffer, uart_rx_buffer + pkt_len, uart_rx_index - pkt_len);
                    uart_rx_index -= pkt_len;
                    uart_rx_buffer[uart_rx_index] = '\0';
                }
                else
                {
                    uart_rx_index = 0;
                }

                in_packet = false;
            }
        }
    }

    return RT_EOK;
}

/* 发送命令到ESP32 */
static void send_command_to_esp32(const char* command)
{
    if (esp32_uart_dev == RT_NULL)
    {
        LOG_E("ESP32 UART not initialized");
        return;
    }

    /* 发送命令 */
    rt_size_t cmd_len = rt_strlen(command);
    rt_size_t written = rt_device_write(esp32_uart_dev, 0, command, cmd_len);
    rt_device_write(esp32_uart_dev, 0, "\r\n", 2);

    LOG_I("Command sent to ESP32: %s (bytes written: %d/%d)", command, written, cmd_len);
}

/* ==================== UI创建函数 ==================== */

/* 创建UI界面 */
static void setup_scr_screen(lv_ui *ui)
{
    /* 创建主屏幕 */
    ui->screen = lv_obj_create(NULL);
    lv_obj_set_size(ui->screen, 800, 480);
    lv_obj_set_style_bg_color(ui->screen, lv_color_hex(0xf0f0f0), LV_PART_MAIN|LV_STATE_DEFAULT);

    /* 创建左侧任务列表容器 */
    ui->task_list_cont = lv_obj_create(ui->screen);
    lv_obj_set_pos(ui->task_list_cont, 10, 10);
    lv_obj_set_size(ui->task_list_cont, 550, 460);
    lv_obj_set_style_bg_color(ui->task_list_cont, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->task_list_cont, 2, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui->task_list_cont, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->task_list_cont, 5, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_all(ui->task_list_cont, 10, LV_PART_MAIN|LV_STATE_DEFAULT);

    /* 创建任务显示标签 */
    ui->task_label = lv_label_create(ui->task_list_cont);
    lv_label_set_text(ui->task_label, "No tasks loaded\nPress GET to load tasks");
    lv_obj_set_pos(ui->task_label, 0, 0);
    lv_obj_set_size(ui->task_label, 530, 440);
    lv_obj_set_style_text_font(ui->task_label, &lv_font_montserratMedium_12, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_label_set_long_mode(ui->task_label, LV_LABEL_LONG_WRAP);

    /* 创建右侧控制面板 */
    ui->control_panel = lv_obj_create(ui->screen);
    lv_obj_set_pos(ui->control_panel, 570, 10);
    lv_obj_set_size(ui->control_panel, 220, 460);
    lv_obj_set_style_bg_color(ui->control_panel, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->control_panel, 2, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui->control_panel, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->control_panel, 5, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_all(ui->control_panel, 10, LV_PART_MAIN|LV_STATE_DEFAULT);

    /* 创建标题 */
    lv_obj_t *title = lv_label_create(ui->control_panel);
    lv_label_set_text(title, "Task Control");
    lv_obj_set_pos(title, 50, 5);
    lv_obj_set_style_text_font(title, &lv_font_montserratMedium_16, LV_PART_MAIN|LV_STATE_DEFAULT);

    /* 创建GET按钮 */
    ui->btn_get = lv_btn_create(ui->control_panel);
    lv_obj_set_pos(ui->btn_get, 60, 35);
    lv_obj_set_size(ui->btn_get, 80, 40);
    lv_obj_set_style_bg_color(ui->btn_get, lv_color_hex(0xFF9800), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_t *get_label = lv_label_create(ui->btn_get);
    lv_label_set_text(get_label, "GET");
    lv_obj_center(get_label);
    lv_obj_set_style_text_color(get_label, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_add_event_cb(ui->btn_get, btn_get_event_handler, LV_EVENT_ALL, NULL);

    /* 创建上键 */
    ui->btn_up = lv_btn_create(ui->control_panel);
    lv_obj_set_pos(ui->btn_up, 60, 85);
    lv_obj_set_size(ui->btn_up, 80, 40);
    lv_obj_set_style_bg_color(ui->btn_up, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_t *up_label = lv_label_create(ui->btn_up);
    lv_label_set_text(up_label, "UP");
    lv_obj_center(up_label);
    lv_obj_set_style_text_color(up_label, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_add_event_cb(ui->btn_up, btn_up_event_handler, LV_EVENT_ALL, NULL);

    /* 创建索引显示框 */
    lv_obj_t *index_cont = lv_obj_create(ui->control_panel);
    lv_obj_set_pos(index_cont, 60, 135);
    lv_obj_set_size(index_cont, 80, 50);
    lv_obj_set_style_bg_color(index_cont, lv_color_hex(0xf0f0f0), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(index_cont, 2, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(index_cont, lv_color_hex(0x666666), LV_PART_MAIN|LV_STATE_DEFAULT);

    ui->index_label = lv_label_create(index_cont);
    lv_label_set_text(ui->index_label, "1");
    lv_obj_center(ui->index_label);
    lv_obj_set_style_text_font(ui->index_label, &lv_font_montserratMedium_16, LV_PART_MAIN|LV_STATE_DEFAULT);

    /* 创建下键 */
    ui->btn_down = lv_btn_create(ui->control_panel);
    lv_obj_set_pos(ui->btn_down, 60, 195);
    lv_obj_set_size(ui->btn_down, 80, 40);
    lv_obj_set_style_bg_color(ui->btn_down, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_t *down_label = lv_label_create(ui->btn_down);
    lv_label_set_text(down_label, "DOWN");
    lv_obj_center(down_label);
    lv_obj_set_style_text_color(down_label, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_add_event_cb(ui->btn_down, btn_down_event_handler, LV_EVENT_ALL, NULL);

    /* 创建Finish按钮 */
    ui->btn_finish = lv_btn_create(ui->control_panel);
    lv_obj_set_pos(ui->btn_finish, 30, 260);
    lv_obj_set_size(ui->btn_finish, 140, 50);
    lv_obj_set_style_bg_color(ui->btn_finish, lv_color_hex(0x4CAF50), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_t *finish_label = lv_label_create(ui->btn_finish);
    lv_label_set_text(finish_label, "FINISH");
    lv_obj_center(finish_label);
    lv_obj_set_style_text_color(finish_label, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(finish_label, &lv_font_montserratMedium_16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_add_event_cb(ui->btn_finish, btn_finish_event_handler, LV_EVENT_ALL, NULL);

    /* 创建Delete按钮 */
    ui->btn_delete = lv_btn_create(ui->control_panel);
    lv_obj_set_pos(ui->btn_delete, 30, 320);
    lv_obj_set_size(ui->btn_delete, 140, 50);
    lv_obj_set_style_bg_color(ui->btn_delete, lv_color_hex(0xF44336), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_t *delete_label = lv_label_create(ui->btn_delete);
    lv_label_set_text(delete_label, "DELETE");
    lv_obj_center(delete_label);
    lv_obj_set_style_text_color(delete_label, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(delete_label, &lv_font_montserratMedium_16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_add_event_cb(ui->btn_delete, btn_delete_event_handler, LV_EVENT_ALL, NULL);

    LOG_I("UI setup completed with GET button");
}

/* ==================== 主线程函数 ==================== */

/* LVGL线程入口函数 */
static void lvgl_thread_entry(void *parameter)
{
    /* 创建UI互斥锁 */
    ui_mutex = rt_mutex_create("ui_mutex", RT_IPC_FLAG_PRIO);
    if (ui_mutex == RT_NULL)
    {
        LOG_E("Failed to create UI mutex");
        return;
    }

    /* 创建UART消息队列 */
    uart_msg_queue = rt_mq_create("uart_mq", sizeof(uart_msg_t), UART_MSG_QUEUE_SIZE, RT_IPC_FLAG_FIFO);
    if (uart_msg_queue == RT_NULL)
    {
        LOG_E("Failed to create UART message queue");
        return;
    }

    /* 创建并启动UART消息处理线程 */
    rt_err_t err = rt_thread_init(&uart_msg_thread, "uart_msg",
                                  uart_msg_process_thread_entry,
                                  RT_NULL,
                                  &uart_msg_thread_stack[0],
                                  sizeof(uart_msg_thread_stack),
                                  PKG_LVGL_THREAD_PRIO + 1,
                                  10);
    if (err != RT_EOK)
    {
        LOG_E("Failed to create UART message processing thread");
        return;
    }
    rt_thread_startup(&uart_msg_thread);

    /* 初始化ESP32串口通信 */
    if (esp32_uart_init() != 0)
    {
        LOG_W("ESP32 UART communication failed");
    }

    /* 初始化任务信息数组 */
    for (int i = 0; i < MAX_TASK_COUNT; i++)
    {
        rt_memset(&task_info_array[i], 0, sizeof(task_info_t));
        task_info_array[i].is_valid = false;
    }

    /* 创建UI */
    setup_scr_screen(&guider_ui);
    lv_scr_load(guider_ui.screen);

    LOG_I("LVGL application started!");
    LOG_I("Using manual GET button for task loading");
    LOG_I("Reduced buffer sizes for memory optimization");

    /* LVGL主循环 */
    while (1)
    {
        /* 获取UI互斥锁 */
        if (rt_mutex_take(ui_mutex, 10) == RT_EOK)
        {
            Touch_Scan();
            lv_task_handler();
            rt_mutex_release(ui_mutex);
        }

        rt_thread_mdelay(LV_DISP_DEF_REFR_PERIOD);
    }
}

/* 初始化LVGL线程 */
static int lvgl_thread_init(void)
{
    rt_err_t err;

    LOG_I("Initializing LVGL thread...");

    err = rt_thread_init(&lvgl_thread, "LVGL",
                         lvgl_thread_entry,
                         RT_NULL,
                         &lvgl_thread_stack[0],
                         sizeof(lvgl_thread_stack),
                         PKG_LVGL_THREAD_PRIO,
                         10);
    if (err != RT_EOK)
    {
        LOG_E("Failed to create LVGL thread");
        return -1;
    }
    rt_thread_startup(&lvgl_thread);

    return 0;
}

INIT_APP_EXPORT(lvgl_thread_init);
