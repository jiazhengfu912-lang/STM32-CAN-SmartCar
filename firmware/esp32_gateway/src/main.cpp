#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "../control_datagram.h"
#include <lwip/sockets.h>
#include <LittleFS.h>
#include <esp_http_server.h>
#include <esp_partition.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include "local_config.h"
extern "C" {
#include "car_protocol.h"
#include "../cache_store.h"
#include "../ota_sender.h"
}

/* 上传与安装分离；HTTP 安装期间独占 UART，USB 仅可在空闲时 PING/QUERY。 */
static HardwareSerial stm32(2);
static car_receiver usb_rx, uart_rx;
static uint32_t io_errors, blocked_updates;
static bool fs_ok;
static httpd_handle_t server;
static WiFiUDP control_udp;
static bool udp_ready;
static uint8_t image_scratch[BOOT_APP_CAPACITY];
static const char *PART = "/littlefs/car_m3.part";
static const char *READY = "/littlefs/car_m3.ready";
static uint32_t accepted_uploads, rejected_uploads;
static uint32_t install_requests;
static SemaphoreHandle_t uart_owner;
struct Ack { uint16_t sequence; uint8_t data[16]; };
static QueueHandle_t ack_queue;
static QueueHandle_t control_queue;
static uint8_t telemetry_data[40];
static uint32_t telemetry_seen;
static bool telemetry_valid;
static char last_install_id[33], last_install_json[512];
static uint8_t last_install_target[12];
static bool last_install_ok;
static portMUX_TYPE status_lock = portMUX_INITIALIZER_UNLOCKED;
static struct {
    char tag[5];
    uint32_t uptime, seen, parser_errors, io_errors, generation;
} latest_stm;

#ifdef CAR_POWER_CUT_TESTS
// 仅台架构建：暂停在两次操作之间，等待用户实际断电，不模拟掉电或复位。
static uint32_t power_cut_point, power_cut_bytes;
/** 仅测试构建在指定边界等待实际断电，超时不继续升级。 */
static void wait_for_power_cut(uint32_t point, uint32_t bytes)
{
    portENTER_CRITICAL(&status_lock);
    power_cut_point = point;
    power_cut_bytes = bytes;
    portEXIT_CRITICAL(&status_lock);
    uint32_t started = millis();
    while (uint32_t(millis() - started) < 600000U) delay(20);
    portENTER_CRITICAL(&status_lock);
    power_cut_point = 0;
    portEXIT_CRITICAL(&status_lock);
    // 十分钟未断电也不继续安装；调用方随后以失败结束。
}
#endif

/** 按小端顺序读取 32 位整数，避免未对齐访问。 */
static uint32_t get32(const uint8_t *p)
{
    return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
}

/** 将 32 位整数编码为小端字节序。 */
static void put32(uint8_t *p, uint32_t v)
{
    for (unsigned i = 0; i < 4; ++i) p[i] = uint8_t(v >> (8 * i));
}

/** 封装协议帧并发送到指定串口，发送失败计入错误统计。 */
static bool send_frame(HardwareSerial &port, uint8_t type, uint16_t seq,
                       const uint8_t *data, uint16_t size)
{
    uint8_t wire[CAR_MAX_FRAME];
    size_t length = car_encode(wire, sizeof(wire), type, seq, data, size);
    if (!length || port.write(wire, length) != length) {
        portENTER_CRITICAL(&status_lock);
        ++io_errors;
        portEXIT_CRITICAL(&status_lock);
        return false;
    }
    return true;
}

/** 仅允许空闲时 USB 诊断查询，拦截从 USB 发起的直接升级命令。 */
static void from_pc(void *, uint8_t type, uint16_t seq, const uint8_t *data, uint16_t size)
{
    if (type == CAR_MSG_PING || type == CAR_MSG_OTA_QUERY) {
        if (xSemaphoreTake(uart_owner, 0) == pdTRUE) {
            send_frame(stm32, type, seq, data, size);
            xSemaphoreGive(uart_owner);
        }
    } else if (type == CAR_MSG_ENTER_UPDATE || (type >= CAR_MSG_OTA_BEGIN && type <= CAR_MSG_OTA_RESET)) {
        portENTER_CRITICAL(&status_lock);
        ++blocked_updates;
        portEXIT_CRITICAL(&status_lock);
    }
}

/** 分发 STM32 身份、遥测和请求应答，并保护跨任务共享快照。 */
static void from_stm(void *, uint8_t type, uint16_t seq, const uint8_t *data, uint16_t size)
{
    if (type == CAR_MSG_DIAGNOSTIC && size == 16 &&
        (!memcmp(data, "SC01", 4) || !memcmp(data, "M2B2", 4))) {
        portENTER_CRITICAL(&status_lock);
        memcpy(latest_stm.tag, data, 4);
        latest_stm.uptime = get32(data + 4);
        latest_stm.seen = millis();
        latest_stm.parser_errors = get32(data + 8);
        latest_stm.io_errors = get32(data + 12);
        ++latest_stm.generation;
        portEXIT_CRITICAL(&status_lock);
    }
    if (type == CAR_MSG_TELEMETRY && size == 40) {
        portENTER_CRITICAL(&status_lock);
        memcpy(telemetry_data, data, 40); telemetry_seen = millis(); telemetry_valid = true;
        portEXIT_CRITICAL(&status_lock);
    }
    if (type == 0x71 && size == 16) {
        Ack ack; ack.sequence = seq; memcpy(ack.data, data, 16);
        xQueueSend(control_queue, &ack, 0);
    }
    if (type == CAR_MSG_OTA_REPLY && size == 16) {
        Ack ack;
        ack.sequence = seq;
        memcpy(ack.data, data, 16);
        xQueueSend(ack_queue, &ack, 0);
    }
    if (type == CAR_MSG_PONG || type == CAR_MSG_DIAGNOSTIC || type == CAR_MSG_OTA_REPLY)
        send_frame(Serial, type, seq, data, size);
}

/** 发送禁止缓存的 JSON HTTP 应答，允许客户端复用连接以减少心跳握手开销。 */
static esp_err_t respond(httpd_req_t *req, const char *status, const char *body)
{
    // 短响应分多次写入时禁用 Nagle 合包，避免等待对端延迟 ACK 拖慢心跳应答。
    int no_delay = 1;
    setsockopt(httpd_req_to_sockfd(req), IPPROTO_TCP, TCP_NODELAY, &no_delay, sizeof(no_delay));
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Connection", "keep-alive");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

/** 重新验证缓存后返回网关、缓存和 STM32 最近身份状态。 */
static esp_err_t status_handler(httpd_req_t *req)
{
    cache_info info;
    bool ready = fs_ok && cache_verify(READY, &info, image_scratch, sizeof(image_scratch));
    if (!ready) memset(&info, 0, sizeof(info));
    portENTER_CRITICAL(&status_lock);
    auto snapshot = latest_stm;
    uint32_t blocked = blocked_updates;
    portEXIT_CRITICAL(&status_lock);
    char body[768];
    snprintf(body, sizeof(body),
        "{\"gateway\":\"SG01\",\"control_transport\":\"udp-v1\",\"control_udp_port\":3232,\"uptime_ms\":%lu,\"filesystem\":%s,\"cache_ready\":%s,"
        "\"image_size\":%lu,\"image_crc32\":\"%08lX\",\"image_version\":%lu,"
        "\"accepted_uploads\":%lu,\"rejected_uploads\":%lu,\"install_enabled\":true,"
        "\"install_requests\":%lu,\"last_install_id\":\"%s\",\"last_install_ok\":%s,"
        "\"blocked_usb_updates\":%lu,\"stm32\":\"%s\",\"stm32_uptime_ms\":%lu,"
        "\"stm32_age_ms\":%lu,\"stm32_parser_errors\":%lu,\"stm32_io_errors\":%lu,"
        "\"http_stack_free_min\":%lu}",
        (unsigned long)millis(), fs_ok ? "true" : "false", ready ? "true" : "false",
        (unsigned long)info.size, (unsigned long)info.crc, (unsigned long)info.version,
        (unsigned long)accepted_uploads, (unsigned long)rejected_uploads,
        (unsigned long)install_requests, last_install_id, last_install_ok ? "true" : "false",
        (unsigned long)blocked, snapshot.tag, (unsigned long)snapshot.uptime,
        (unsigned long)(millis() - snapshot.seen), (unsigned long)snapshot.parser_errors,
        (unsigned long)snapshot.io_errors, (unsigned long)uxTaskGetStackHighWaterMark(nullptr));
    return respond(req, "200 OK", body);
}

/** 校验本机配置的 Bearer 令牌，空令牌不能授权写操作。 */
static bool authorized(httpd_req_t *req)
{
    char authorization[64], expected[48];
    snprintf(expected, sizeof(expected), "Bearer %s", WIFI_TOKEN);
    if (!WIFI_TOKEN[0] || httpd_req_get_hdr_value_str(req, "Authorization", authorization,
        sizeof(authorization)) != ESP_OK || strcmp(authorization, expected) != 0) {
        respond(req, "401 Unauthorized", "{\"error\":\"authorization_required\"}");
        return false;
    }
    return true;
}

/** 接收有大小和时间上限的上传，完整校验后才发布缓存。 */
static esp_err_t upload_handler(httpd_req_t *req)
{
    // 正常返回使 httpd 清理尚未读完的有限正文；立即关闭会令 TCP RST 吞掉 401。
    if (!authorized(req)) return req->content_len <= CACHE_MAX_SIZE ? ESP_OK : ESP_FAIL;
    if (!fs_ok) {
        respond(req, "503 Service Unavailable", "{\"error\":\"filesystem_unavailable\"}");
        return ESP_FAIL;
    }
    if (req->content_len < BOOT_META_SIZE + 258U || req->content_len > CACHE_MAX_SIZE) {
        ++rejected_uploads;
        respond(req, "413 Payload Too Large", "{\"error\":\"package_size\"}");
        return ESP_FAIL;
    }
#ifdef CAR_POWER_CUT_TESTS
    char fault[32];
    size_t pause_at = 0;
    if (httpd_req_get_hdr_value_str(req, "X-Test-Power-Cut", fault, sizeof(fault)) == ESP_OK) {
        if (strcmp(fault, "cache-partial"))
            return respond(req, "400 Bad Request", "{\"error\":\"unknown_power_cut_test\"}");
        pause_at = req->content_len * 6U / 10U;
    }
#endif
    cache_upload upload;
    cache_info info;
    if (!cache_start(&upload, PART, READY, req->content_len)) {
        ++rejected_uploads;
        respond(req, "500 Internal Server Error", "{\"error\":\"cache_open\"}");
        return ESP_FAIL;
    }
    uint32_t started = millis();
    while (upload.written < upload.expected) {
        uint8_t buffer[1024];
        size_t wanted = min(sizeof(buffer), upload.expected - upload.written);
#ifdef CAR_POWER_CUT_TESTS
        if (pause_at) wanted = min(wanted, pause_at - upload.written);
#endif
        int count = httpd_req_recv(req, reinterpret_cast<char *>(buffer), wanted);
        // 5 秒 socket 超时；整次上传另限 30 秒，防止慢速请求无限占用服务。
        if (count <= 0 || uint32_t(millis() - started) > 30000U) {
            cache_abort(&upload);
            ++rejected_uploads;
            respond(req, "408 Request Timeout", "{\"error\":\"upload_incomplete\"}");
            return ESP_FAIL;
        }
        if (!cache_write(&upload, buffer, size_t(count))) {
            cache_abort(&upload);
            ++rejected_uploads;
            respond(req, "500 Internal Server Error", "{\"error\":\"cache_write\"}");
            return ESP_FAIL;
        }
#ifdef CAR_POWER_CUT_TESTS
        if (pause_at && upload.written == pause_at) {
            wait_for_power_cut(1, upload.written);
            cache_abort(&upload);
            ++rejected_uploads;
            return respond(req, "408 Request Timeout", "{\"error\":\"power_cut_not_performed\"}");
        }
#endif
    }
    if (!cache_finish(&upload, &info, image_scratch, sizeof(image_scratch))) {
        ++rejected_uploads;
        return respond(req, "422 Unprocessable Entity", "{\"error\":\"package_verification\"}");
    }
    ++accepted_uploads;
    return status_handler(req);
}

struct ExchangeContext {
    uint8_t drop_mask, dropped_mask;
#ifdef CAR_POWER_CUT_TESTS
    uint8_t power_cut;
    uint32_t image_size;
#endif
};

/** 独占串口期间发送一次升级请求，等待匹配序号和类型的应答。 */
static int exchange(void *context, uint8_t type, uint16_t seq, const uint8_t *data,
                    uint16_t size, uint8_t reply[16], uint32_t timeout_ms)
{
    auto *test = static_cast<ExchangeContext *>(context);
#ifdef CAR_POWER_CUT_TESTS
    // 上一块/END 的 ACK 已由实际发送状态机验证，再到达下一次 exchange。
    if ((test->power_cut == 2 && type == CAR_MSG_OTA_DATA && size >= 8 && get32(data + 4) == 128) ||
        (test->power_cut == 3 && type == CAR_MSG_OTA_END) ||
        (test->power_cut == 4 && type == CAR_MSG_OTA_RESET)) {
        wait_for_power_cut(test->power_cut, test->power_cut == 2 ? 128 : test->image_size);
        return -1;
    }
#endif
    xQueueReset(ack_queue);
    if (!send_frame(stm32, type, seq, data, size)) return -1;
    uint32_t started = millis();
    while (uint32_t(millis() - started) < timeout_ms) {
        Ack ack;
        if (xQueueReceive(ack_queue, &ack, pdMS_TO_TICKS(10)) != pdTRUE) continue;
        if (ack.sequence != seq || ack.data[2] != type) continue;
        uint8_t bit = type == CAR_MSG_OTA_BEGIN ? 1 : type == CAR_MSG_OTA_DATA ? 2 :
                      type == CAR_MSG_OTA_END ? 4 : type == CAR_MSG_OTA_RESET ? 8 : 0;
        // 台架测试可丢弃一次匹配 ACK；仍等到真实超时才重发。
        if ((test->drop_mask & bit) && !(test->dropped_mask & bit)) {
            test->dropped_mask |= bit;
            continue;
        }
        memcpy(reply, ack.data, 16);
        return 1;
    }
    return 0;
}

/** 取得当前身份代数，供升级后识别新的启动消息。 */
static uint32_t mark_identity(void *)
{
    portENTER_CRITICAL(&status_lock);
    uint32_t generation = latest_stm.generation;
    portEXIT_CRITICAL(&status_lock);
    return generation;
}

/** 等待指定启动身份的新消息，超时或错误统计非零则不报告成功。 */
static int wait_identity(void *, const char tag[4], uint32_t after, uint32_t timeout_ms)
{
    uint32_t started = millis();
    do {
        portENTER_CRITICAL(&status_lock);
        auto snapshot = latest_stm;
        portEXIT_CRITICAL(&status_lock);
        if (snapshot.generation != after && !memcmp(snapshot.tag, tag, 4) &&
            uint32_t(millis() - snapshot.seen) < 2500U && !snapshot.parser_errors && !snapshot.io_errors)
            return 1;
        delay(5);
    } while (uint32_t(millis() - started) < timeout_ms);
    return 0;
}

/** 验证安装请求与缓存一致性，串行执行 OTA 并保存请求去重结果。 */
static esp_err_t install_handler(httpd_req_t *req)
{
    if (!authorized(req)) return req->content_len <= CACHE_MAX_SIZE ? ESP_OK : ESP_FAIL;
    char id[33];
    uint8_t wanted[12];
    if (req->content_len != sizeof(wanted) ||
        httpd_req_get_hdr_value_str(req, "X-Install-Id", id, sizeof(id)) != ESP_OK || strlen(id) != 32) {
        respond(req, "400 Bad Request", "{\"error\":\"install_request\"}");
        return ESP_FAIL;
    }
    for (char c : id) {
        if (c && !((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f')))
            return respond(req, "400 Bad Request", "{\"error\":\"install_id\"}");
    }
    size_t received = 0;
    while (received < sizeof(wanted)) {
        int count = httpd_req_recv(req, reinterpret_cast<char *>(wanted) + received, sizeof(wanted) - received);
        if (count <= 0) return ESP_FAIL;
        received += size_t(count);
    }
    // 同一请求重试返回已有结果，不再擦写。更换目标必须使用新请求 ID。
    if (!strcmp(id, last_install_id)) {
        if (memcmp(wanted, last_install_target, sizeof(wanted)))
            return respond(req, "409 Conflict", "{\"error\":\"install_id_conflict\"}");
        return respond(req, last_install_ok ? "200 OK" : "502 Bad Gateway", last_install_json);
    }
    cache_info info;
    if (!fs_ok || !cache_verify(READY, &info, image_scratch, sizeof(image_scratch)))
        return respond(req, "409 Conflict", "{\"error\":\"no_verified_cache\"}");
    if (get32(wanted) != info.size || get32(wanted + 4) != info.crc || get32(wanted + 8) != info.version)
        return respond(req, "409 Conflict", "{\"error\":\"cache_changed\"}");
    ExchangeContext context = {};
    char fault[24];
    if (httpd_req_get_hdr_value_str(req, "X-Test-Drop-Ack", fault, sizeof(fault)) == ESP_OK) {
        if (!strcmp(fault, "begin,data,end")) context.drop_mask = 7;
        else if (!strcmp(fault, "reset")) context.drop_mask = 8;
        else return respond(req, "400 Bad Request", "{\"error\":\"unknown_test_mode\"}");
    }
#ifdef CAR_POWER_CUT_TESTS
    char power_fault[32];
    if (httpd_req_get_hdr_value_str(req, "X-Test-Power-Cut", power_fault, sizeof(power_fault)) == ESP_OK) {
        if (!strcmp(power_fault, "stm32-first-block")) context.power_cut = 2;
        else if (!strcmp(power_fault, "stm32-before-commit")) context.power_cut = 3;
        else if (!strcmp(power_fault, "stm32-committed")) context.power_cut = 4;
        else return respond(req, "400 Bad Request", "{\"error\":\"unknown_power_cut_test\"}");
        context.image_size = info.size;
    }
#endif
    if (xSemaphoreTake(uart_owner, pdMS_TO_TICKS(100)) != pdTRUE)
        return respond(req, "409 Conflict", "{\"error\":\"uart_busy\"}");
    uint32_t session;
    do { session = esp_random(); } while (!session);
    sender_link link = {exchange, mark_identity, wait_identity, &context};
    sender_report report;
    ++install_requests;
    uint32_t started = millis();
    int result = sender_install(&link, &info, image_scratch, session, &report);
    xSemaphoreGive(uart_owner);
    memcpy(last_install_id, id, sizeof(id));
    memcpy(last_install_target, wanted, sizeof(wanted));
    last_install_ok = result == SEND_OK;
    snprintf(last_install_json, sizeof(last_install_json),
        "{\"result\":\"%s\",\"id\":\"%s\",\"error_code\":%d,\"stage\":%lu,"
        "\"session\":%lu,\"transferred\":%lu,\"blocks\":%lu,\"retries\":%lu,"
        "\"remote_status\":%lu,\"running_version\":%lu,\"elapsed_ms\":%lu,\"dropped_ack_mask\":%u}",
        last_install_ok ? "PASS" : "FAIL", id, result, (unsigned long)report.stage,
        (unsigned long)report.session, (unsigned long)report.transferred, (unsigned long)report.blocks,
        (unsigned long)report.retries, (unsigned long)report.remote_status,
        (unsigned long)report.running_version, (unsigned long)(millis() - started), context.dropped_mask);
    // 成败均保留完整缓存；重启只校验缓存，不自动重试安装。
    return respond(req, last_install_ok ? "200 OK" : "502 Bad Gateway", last_install_json);
}

/** 读取小端有符号遥测量，单位由协议文档约定。 */
static int signed16(const uint8_t *p) { return int16_t(uint16_t(p[0]) | uint16_t(p[1]) << 8); }

/** 提供最新运动快照和数据年龄；没有新反馈时显式标记为不可用。 */
static esp_err_t telemetry_handler(httpd_req_t *req)
{
    uint8_t p[40]; uint32_t age; bool valid;
    portENTER_CRITICAL(&status_lock);
    memcpy(p, telemetry_data, 40); age = millis() - telemetry_seen; valid = telemetry_valid;
    portEXIT_CRITICAL(&status_lock);
    char body[768];
    snprintf(body, sizeof(body),
        "{\"valid\":%s,\"age_ms\":%lu,\"uptime_ms\":%lu,\"version\":%lu,"
        "\"state\":%u,\"fault\":%u,\"feedback_mask\":%u,\"target_left\":%d,\"target_right\":%d,"
        "\"rpm_left\":%d,\"rpm_right\":%d,\"output_left\":%d,\"output_right\":%d,"
        "\"current_left\":%d,\"current_right\":%d,\"session\":%lu,\"sequence\":%lu,\"io_errors\":%lu}",
        valid && age < 500 ? "true" : "false", (unsigned long)age, (unsigned long)get32(p),
        (unsigned long)get32(p+4), p[8], p[9], p[10], signed16(p+12), signed16(p+14),
        signed16(p+16), signed16(p+18), signed16(p+20), signed16(p+22), signed16(p+24), signed16(p+26),
        (unsigned long)get32(p+28), (unsigned long)get32(p+32), (unsigned long)get32(p+36));
    return respond(req, "200 OK", body);
}

/** 将一次认证控制请求转发至 STM32，并等待对应应答；不自动重发或保持运动。 */
static esp_err_t control_handler(httpd_req_t *req)
{
    if (!authorized(req)) return req->content_len <= 64 ? ESP_OK : ESP_FAIL;
    if (req->content_len != 16) { respond(req,"400 Bad Request","{\"error\":\"control_size\"}"); return ESP_FAIL; }
    uint8_t p[16]; size_t n=0;
    while(n<sizeof(p)) {
        int count=httpd_req_recv(req,reinterpret_cast<char *>(p)+n,sizeof(p)-n);
        if(count<=0) return ESP_FAIL; n+=size_t(count);
    }
    if(p[0]>3 || p[1] || p[2] || p[3]) return respond(req,"400 Bad Request","{\"error\":\"control_argument\"}");
    if(xSemaphoreTake(uart_owner,0)!=pdTRUE) return respond(req,"409 Conflict","{\"error\":\"uart_busy\"}");
    static uint16_t sequence=40000;
    uint16_t expected=sequence=0x8000U|((sequence+1U)&0x7FFFU); xQueueReset(control_queue);
    bool sent=send_frame(stm32,0x40,expected,p,sizeof(p)), found=false; Ack ack;
    uint32_t started=millis();
    while(sent && millis()-started<200U) {
        if(xQueueReceive(control_queue,&ack,pdMS_TO_TICKS(5))==pdTRUE && ack.sequence==expected) { found=true; break; }
    }
    xSemaphoreGive(uart_owner);
    if(!found) return respond(req,"504 Gateway Timeout","{\"error\":\"no_control_ack\"}");
    char body[160];
    snprintf(body,sizeof(body),"{\"result\":%u,\"state\":%u,\"fault\":%u,\"session\":%lu,\"sequence\":%lu}",
             ack.data[0],ack.data[1],ack.data[2],(unsigned long)get32(ack.data+4),(unsigned long)get32(ack.data+8));
    return respond(req,ack.data[0] ? "409 Conflict":"200 OK",body);
}

/** 创建有界 HTTP 服务并注册只读状态、遥测、控制及升级接口。 */
static void start_server()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    // 实测默认 4096 字节在上传发布→状态读回→LittleFS 调用链触发栈保护。
    config.stack_size = 8192;
    config.max_open_sockets = 3;
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 5;
    config.send_wait_timeout = 5;
    if (httpd_start(&server, &config) != ESP_OK) { server = nullptr; return; }
    httpd_uri_t status_uri = {};
    status_uri.uri = "/status";
    status_uri.method = HTTP_GET;
    status_uri.handler = status_handler;
    httpd_uri_t cache_uri = {};
    cache_uri.uri = "/cache";
    cache_uri.method = HTTP_POST;
    cache_uri.handler = upload_handler;
    httpd_uri_t install_uri = {};
    install_uri.uri = "/install";
    install_uri.method = HTTP_POST;
    install_uri.handler = install_handler;
    httpd_uri_t control_uri = {};
    control_uri.uri="/control"; control_uri.method=HTTP_POST; control_uri.handler=control_handler;
    httpd_uri_t telemetry_uri = {};
    telemetry_uri.uri="/telemetry"; telemetry_uri.method=HTTP_GET; telemetry_uri.handler=telemetry_handler;
    if (httpd_register_uri_handler(server, &control_uri)!=ESP_OK ||
        httpd_register_uri_handler(server, &telemetry_uri)!=ESP_OK ||
        httpd_register_uri_handler(server, &status_uri) != ESP_OK ||
        httpd_register_uri_handler(server, &cache_uri) != ESP_OK ||
        httpd_register_uri_handler(server, &install_uri) != ESP_OK) {
        httpd_stop(server);
        server = nullptr;
    }
}

/** 初始化串口、队列、文件系统和 Wi-Fi，不自动格式化或安装缓存。 */
void setup()
{
    Serial.setRxBufferSize(512);
    stm32.setRxBufferSize(512);
    Serial.begin(115200);
    stm32.begin(115200, SERIAL_8N1, 16, 17);
    car_receiver_init(&usb_rx, from_pc, nullptr);
    car_receiver_init(&uart_rx, from_stm, nullptr);
    uart_owner = xSemaphoreCreateMutex();
    ack_queue = xQueueCreate(8, sizeof(Ack));
    control_queue = xQueueCreate(8, sizeof(Ack));
    if (!uart_owner || !ack_queue || !control_queue) abort();
    // 挂载失败不能默默格式化已有分区。格式化仅由已备份的本机安装步骤执行。
    fs_ok = LittleFS.begin(false);
    if (fs_ok) {
        // 断电遗留的 .part 永不晋升为 ready；完整 ready 在每次读取时重新验证。
        remove(PART);
    }
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    // 关闭 Wi-Fi 省电等待，避免控制请求延迟超过 STM32 的 300 ms 心跳期限。
    WiFi.setSleep(false);
    if (WIFI_SSID[0]) WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

/** 只转发本轮最新有效速度包；不缓存重传，UART 忙时丢弃，由下一新包或 STM32 超时决定。 */
static void poll_control_udp()
{
    if (!udp_ready) {
        if (WiFi.status()!=WL_CONNECTED) return;
        udp_ready=control_udp.begin(CONTROL_UDP_PORT)!=0;
        if (!udp_ready) return;
    }
    uint8_t packet[CONTROL_DATAGRAM_SIZE], latest[16], decoded[16];
    bool have=false;
    for (unsigned i=0;i<8;i++) {
        int length=control_udp.parsePacket();
        if (!length) break;
        int count=control_udp.read(packet,sizeof(packet));
        control_udp.flush();
        if (length!=CONTROL_DATAGRAM_SIZE || count!=CONTROL_DATAGRAM_SIZE ||
            !control_datagram_decode(packet,sizeof(packet),WIFI_TOKEN,decoded)) continue;
        // 同一会话内只保留最大序号；旧会话最终仍由 STM32 拒绝。
        if (have && get32(latest+4)==get32(decoded+4) && get32(decoded+8)<=get32(latest+8)) continue;
        memcpy(latest,decoded,sizeof(latest)); have=true;
    }
    if (have && xSemaphoreTake(uart_owner,0)==pdTRUE) {
        static uint16_t sequence=10000;
        sequence=(sequence+1U)&0x7FFFU;
        send_frame(stm32,0x40,sequence,latest,sizeof(latest));
        xSemaphoreGive(uart_owner);
    }
}

/** 持续处理两路串口、状态上报与网络服务启动，不自动发送运动指令。 */
void loop()
{
    static uint32_t last_status;
    static uint16_t seq;
    for (unsigned i = 0; i < CAR_MAX_FRAME && Serial.available(); ++i)
        car_receiver_feed(&usb_rx, uint8_t(Serial.read()), millis());
    for (unsigned i = 0; i < CAR_MAX_FRAME && stm32.available(); ++i)
        car_receiver_feed(&uart_rx, uint8_t(stm32.read()), millis());
    car_receiver_poll(&usb_rx, millis());
    car_receiver_poll(&uart_rx, millis());
    if (!server && WiFi.status() == WL_CONNECTED) start_server();
    poll_control_udp();
    if (uint32_t(millis() - last_status) >= 1000U) {
        last_status = millis();
        uint8_t data[16];
        memcpy(data, "SG01", 4);
        put32(data + 4, millis());
        put32(data + 8, usb_rx.crc_errors + usb_rx.header_errors + usb_rx.timeouts +
              uart_rx.crc_errors + uart_rx.header_errors + uart_rx.timeouts);
        portENTER_CRITICAL(&status_lock);
        uint32_t errors = io_errors;
        portEXIT_CRITICAL(&status_lock);
        put32(data + 12, errors);
        send_frame(Serial, CAR_MSG_DIAGNOSTIC, seq, data, sizeof(data));
#ifdef CAR_POWER_CUT_TESTS
        char network[224];
#else
        char network[128];
#endif
        snprintf(network, sizeof(network), "{\"ip\":\"%s\",\"wifi_status\":%d,\"filesystem\":%s,\"http\":%s}",
                 WiFi.localIP().toString().c_str(), int(WiFi.status()), fs_ok ? "true" : "false",
                 server ? "true" : "false");
#ifdef CAR_POWER_CUT_TESTS
        portENTER_CRITICAL(&status_lock);
        uint32_t point = power_cut_point, bytes = power_cut_bytes;
        portEXIT_CRITICAL(&status_lock);
        size_t end = strlen(network) - 1;
        snprintf(network + end, sizeof(network) - end,
                 ",\"power_cut_test\":true,\"power_cut_point\":%lu,\"power_cut_bytes\":%lu}",
                 (unsigned long)point, (unsigned long)bytes);
#endif
        // 0x12 只发给 USB 主机，不注入 STM32 协议链路。
        send_frame(Serial, 0x12, seq++, reinterpret_cast<const uint8_t *>(network), strlen(network));
    }
    delay(1);
}
