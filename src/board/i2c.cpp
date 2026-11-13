#include <Arduino.h>
#include "board/i2c.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "robot/types.h"
//I2C_SCL(IO19)     I2C_SDA(IO18)
#define I2C_SDA GPIO_NUM_18 //GPIO_NUM_18
#define I2C_SCL GPIO_NUM_19 //GPIO_NUM_19
#define I2C_FREQ_HZ 400000 //400000
#define I2C_PORT  I2C_NUM_0
constexpr uint32_t kI2cTimeoutMs = 2;

namespace {
// The driver stores every command node here. The largest request has a register
// write followed by a read; read length does not require one node per byte.
alignas(4) uint8_t commandStorage[I2C_LINK_RECOMMENDED_SIZE(2)];
StaticSemaphore_t mutexStorage;
SemaphoreHandle_t busMutex = nullptr;
portMUX_TYPE mutexInit = portMUX_INITIALIZER_UNLOCKED;

SemaphoreHandle_t getBusMutex() {
    portENTER_CRITICAL(&mutexInit);
    if (busMutex == nullptr) busMutex = xSemaphoreCreateMutexStatic(&mutexStorage);
    const SemaphoreHandle_t result = busMutex;
    portEXIT_CRITICAL(&mutexInit);
    return result;
}

class BusLock {
public:
    BusLock() : mutex_(getBusMutex()), locked_(mutex_ != nullptr &&
        xSemaphoreTake(mutex_, pdMS_TO_TICKS(kI2cTimeoutMs)) == pdTRUE) {}
    ~BusLock() { if (locked_) xSemaphoreGive(mutex_); }
    explicit operator bool() const { return locked_; }
    BusLock(const BusLock &) = delete;
    BusLock &operator=(const BusLock &) = delete;
private:
    SemaphoreHandle_t mutex_;
    bool locked_;
};

// Construct only while holding BusLock. RAII releases the command on every
// builder or driver error, before the owner releases the shared storage mutex.
class CommandLink {
public:
    CommandLink() : handle_(i2c_cmd_link_create_static(commandStorage, sizeof(commandStorage))) {}
    ~CommandLink() { if (handle_ != nullptr) i2c_cmd_link_delete_static(handle_); }
    explicit operator bool() const { return handle_ != nullptr; }
    bool start() { return i2c_master_start(handle_) == ESP_OK; }
    bool write(uint8_t value) { return i2c_master_write_byte(handle_, value, true) == ESP_OK; }
    bool read(uint8_t *buffer, uint32_t length, i2c_ack_type_t ack) {
        return i2c_master_read(handle_, buffer, length, ack) == ESP_OK;
    }
    bool readLast(uint8_t *buffer) {
        return i2c_master_read_byte(handle_, buffer, I2C_MASTER_NACK) == ESP_OK;
    }
    bool stop() { return i2c_master_stop(handle_) == ESP_OK; }
    bool execute() {
        return i2c_master_cmd_begin(I2C_PORT, handle_, pdMS_TO_TICKS(kI2cTimeoutMs)) == ESP_OK;
    }
    CommandLink(const CommandLink &) = delete;
    CommandLink &operator=(const CommandLink &) = delete;
private:
    i2c_cmd_handle_t handle_;
};
} // namespace

void I2C_Init(){
    BusLock lock;
    if (!lock) return;
    static i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA;
    conf.scl_io_num = I2C_SCL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_HZ;
    i2c_param_config(I2C_PORT,&conf);
    i2c_driver_install(I2C_PORT,conf.mode,0,0,0);
}

void Switch_I2C_To_Serial() {
    BusLock lock;
    if (!lock) return;
    i2c_driver_delete(I2C_PORT);
    gpio_reset_pin((gpio_num_t)18);
    gpio_reset_pin((gpio_num_t)19);
    Serial.begin(921600, SERIAL_8N1, 18, 19);
}

void I2C_WriteByte(uint8_t slaveAddr, uint8_t regAddr, uint8_t data){
    BusLock lock;
    if (!lock) return;
    CommandLink cmd;
    if (!cmd || !cmd.start() || !cmd.write(slaveAddr << 1) ||
        !cmd.write(regAddr) || !cmd.write(data) || !cmd.stop()) return;
    (void)cmd.execute(); // Preserve the existing void write API.
}

bool I2C_slave_read(uint8_t slave_addr, uint8_t data, uint8_t *buf, uint32_t len) {
    if (buf == nullptr || len == 0) return false;
    BusLock lock;
    if (!lock) return false;
    {
        CommandLink cmd;
        if (!cmd || !cmd.start() || !cmd.write(slave_addr << 1) ||
            !cmd.write(data) || !cmd.stop() || !cmd.execute()) return false;
    }
    // Preserve the legacy STOP between transactions and ACK for every byte.
    CommandLink cmd;
    return cmd && cmd.start() && cmd.write((slave_addr << 1) | I2C_MASTER_READ) &&
        cmd.read(buf, len, I2C_MASTER_ACK) && cmd.stop() && cmd.execute();
}

bool I2C_ReadIMU_Fast(uint8_t slave_addr, uint8_t reg_addr, uint8_t *buf, uint32_t len) {
    if (buf == nullptr || len == 0) return false;
    BusLock lock;
    if (!lock) return false;
    CommandLink cmd;
    if (!cmd || !cmd.start() || !cmd.write((slave_addr << 1) | I2C_MASTER_WRITE) ||
        !cmd.write(reg_addr) || !cmd.start() ||
        !cmd.write((slave_addr << 1) | I2C_MASTER_READ)) return false;
    if (len > 1 && !cmd.read(buf, len - 1, I2C_MASTER_ACK)) return false;
    // Repeated START and final NACK remain a single driver submission.
    return cmd.readLast(buf + len - 1) && cmd.stop() && cmd.execute();
}
