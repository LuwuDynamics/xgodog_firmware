#include <Arduino.h>
#include <BLEDevice.h>
#include "board/ble.h"
#include "robot/state.h"
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "protocol/protocol.h"
#include "board/calibration.h"

static const uint16_t BLE_RX_BUFFER_SIZE = 512;
static uint8_t rxBLEBuffer[BLE_RX_BUFFER_SIZE];
static uint16_t rxWrite = 0;
static uint16_t rxRead = 0;
static uint16_t rxCount = 0;
static bool rxResetPending = false;
static portMUX_TYPE bleBufferMux = portMUX_INITIALIZER_UNLOCKED;
static StaticSemaphore_t bleTxMutexStorage;
static SemaphoreHandle_t bleTxMutex = nullptr;

static BLEServer *pServer = nullptr;
static BLECharacteristic *pTxCharacteristic = nullptr;
static volatile bool deviceConnected = false;
static volatile uint16_t peerMtu = 23;

static void ClearBleInput()
{
    portENTER_CRITICAL(&bleBufferMux);
    rxWrite = rxRead = rxCount = 0;
    rxResetPending = true;
    portEXIT_CRITICAL(&bleBufferMux);
}

// See the following for generating UUIDs: https://www.uuidgenerator.net/
#define SERVICE_UUID "0000fff0-0000-1000-8000-00805f9b34fb" // UART service UUID
#define CHARACTERISTIC_UUID_RX "0000fff2-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_TX "0000fff1-0000-1000-8000-00805f9b34fb"

class BleConnectionCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        peerMtu = 23;
        deviceConnected = true;
    };

    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
        peerMtu = 23;
        ClearBleInput();
        pServer->startAdvertising();
    }

    void onMtuChanged(BLEServer *, esp_ble_gatts_cb_param_t *param) override
    {
        peerMtu = param->mtu.mtu;
    }
};

class BleReceiveCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        uint8_t *rxData = pCharacteristic->getData();
        const size_t rxLength = pCharacteristic->getLength();
        if (rxData == NULL || rxLength == 0) return;
        portENTER_CRITICAL(&bleBufferMux);
        if (rxLength > BLE_RX_BUFFER_SIZE - rxCount) {
            // Drop the overflowing batch and reset on the consumer task. Never
            // overwrite bytes while the parser is reading an older frame.
            rxWrite = rxRead = rxCount = 0;
            rxResetPending = true;
        } else {
            for (size_t i = 0; i < rxLength; ++i) {
                rxBLEBuffer[rxWrite] = rxData[i];
                rxWrite = (rxWrite + 1) % BLE_RX_BUFFER_SIZE;
            }
            rxCount += (uint16_t)rxLength;
        }
        portEXIT_CRITICAL(&bleBufferMux);
    }
};

static void BLE_ProcessOnce(void)
{
    uint8_t chunk[64];
    for (uint16_t processed = 0; processed < BLE_RX_BUFFER_SIZE;) {
        portENTER_CRITICAL(&bleBufferMux);
        const bool reset = rxResetPending;
        rxResetPending = false;
        const uint16_t count = min(rxCount, (uint16_t)sizeof(chunk));
        for (uint16_t i = 0; i < count; ++i) {
            chunk[i] = rxBLEBuffer[rxRead];
            rxRead = (rxRead + 1) % BLE_RX_BUFFER_SIZE;
        }
        rxCount -= count;
        portEXIT_CRITICAL(&bleBufferMux);
        if (reset) Protocol_ResetPort(PORT_BLE);
        if (count == 0) break;
        Protocol_ProcessBytes(chunk, count, PORT_BLE);
        processed += count;
    }
}

void InitBLE()
{
    deviceConnected = false;
    bleTxMutex = xSemaphoreCreateMutexStatic(&bleTxMutexStorage);
    BLEDevice::init(BLEName.c_str());
    // MTU 默认一般是 23；这里显式请求更大的 MTU（如 128）
    // 注意：最终 MTU 仍取决于客户端是否同意协商。
    BLEDevice::setMTU(128);

    // 创建一个 BLE 服务
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new BleConnectionCallbacks()); //设置回调

    BLEService *pService = pServer->createService(SERVICE_UUID);

    // 创建一个 BLE 特征
    pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
    pTxCharacteristic->addDescriptor(new BLE2902());
    BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
    pRxCharacteristic->setCallbacks(new BleReceiveCallbacks()); //设置回调

    pService->start();                  // 开始服务
    pServer->getAdvertising()->start(); // 开始广播
}

void taskBLE(void *parameter)
{
    (void)parameter;
    BLENameRead();
    InitBLE();
    for (;;)
    {
        BLE_ProcessOnce();
        vTaskDelay(pdMS_TO_TICKS(3));
    }
}

void BLE_Send(uint8_t *dataAddr, uint8_t dataLen)
{
    if (pTxCharacteristic == NULL || !deviceConnected || dataAddr == NULL) return;
    xSemaphoreTake(bleTxMutex, portMAX_DELAY);
    if (deviceConnected && pServer != nullptr) {
        const uint16_t mtu = peerMtu;
        // A notification carries at most MTU - 3 bytes. Preserve the complete
        // protocol byte stream, including the 254-byte telemetry frame.
        if (mtu > 3) {
            for (uint16_t offset = 0; offset < dataLen && deviceConnected;) {
                const uint16_t count = min((uint16_t)(dataLen - offset), (uint16_t)(mtu - 3));
                pTxCharacteristic->setValue(dataAddr + offset, count);
                pTxCharacteristic->notify();
                offset += count;
            }
        }
    }
    xSemaphoreGive(bleTxMutex);
}


void BLENameWrite(uint8_t *data) {
    ESP.flashEraseSector(FLASH_ADDR_NAME / 4096);
    ESP.flashWrite(FLASH_ADDR_NAME, (uint32_t *)data, 32);
}

static void generateBleNameSuffix() {
    uint8_t name[32] = {0};
    name[0] = 4;
    vTaskDelay(pdMS_TO_TICKS(3));
    srand(millis() + analogRead(34));
    vTaskDelay(pdMS_TO_TICKS(rand() % 10));
    srand(millis() + analogRead(34));
    for (int i = 0; i < 4; i++) {
        name[1 + i] = rand() % 10 + 48;
    }
    BLENameWrite(name);
}

void BLENameRead() {
    uint8_t storedName[32];
    const int maxRetries = 3;
    int retries = 0;
    if(xgo_version == XGOMINI){
        if(hasWheel == 1){
            BLEName = "XGO-MW-";
        }else{
            BLEName = "XGO-M-";
        }
    }else if(xgo_version == XGOLITE){
        BLEName = "XGO-L-";
    }
    while (retries < maxRetries) {
        ESP.flashRead(FLASH_ADDR_NAME, (uint32_t *)storedName, 32);
        if (storedName[0] != 0 && storedName[0] <= FLASH_NAME_MAX_LEN) {
            for (int i = 0; i < storedName[0]; i++) {
                BLEName += (char)storedName[i + 1];
            }
            return;
        }
        generateBleNameSuffix();
        retries++;
    }
}
