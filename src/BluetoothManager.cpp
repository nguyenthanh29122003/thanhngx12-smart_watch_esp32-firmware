// src/BluetoothManager.cpp
#include "BluetoothManager.h"
#include "Config.h"
#include <string.h> // Cho strlen
#include <ArduinoJson.h> // <<< VẪN CẦN CHO PHẦN NHẬN WIFI/TIME CONFIG

// --- Server Callbacks (Giữ nguyên) ---
void MyServerCallbacks::onConnect(NimBLEServer* pServer) {
    Serial.println("BLE Device connected");
}
void MyServerCallbacks::onDisconnect(NimBLEServer* pServer) {
    Serial.println("BLE Device disconnected - Restarting advertising");
    vTaskDelay(pdMS_TO_TICKS(500));
    NimBLEDevice::startAdvertising();
}

// --- WifiConfigCallbacks (Sửa để gọi initWiFi và setStatus) ---
void WifiConfigCallbacks::onWrite(NimBLECharacteristic* pCharacteristic) {
    String value = pCharacteristic->getValue().c_str();
    Serial.println("[BLE] Received data on config char: " + value);

    StaticJsonDocument<256> doc; // Vẫn dùng ArduinoJson để parse data nhận vào
    DeserializationError error = deserializeJson(doc, value);
    if (error) {
        Serial.println("JSON parse failed: " + String(error.c_str()));
        manager->setStatus("{\"error\":\"JSON parse failed\"}");
        return;
    }

    if (doc.containsKey("ssid") && doc.containsKey("password")) {
        Serial.println("Received WiFi credentials via BLE.");
        manager->wifiSSID = doc["ssid"].as<String>();
        manager->wifiPassword = doc["password"].as<String>();
        manager->initWiFi(); // <-- Gọi hàm non-blocking
        manager->setStatus("{\"wifi_status\":\"connecting\"}");
    } else if (doc.containsKey("time")) {
        // --- Logic xử lý thời gian với mktime (Giữ nguyên) ---
        struct tm timeinfo = {};
        timeinfo.tm_year = doc["time"]["year"].as<int>() - 1900;
        timeinfo.tm_mon = doc["time"]["month"].as<int>() - 1;
        timeinfo.tm_mday = doc["time"]["day"].as<int>();
        timeinfo.tm_hour = doc["time"]["hour"].as<int>();
        timeinfo.tm_min = doc["time"]["minute"].as<int>();
        timeinfo.tm_sec = doc["time"]["second"].as<int>();
        timeinfo.tm_isdst = -1;
        time_t calculated_time = mktime(&timeinfo);
        if (calculated_time != (time_t)-1) {
            manager->timeManager->setTimeFromBLE(timeinfo);
            manager->setStatus("{\"time_status\":\"set_ok\"}");
        } else {
            Serial.println("Error: mktime failed from BLE time data.");
            manager->setStatus("{\"time_status\":\"set_error\"}");
        }
    } else {
        Serial.println("Unknown JSON format received via BLE config.");
        manager->setStatus("{\"error\":\"unknown_format\"}");
    }
}

// --- Constructor (Thêm khởi tạo biến mới) ---
BluetoothManager::BluetoothManager(TimeManager* timeManager)
    : pServer(nullptr), pDataCharacteristic(nullptr), pWifiConfigCharacteristic(nullptr),
      pStatusCharacteristic(nullptr), taskHandle(NULL), dataMutex(NULL),
      axLocal(0.0f), ayLocal(0.0f), azLocal(0.0f), gxLocal(0.0f), gyLocal(0.0f), gzLocal(0.0f),
      stepCountLocal(0), heartRateLocal(0), spo2Local(-999),
      irValueLocal(0), redValueLocal(0), wifiConnectedLocal(false),
      timestampLocal("Not initialized"),
      temperatureLocal(NAN), pressureLocal(NAN), // Khởi tạo temp/pres
      statusLocal("{\"status\":\"initializing\"}"),
      wifiSSID(""), wifiPassword(""), timeManager(timeManager)
{
    dataMutex = xSemaphoreCreateMutex();
    if (dataMutex == NULL) { Serial.println("CRITICAL: Failed to create BLE data mutex!"); }
}

// --- begin() ---
void BluetoothManager::begin() {
    Serial.println("Initializing Bluetooth Manager...");
    NimBLEDevice::init("ESP32-S3 Watch"); // Tên thiết bị
    setupBLE();
    initWiFi(); // Đăng ký event và thử kết nối non-blocking
    Serial.println("Bluetooth Manager Initialized.");
}

// --- setupBLE() - Thêm Status Characteristic ---
void BluetoothManager::setupBLE() {
    Serial.println("Setting up BLE Server...");
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    Serial.println("Creating BLE Service: " SERVICE_UUID);
    NimBLEService* pService = pServer->createService(SERVICE_UUID);

    // Data Characteristic (READ/NOTIFY)
    pDataCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY );
    pDataCharacteristic->createDescriptor("2901")->setValue("Sensor Data JSON");

    // WiFi Config Characteristic (WRITE)
    pWifiConfigCharacteristic = pService->createCharacteristic(
        WIFI_CONFIG_UUID,
        NIMBLE_PROPERTY::WRITE );
    pWifiConfigCharacteristic->setCallbacks(new WifiConfigCallbacks(this));
    pWifiConfigCharacteristic->createDescriptor("2901")->setValue("WiFi/Time Config JSON");

    // --- Status Characteristic (READ/NOTIFY) ---
    pStatusCharacteristic = pService->createCharacteristic(
        STATUS_UUID, // UUID từ Config.h
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY );
    pStatusCharacteristic->createDescriptor("2901")->setValue("Device Status JSON");
    pStatusCharacteristic->setValue(statusLocal); // Đặt giá trị ban đầu

    // Start Service & Advertising
    pService->start();
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->start();
    Serial.println("BLE Service Started and Advertising...");
}

// --- startTask() ---
void BluetoothManager::startTask(UBaseType_t priority) {
    xTaskCreate(taskFunction, "BluetoothTask", 4096, this, priority, &taskHandle);
    if (taskHandle == NULL) Serial.println("CRITICAL: Error creating Bluetooth Task!");
    else Serial.println("Bluetooth Task started.");
}

// --- stopTask() ---
void BluetoothManager::stopTask() {
    if (taskHandle != NULL) {
        TaskHandle_t tempHandle = taskHandle;
        taskHandle = NULL;
        vTaskDelete(tempHandle);
        Serial.println("Bluetooth task stopped.");
    }
}

// --- taskFunction() ---
void BluetoothManager::taskFunction(void* pvParameters) {
    BluetoothManager* instance = static_cast<BluetoothManager*>(pvParameters);
    if(instance == nullptr) { vTaskDelete(NULL); return; }

    Serial.println("Bluetooth Task running...");
    while (true) {
        // Gửi data chỉ khi có kết nối
        if (instance->pServer != nullptr && instance->pServer->getConnectedCount() > 0) {
            instance->sendData(); // Gọi hàm sendData (dùng String)
        }
        // Cập nhật trạng thái WiFi cục bộ dựa trên trạng thái thực tế
        bool currentWifiStatus = (WiFi.status() == WL_CONNECTED);
        if (xSemaphoreTake(instance->dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
             if (instance->wifiConnectedLocal != currentWifiStatus) {
                  instance->wifiConnectedLocal = currentWifiStatus;
                  // Không cần setStatus ở đây vì event handler đã làm
             }
             xSemaphoreGive(instance->dataMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Gửi mỗi giây
    }
}

// --- updateData() - Cập nhật thêm temp/pres ---
void BluetoothManager::updateData(float ax, float ay, float az, int stepCount, int heartRate, int spo2,
                                 long irValue, long redValue, bool wifiConnected, float gx, float gy, float gz,
                                 float temperature, float pressure, // <-- THÊM THAM SỐ MỚI
                                 const char* timestamp) {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        axLocal = ax; ayLocal = ay; azLocal = az;
        gxLocal = gx; gyLocal = gy; gzLocal = gz;
        stepCountLocal = stepCount;
        heartRateLocal = heartRate;
        spo2Local = spo2;
        irValueLocal = irValue; redValueLocal = redValue;
        // Không cập nhật wifiConnectedLocal ở đây nữa, để taskFunction tự cập nhật
        // wifiConnectedLocal = wifiConnected;
        temperatureLocal = temperature; // <-- LƯU DỮ LIỆU MỚI
        pressureLocal = pressure;       // <-- LƯU DỮ LIỆU MỚI
        timestampLocal = (timestamp != nullptr && strlen(timestamp) > 0) ? String(timestamp) : "Not initialized";
        xSemaphoreGive(dataMutex);
    } else { Serial.println("CRITICAL: Failed to take BLE dataMutex in updateData!"); }
}

// ===== sendData() - BẮT BUỘC DÙNG GHÉP STRING =====
void BluetoothManager::sendData() {
    // Biến tạm để giữ giá trị khi mutex được giữ
    float _ax, _ay, _az, _gx, _gy, _gz, _temp;
    int _steps, _hr, _spo2;
    long _ir, _red;
    float _pres; // Dùng float cho áp suất
    bool _wifi;
    String _timestamp;

    // Lấy dữ liệu an toàn
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        _ax = axLocal; _ay = ayLocal; _az = azLocal;
        _gx = gxLocal; _gy = gyLocal; _gz = gzLocal;
        _steps = stepCountLocal; _hr = heartRateLocal; _spo2 = spo2Local;
        _ir = irValueLocal; _red = redValueLocal;
        _wifi = wifiConnectedLocal; // Lấy trạng thái đã được cập nhật trong task
        _temp = temperatureLocal;
        _pres = pressureLocal;
        _timestamp = timestampLocal;
        xSemaphoreGive(dataMutex); // Trả mutex sớm nhất có thể

        // --- Bắt đầu ghép chuỗi JSON ---
        String jsonStr = "{";
        char buffer[15]; // Buffer cho dtostrf

        // IMU Data
        dtostrf(_ax, 1, 2, buffer); jsonStr += "\"ax\":" + String(buffer); // Giảm width xuống 1 để tránh thừa khoảng trắng
        dtostrf(_ay, 1, 2, buffer); jsonStr += ", \"ay\":" + String(buffer);
        dtostrf(_az, 1, 2, buffer); jsonStr += ", \"az\":" + String(buffer);
        dtostrf(_gx, 1, 2, buffer); jsonStr += ", \"gx\":" + String(buffer);
        dtostrf(_gy, 1, 2, buffer); jsonStr += ", \"gy\":" + String(buffer);
        dtostrf(_gz, 1, 2, buffer); jsonStr += ", \"gz\":" + String(buffer);

        // Sensor Data (Int/Long)
        jsonStr += ", \"steps\":" + String(_steps);
        jsonStr += ", \"hr\":" + String(_hr);
        jsonStr += ", \"spo2\":" + String(_spo2);
        jsonStr += ", \"ir\":" + String(_ir);
        jsonStr += ", \"red\":" + String(_red);

        // WiFi Status (Bool)
        jsonStr += ", \"wifi\":" + String(_wifi ? "true" : "false"); // <<< Dùng "true"/"false" trực tiếp

        // Temperature (Float, xử lý NAN)
        jsonStr += ", \"temp\":"; // Thêm key trước
        if (!isnan(_temp)) {
            dtostrf(_temp, 1, 1, buffer); // 1 chữ số thập phân, width 1
            jsonStr += String(buffer);
        } else {
             jsonStr += "null"; // <<< Gửi giá trị null JSON
        }

        // Pressure (Float, xử lý NAN)
        jsonStr += ", \"pres\":"; // Thêm key trước
        if (!isnan(_pres)) {
             dtostrf(_pres, 1, 0, buffer); // 0 chữ số thập phân, width 1
             jsonStr += String(buffer);
        } else {
             jsonStr += "null"; // <<< Gửi giá trị null JSON
        }

        // Timestamp (String)
        jsonStr += ", \"timestamp\":\""; // Mở ngoặc kép cho chuỗi timestamp
        // <<< XỬ LÝ CHUỖI TIMESTAMP (QUAN TRỌNG) >>>
        // Cần đảm bảo _timestamp không chứa ký tự đặc biệt gây lỗi JSON (như dấu ")
        // Nếu _timestamp có thể là "Not initialized" hoặc tương tự, vẫn ổn
        if (_timestamp.length() > 0) {
            // TODO: Nếu cần, thêm bước escape các ký tự đặc biệt trong _timestamp
            // Ví dụ đơn giản (có thể chưa đủ): _timestamp.replace("\"", "\\\"");
            jsonStr += _timestamp;
        }
        jsonStr += "\""; // <<< ĐÓNG NGOẶC KÉP CHO TIMESTAMP >>>

        jsonStr += "}"; // Kết thúc chuỗi JSON
        // --- Kết thúc ghép chuỗi JSON ---


        // Gửi qua BLE
        if (pDataCharacteristic != nullptr && pServer != nullptr && pServer->getConnectedCount() > 0) {
            try {
                // Serial.println("Sending JSON (String): " + jsonStr); // Debug
                pDataCharacteristic->setValue(jsonStr);
                pDataCharacteristic->notify();
            } catch (const std::exception& e) {
                Serial.printf("BLE Send Exception: %s\n", e.what());
            } catch (...) {
                Serial.println("Unknown BLE Send Exception!");
            }
        }
    } else {
        Serial.println("Mutex timeout in sendData! Skipping send.");
    }
}

// isWifiConnected() (Giữ nguyên)
bool BluetoothManager::isWifiConnected() {
    // Trả về trạng thái được cập nhật bởi task/event handler
    bool status;
     if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
         status = wifiConnectedLocal;
         xSemaphoreGive(dataMutex);
     } else {
          status = false; // Trạng thái không chắc chắn nếu mutex timeout
          Serial.println("Timeout taking mutex in isWifiConnected!");
     }
    return status;
    // Hoặc đơn giản hơn: return WiFi.status() == WL_CONNECTED; (Không cần mutex)
}


// ===== Hàm xử lý WiFi Events (NON-BLOCKING) =====
void BluetoothManager::wifiEventHandler(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.printf("[WiFi Event] event: %d\n", event);
    // !! QUAN TRỌNG: KHÔNG TRUY CẬP TRỰC TIẾP BIẾN THÀNH VIÊN this-> ở đây !!
    // Hàm static không có con trỏ 'this'.
    // Cần có cách truyền trạng thái về task chính hoặc dùng biến static/global (không khuyến khích)
    // -> Cách tốt nhất là Task BLE hoặc Task Loop sẽ kiểm tra WiFi.status() định kỳ.
    // -> Chúng ta chỉ dùng handler này để IN LOG và gửi STATUS BLE.

    // Tạo đối tượng tạm để gọi setStatus (CẨN THẬN VỚI CONCURRENCY)
    // --> CÁCH NÀY KHÔNG AN TOÀN, KHÔNG NÊN DÙNG TRONG ISR HOẶC EVENT HANDLER <--
    // BluetoothManager* instance = ble; // Giả sử ble là global (không nên)

    // --> CÁCH AN TOÀN HƠN: GỬI EVENT QUA QUEUE/EVENT GROUP TỚI TASK KHÁC <--

    // --> GIẢI PHÁP TẠM THỜI: Chỉ in log, Task khác tự kiểm tra status <--
    switch (event) {
        case SYSTEM_EVENT_STA_START:
            Serial.println("WiFi STA Started by Event");
            // Không gọi connect ở đây
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            Serial.print("WiFi Connected by Event! IP: ");
            Serial.println(WiFi.localIP());
            // Gửi status qua BLE (Tạm thời gọi trực tiếp, cần cơ chế an toàn hơn)
            // if (ble.pStatusCharacteristic) ble.setStatus("{\"wifi_status\":\"connected\"}"); // KHÔNG AN TOÀN
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            Serial.println("WiFi Disconnected by Event");
             // if (ble.pStatusCharacteristic) ble.setStatus("{\"wifi_status\":\"disconnected\"}"); // KHÔNG AN TOÀN
            // Xử lý kết nối lại có thể thực hiện trong task chính
            break;
        default:
            break;
    }
}

// --- Hàm initWiFi() - Đăng ký event và bắt đầu kết nối non-blocking ---
void BluetoothManager::initWiFi() {
    static bool wifiEventRegistered = false;
    if (!wifiEventRegistered) {
        Serial.println("Registering WiFi Event Handler...");
        // Đăng ký hàm static handler
        WiFi.onEvent(wifiEventHandler);
        wifiEventRegistered = true;
    }

    if (wifiSSID.length() > 0 && wifiPassword.length() > 0) {
        Serial.printf("Initiating WiFi connection to: %s (Non-Blocking)\n", wifiSSID.c_str());
        WiFi.mode(WIFI_STA);
        WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
        // Kết quả sẽ được xử lý bởi Event Handler (log) và task khác (kiểm tra status)
    } else {
         Serial.println("No WiFi credentials. Skipping connection.");
    }
}

// --- Hàm setStatus() ---
void BluetoothManager::setStatus(const String &status) {
    bool wasConnected = false;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        statusLocal = status;
        // Kiểm tra trạng thái kết nối BLE bên trong mutex
        wasConnected = (pServer != nullptr && pServer->getConnectedCount() > 0);
        xSemaphoreGive(dataMutex);

        // Gửi notify nếu đã kết nối và char hợp lệ
        if (wasConnected && pStatusCharacteristic != nullptr) {
             try {
                 Serial.println("BLE Sending Status: " + status); // Debug
                 pStatusCharacteristic->setValue(status);
                 pStatusCharacteristic->notify();
             } catch (...) {
                 Serial.println("BLE Status Send Exception!");
             }
        }
    } else {
        Serial.println("Timeout taking BLE dataMutex in setStatus!");
    }
}