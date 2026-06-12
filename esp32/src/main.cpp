#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_AMG88xx.h>

#include <TensorFlowLite_ESP32.h>
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "ECE140_WIFI.h"
#include "ECE140_MQTT.h"
#include "model_data.h"
#include "model_params.h"

Adafruit_AMG88xx amg;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];


constexpr int kTensorArenaSize = 8 * 1024;
alignas(16) uint8_t tensor_arena[kTensorArenaSize];

const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input_tensor = nullptr;
TfLiteTensor* output_tensor = nullptr;

float features[N_FEATURES];

const char* ucsdUsername = UCSD_USERNAME;
String ucsdPasswordStr = String(UCSD_PASSWORD); // + '#';
const char* ucsdPassword = ucsdPasswordStr.c_str();
const char* wifiSsid = WIFI_SSID;
const char* nonEnterpriseWifiPassword = NON_ENTERPRISE_WIFI_PASSWORD;
unsigned long lastPublish = 0;


const char* CLIENT_ID = MQTT_CLIENT_ID;
const char* TOPIC_PREFIX = MQTT_TOPIC;


volatile bool oneShotReq = false;
volatile bool autoReq =  false;

ECE140_WIFI wifi;
ECE140_MQTT mqtt(CLIENT_ID, TOPIC_PREFIX);

void setupModel() {
    model = tflite::GetModel(model_tflite);

    static tflite::AllOpsResolver resolver;
    static tflite::MicroErrorReporter micro_error_reporter;
    static tflite::MicroInterpreter static_interpreter(
        model, resolver, tensor_arena, kTensorArenaSize, &micro_error_reporter);
    interpreter = &static_interpreter;

    interpreter->AllocateTensors();
    input_tensor = interpreter->input(0);
    output_tensor = interpreter->output(0);

    Serial.printf("[TFLite] Input: %d dims, type=%d\n",
                  input_tensor->dims->data[1], input_tensor->type);
    Serial.printf("[TFLite] Arena used: %d bytes\n",
                  interpreter->arena_used_bytes());
}

int largestBlob(float grid[8][8], float threshold) {
    bool visited[8][8] = {};
    int largest = 0;
    int qr[64], qc[64];

    for (int r = 0; r < 8; r++) {
        for (int c = 0; c < 8; c++) {
            if (visited[r][c] || grid[r][c] <= threshold) continue;
            int size = 0;
            int head = 0, tail = 0;
            qr[tail] = r; qc[tail] = c; tail++;
            visited[r][c] = true;
            while (head < tail) {
                int cr = qr[head], cc = qc[head]; head++;
                size++;
                const int dr[] = {-1, 1, 0, 0};
                const int dc[] = {0, 0, -1, 1};
                for (int d = 0; d < 4; d++) {
                    int nr = cr + dr[d], nc = cc + dc[d];
                    if (nr >= 0 && nr < 8 && nc >= 0 && nc < 8
                        && !visited[nr][nc] && grid[nr][nc] > threshold) {
                        visited[nr][nc] = true;
                        qr[tail] = nr; qc[tail] = nc; tail++;
                    }
                }
            }
            if (size > largest) largest = size;
        }
    }
    return largest;
}

void computeFeatures(float* raw_pixels, float* out_features) {
    float grid[8][8];
    for (int i = 0; i < 64; i++) grid[i / 8][i % 8] = raw_pixels[i];

    float sorted[64];
    memcpy(sorted, raw_pixels, 64 * sizeof(float));
    for (int i = 1; i < 64; i++) {
        float key = sorted[i];
        int j = i - 1;
        while (j >= 0 && sorted[j] > key) { sorted[j + 1] = sorted[j]; j--; }
        sorted[j + 1] = key;
    }
    float median = (sorted[31] + sorted[32]) / 2.0f;
    float threshold = median + 3.0f;

    float sum_sq = 0.0f;
    float row_min = raw_pixels[0], row_max = raw_pixels[0];
    int count_above_3 = 0, count_above_5 = 0;
    int hot_count = 0;
    float hot_row_sum = 0.0f, hot_col_sum = 0.0f;

    for (int i = 0; i < 64; i++) {
        float diff = raw_pixels[i] - median;
        sum_sq += diff * diff;
        if (raw_pixels[i] < row_min) row_min = raw_pixels[i];
        if (raw_pixels[i] > row_max) row_max = raw_pixels[i];
        if (raw_pixels[i] > threshold) {
            count_above_3++;
            hot_row_sum += (float)(i / 8);
            hot_col_sum += (float)(i % 8);
            hot_count++;
        }
        if (raw_pixels[i] > median + 5.0f) count_above_5++;
    }
    float std_dev = sqrtf(sum_sq / 64.0f);
    if (std_dev < 0.1f) std_dev = 0.1f;

    for (int i = 0; i < 64; i++) {
        out_features[i] = (raw_pixels[i] - median) / std_dev;
    }

    out_features[64] = row_max;
    out_features[65] = row_max - row_min;
    out_features[66] = (float)count_above_3;
    out_features[67] = (float)count_above_5;

    float h_sum = 0.0f, v_sum = 0.0f;
    for (int r = 0; r < 8; r++) {
        for (int c = 0; c < 7; c++) h_sum += fabsf(grid[r][c+1] - grid[r][c]);
    }
    for (int r = 0; r < 7; r++) {
        for (int c = 0; c < 8; c++) v_sum += fabsf(grid[r+1][c] - grid[r][c]);
    }
    out_features[68] = (h_sum / 56.0f + v_sum / 56.0f) / 2.0f;

    out_features[69] = (float)largestBlob(grid, threshold);

    float q[4] = {0, 0, 0, 0};
    for (int r = 0; r < 4; r++) for (int c = 0; c < 4; c++) q[0] += grid[r][c];
    for (int r = 0; r < 4; r++) for (int c = 4; c < 8; c++) q[1] += grid[r][c];
    for (int r = 4; r < 8; r++) for (int c = 0; c < 4; c++) q[2] += grid[r][c];
    for (int r = 4; r < 8; r++) for (int c = 4; c < 8; c++) q[3] += grid[r][c];
    for (int i = 0; i < 4; i++) q[i] /= 16.0f;
    float q_mean = (q[0] + q[1] + q[2] + q[3]) / 4.0f;
    float q_var = 0.0f;
    for (int i = 0; i < 4; i++) q_var += (q[i] - q_mean) * (q[i] - q_mean);
    out_features[70] = q_var / 4.0f;

    float center_sum = 0.0f, outer_sum = 0.0f;
    int outer_count = 0;
    for (int r = 0; r < 8; r++) {
        for (int c = 0; c < 8; c++) {
            if (r >= 2 && r < 6 && c >= 2 && c < 6) {
                center_sum += grid[r][c];
            } else {
                outer_sum += grid[r][c];
                outer_count++;
            }
        }
    }
    out_features[71] = (center_sum / 16.0f) - (outer_sum / (float)outer_count);

    float row_maxes[8], col_maxes[8];
    for (int r = 0; r < 8; r++) {
        row_maxes[r] = grid[r][0];
        for (int c = 1; c < 8; c++) if (grid[r][c] > row_maxes[r]) row_maxes[r] = grid[r][c];
    }
    for (int c = 0; c < 8; c++) {
        col_maxes[c] = grid[0][c];
        for (int r = 1; r < 8; r++) if (grid[r][c] > col_maxes[c]) col_maxes[c] = grid[r][c];
    }
    float rm_mean = 0, cm_mean = 0;
    for (int i = 0; i < 8; i++) { rm_mean += row_maxes[i]; cm_mean += col_maxes[i]; }
    rm_mean /= 8.0f; cm_mean /= 8.0f;
    float rm_var = 0, cm_var = 0;
    for (int i = 0; i < 8; i++) {
        rm_var += (row_maxes[i] - rm_mean) * (row_maxes[i] - rm_mean);
        cm_var += (col_maxes[i] - cm_mean) * (col_maxes[i] - cm_mean);
    }
    out_features[72] = sqrtf(rm_var / 8.0f);
    out_features[73] = sqrtf(cm_var / 8.0f);

    if (hot_count > 0) {
        float cr = hot_row_sum / (float)hot_count;
        float cc = hot_col_sum / (float)hot_count;
        out_features[74] = sqrtf((cr - 3.5f) * (cr - 3.5f) + (cc - 3.5f) * (cc - 3.5f));
    } else {
        out_features[74] = 0.0f;
    }

    out_features[75] = (float)count_above_3 / 64.0f;

    for (int i = 0; i < N_FEATURES; i++) {
        out_features[i] = (out_features[i] - SCALER_MEAN[i]) / SCALER_SCALE[i];
    }
    out_features[74] = 0.0f;
    out_features[75] = 0.0f;
}

float runInference(float scaled_features[N_FEATURES]) {
    float input_scale = input_tensor->params.scale;
    int input_zero_point = input_tensor->params.zero_point;

    int8_t* input_data = input_tensor->data.int8;
    for (int i = 0; i < N_FEATURES; i++) {
        int val = (int)roundf(scaled_features[i] / input_scale) + input_zero_point;
        if (val < -128) val = -128;
        if (val > 127) val = 127;
        input_data[i] = (int8_t)val;
    }

    interpreter->Invoke();

    float output_scale = output_tensor->params.scale;
    int output_zero_point = output_tensor->params.zero_point;
    int8_t raw_output = output_tensor->data.int8[0];
    float confidence = (raw_output - output_zero_point) * output_scale;

    return confidence;
}

void readNpost(){
    amg.readPixels(pixels);
    float thermistor = amg.readThermistor();

    computeFeatures(pixels, features);
    float confidence = runInference(features);
    bool present = confidence > 0.5f;
    String macAdd = WiFi.macAddress();

    String message = "{\"prediction\":\"";
    message += present ? "present" : "empty";
    message += "\",\"confidence\":";
    message += String(confidence, 4);
    float maxTemp = pixels[0], minTemp = pixels[0];
    for (int i = 1; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
        if (pixels[i] > maxTemp) maxTemp = pixels[i];
        if (pixels[i] < minTemp) minTemp = pixels[i];
    }
    message += ",\"max_temp\":";
    message += String(maxTemp, 1);
    message += ",\"min_temp\":";
    message += String(minTemp, 1);
    message += ",\"mac_address\":\"";
    message += String(macAdd);
    message += "\",\"thermistor\":";
    message += String(thermistor, 2);
    message += ",\"pixels\":[";

    for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
        message += String(pixels[i], 2);
        if (i < AMG88xx_PIXEL_ARRAY_SIZE - 1) message += ",";
    }
    message += "]}";

    Serial.println(message); // Use for debugging if needed
    mqtt.publishMessage("scan", message);

    Serial.printf("[%s] conf=%.3f | min=%.1fC max=%.1fC\n",
                  present ? "PRESENT" : "EMPTY  ",
                  confidence, minTemp, maxTemp);
}





//unsigned long lastAutoSendMs = 0; using last Publish
const unsigned long AUTO_PERIOD = 500; //1 sec = 1000 ms so half a second

/**
 * @brief Callback function for MQTT messages
 */
void mqttCallback(char* topic, uint8_t* payload, unsigned int length) {
    String message = "";
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }

    Serial.print("[MQTT] Received: ");
    Serial.println(message);

    if (message == "request") {
    oneShotReq = true;
    Serial.println("[CMD] one-shot request");
    } 
    else if (message == "auto") {
        autoReq = true;
        Serial.println("[CMD] auto mode ON");
    } 
    else if (message == "stop") {
        autoReq = false;
        Serial.println("[CMD] auto mode OFF");
    } 
    else {
        Serial.println("[CMD] unknown command (expected: request/auto/stop)");
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("attempting setup wifi");
    if(strlen(nonEnterpriseWifiPassword)<2){
        wifi.connectToWPAEnterprise(wifiSsid, ucsdUsername, ucsdPassword);
        Serial.println("ucsd");
    } else {
        wifi.connectToWiFi(wifiSsid,nonEnterpriseWifiPassword);
        Serial.println("local");
    }
    delay(1000);

    // Connect to MQTT broker
    mqtt.connectToBroker();
    
    mqtt.setCallback(mqttCallback);  // Must be AFTER connectToBroker (it recreates client)

    // Subscribing to command
    mqtt.subscribeTopic("command");
    Serial.println("[MQTT] Subscribed to command topic");

    // Initialize AMG8833
    Wire.begin();
    if (!amg.begin()) {
        while (1) {
            Serial.println("{\"error\":\"AMG8833 not detected\"}");
            delay(1000);
        }
    }

    delay(100);  // Let sensor boot up
    Serial.println("[Setup] AMG8833 and MQTT ready!");

    setupModel();
    Serial.println("[OK] Model loaded, starting inference loop");
    delay(100);
    Serial.println("[OK] Awaiting for command");
}


void loop() {
    //Serial.println(WiFi.macAddress());
    //delay(5000);
    mqtt.loop();

    // respond back to requests without delays
    // One-shot request 
    ///*
    if (oneShotReq) {
        oneShotReq = false;
        readNpost();
    }

    // Auto mode every sec
    if (autoReq) {
        unsigned long now = millis();
        if (now - lastPublish >= AUTO_PERIOD) {
            lastPublish = now;
            readNpost();
        }
    }//*/
    /*
    readNpost();
    delay(5000);
    //*/
}