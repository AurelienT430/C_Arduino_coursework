#include <math.h>
#include <stdlib.h>

const int B = 4275000;
const int R0 = 100000;
const int pinTempSensor = A0;

const int sampleDuration = 60;

const float ACTIVE_RATE = 1.0;
const float IDLE_RATE = 0.2;
const float POWER_DOWN_RATE = 1.0 / 30.0;

const int ACTIVE_INTERVAL_MS = 1000;
const int IDLE_INTERVAL_MS = 5000;
const int POWER_DOWN_INTERVAL_MS = 30000;

enum PowerMode { ACTIVE, IDLE, POWER_DOWN };
// ----Set initial mode to ACTIVE ----
PowerMode mode = ACTIVE;

#if defined(ARDUINO_ARCH_AVR)
#define debug Serial
#elif defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_SAM)
#define debug SerialUSB
#else
#define debug Serial
#endif

float read_temperature() {
    int a = analogRead(pinTempSensor);
    float R = 1023.0 / a - 1.0;
    R = R0 * R;
    float temperature = 1.0 / (log(R / R0) / B + 1 / 298.15) - 273.15;
    return temperature;
}

void collect_temperature_data(float* data, int count, int interval_ms) {
    for (int i = 0; i < count; i++) {
        data[i] = read_temperature();
        delay(interval_ms);
    }
}


void apply_dft(float* input, float* real, float* imag, float* magnitude, float* frequency, int N, float fs) {
    for (int k = 0; k < N; k++) {
        *(real + k) = 0.0;
        *(imag + k) = 0.0;

        for (int n = 0; n < N; n++) {
            float angle = 2 * PI * k * n / N;
            *(real + k) += *(input + n) * cos(angle);
            *(imag + k) -= *(input + n) * sin(angle);
        }

        *(magnitude + k) = sqrt(*(real + k) * *(real + k) + *(imag + k) * *(imag + k));
        *(frequency + k) = k * fs / N;
    }
}

void send_data_to_pc(float* timeData, float* freqData, float* magnitudes, int N, float sampleRate) {
    Serial.println("Time (s),Temperature (C),Frequency (Hz),Magnitude");

    for (int i = 0; i < N; i++) {
        float time = i * (1.0 / sampleRate);
        Serial.print(time, 2);
        Serial.print(",");
        Serial.print(*(timeData + i), 2);
        Serial.print(",");
        Serial.print(*(freqData + i), 2);
        Serial.print(",");
        Serial.println(*(magnitudes + i), 2);
        delay(10);
    }

    Serial.println("---- Data transmission complete ----");
}

PowerMode decide_power_mode(float* magnitudes, float* frequencies, int N) {
    float weightedSum = 0.0;
    float totalMagnitude = 0.0;

    int dominantIndex = 1;  // Skip DC (index 0)
    for (int i = 1; i < N; i++) {
        weightedSum += magnitudes[i] * frequencies[i];
        totalMagnitude += magnitudes[i];

        if (magnitudes[i] > magnitudes[dominantIndex]) {
            dominantIndex = i;
        }
    }

    float averageFrequency = 0.0;
    if (totalMagnitude > 0.0) {
        averageFrequency = weightedSum / totalMagnitude;
    }

    float dominantFrequency = frequencies[dominantIndex];

    Serial.print("Average frequency: ");
    Serial.print(averageFrequency, 3);
    Serial.print(" Hz | ");

    Serial.print("Dominant frequency: ");
    Serial.print(dominantFrequency, 3);
    Serial.println(" Hz");

    // You can optionally use dominantFrequency instead of averageFrequency
    if (averageFrequency > 0.5) {
        return ACTIVE;
    } else if (averageFrequency > 0.1) {
        return IDLE;
    } else {
        return POWER_DOWN;
    }
}

const int maxTrendHistory = 10;
float variationHistory[maxTrendHistory] = {0};
int trendIndex = 0;
int idleCounter = 0;
float samplingRate = 1.0;  // Initial sampling rate
float variationThreshold = 1.0;  // Change this based on testing

const char* get_mode_name(PowerMode mode) {
    switch (mode) {
        case ACTIVE: return "ACTIVE";
        case IDLE: return "IDLE";
        case POWER_DOWN: return "POWER_DOWN";
        default: return "UNKNOWN";
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(1000);
    Serial.println("Starting temperature DFT monitor...");
}

void loop() {

    // ---- Calculate sampling parameters ----
    int count = sampleDuration * samplingRate;
    if (count <= 0) count = 1;  // Prevent invalid memory allocation or divide-by-zero

    int intervalMs = 60000 / count;  // Ensure total sampling duration = 60s

    // ---- Allocate memory ----
    float* temperatureData = (float*)malloc(count * sizeof(float));
    float* real = (float*)malloc(count * sizeof(float));
    float* imag = (float*)malloc(count * sizeof(float));
    float* magnitude = (float*)malloc(count * sizeof(float));
    float* frequency = (float*)malloc(count * sizeof(float));

    if (!temperatureData || !real || !imag || !magnitude || !frequency) {
        Serial.println("Memory allocation failed!");
        while (1);
    }

    // ---- Collect data ----
    Serial.println("Collecting data...");
    collect_temperature_data(temperatureData, count, intervalMs);

    // ---- Compute temperature variation ----
    float totalVariation = 0.0;
    for (int i = 1; i < count; i++) {
        totalVariation += fabs(temperatureData[i] - temperatureData[i - 1]);
    }

    variationHistory[trendIndex % maxTrendHistory] = totalVariation;
    trendIndex++;

    float sum = 0.0;
    for (int i = 0; i < min(trendIndex, maxTrendHistory); i++) {
        sum += variationHistory[i];
    }
    float avgVariation = sum / min(trendIndex, maxTrendHistory);
    
    // ---- Send data to PC ----
    send_data_to_pc(temperatureData, frequency, magnitude, count, samplingRate);

    // ---- Perform DFT ----
    apply_dft(temperatureData, real, imag, magnitude, frequency, count, samplingRate);

    // Adjust based on temperature variation
    if (avgVariation > variationThreshold) {
        mode = ACTIVE;
        idleCounter = 0;
    } else if (mode == IDLE) {
        idleCounter++;
        if (idleCounter >= 5) {
            mode = POWER_DOWN;
        }
    } else {
        idleCounter = 0;
    }

    Serial.print("System Mode: ");
    Serial.println(get_mode_name(mode));

    // ---- Update sampling rate based on mode for next loop ----
    switch (mode) {
        case ACTIVE:
            samplingRate = ACTIVE_RATE;
            break;
        case IDLE:
            samplingRate = IDLE_RATE;
            break;
        case POWER_DOWN:
            samplingRate = POWER_DOWN_RATE;
            break;
    }

    // ---- Decide power mode ----
    mode = decide_power_mode(magnitude, frequency, count);

    // ---- Clean up ----
    free(temperatureData);
    free(real);
    free(imag);
    free(magnitude);
    free(frequency);

    Serial.print("Average temperature variation: ");
    Serial.println(avgVariation, 3);

    Serial.println("---- Cycle complete. Restarting... ----");
    delay(1000);  // Optional delay between cycles
}
