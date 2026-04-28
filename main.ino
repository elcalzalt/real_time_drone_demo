/*
   ============================================================================
   AUTONOMOUS SOLAR PANEL CLEANING DRONE - REAL-TIME SYSTEM
   Company Context: Icarus Systems
   Product: Autonomous Water-Spraying Brush Drone for Solar Panel Maintenance

   System Overview:
   - Multi-rotor drone with onboard water tank
   - Ultrasonic dust/dirt sensor to detect contamination
   - Brush motor control for cleaning
   - Water pump solenoid valve for spray delivery
   - Real-time obstacle avoidance (LiDAR simulation)
   - Safety: Emergency stop and landing protocol

   Real-Time Requirements:
   - HARD RT: Motor control updates (CTRL_PERIOD_MS = 20ms)
   - HARD RT: Emergency stop response (ISR latency < 5ms)
   - HARD RT: Sensor fusion (SENSOR_PERIOD_MS = 50ms)
   - SOFT RT: Telemetry logging (TELEMETRY_PERIOD_MS = 1000ms)
   - SOFT RT: Web status update (HTTP_PERIOD_MS = 2000ms)
   ============================================================================
*/

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_timer.h>

/* ========== HARDWARE PIN DEFINITIONS ========== */
#define PIN_MOTOR_BRUSH       32   // Brush motor PWM (GPIO32)
#define PIN_PUMP_VALVE        33   // Water pump solenoid (GPIO33)
#define PIN_PROP_MOTOR_L      26   // Left propeller motor (GPIO26)
#define PIN_PROP_MOTOR_R      27   // Right propeller motor (GPIO27)
#define PIN_DUST_SENSOR       34   // Ultrasonic dust sensor ADC (GPIO34 - ADC pin)
#define PIN_LIDAR_SIM         35   // LiDAR obstacle detection (GPIO35 - ADC pin)
#define PIN_EMERGENCY_STOP    25   // Emergency stop button (GPIO25 - Pull-up)
#define PIN_LED_HEARTBEAT     13   // Status heartbeat LED (GPIO13)
#define PIN_LED_HARD_DEADLINE 12   // Hard deadline compliance indicator (GPIO12)
#define PIN_LED_SOFT_DEADLINE 14   // Soft deadline indicator (GPIO14)
#define PIN_BATTERY_SENSE     36   // Battery voltage sensing (GPIO36 - ADC pin)

/* ========== TIMING PARAMETERS (milliseconds & microseconds) ========== */
#define CTRL_PERIOD_MS        20    // Motor control: HARD DEADLINE - safety critical
#define SENSOR_PERIOD_MS      50    // Sensor fusion: HARD DEADLINE - safety critical
#define OBSTACLE_PERIOD_MS    30    // Obstacle avoidance: HARD DEADLINE
#define TELEMETRY_PERIOD_MS   1000  // Logging & comms: SOFT DEADLINE
#define STATUS_BLINK_MS       100   // Heartbeat LED: SOFT DEADLINE
#define COMM_PERIOD_MS        2000  // External HTTP/Serial: SOFT DEADLINE

/* ========== DEADLINE ENFORCEMENT ========== */
#define HARD_DEADLINE_MISS_TOLERANCE_US   2000  // 2ms miss = log but continue
#define SOFT_DEADLINE_MISS_TOLERANCE_US   5000  // 5ms miss = log warning

/* ========== SYSTEM STATE & INTER-TASK COMMUNICATION ========== */
typedef struct {
  uint16_t dust_level;        // Dust contamination (0-4095 from ADC)
  uint16_t obstacle_distance; // LiDAR distance simulation
  uint16_t battery_voltage;   // Battery in millivolts
  uint8_t motor_brush_pwm;    // Brush motor PWM (0-255)
  uint8_t pump_valve_state;   // Pump state (0=off, 1=on)
  uint8_t prop_thrust_L;      // Left propeller thrust (0-255)
  uint8_t prop_thrust_R;      // Right propeller thrust (0-255)
  uint32_t timestamp_us;      // Microsecond timestamp from esp_timer_get_time()
  uint8_t system_healthy;     // Overall health flag
} SystemState_t;

typedef struct {
  uint32_t task_id;           // Which task/ISR triggered
  uint64_t timestamp_us;      // Event timestamp
  uint8_t event_code;         // Event type
} Event_t;

/* ========== FREERTOS OBJECTS ========== */
TaskHandle_t xTaskMotorControl = NULL;
TaskHandle_t xTaskSensorFusion = NULL;
TaskHandle_t xTaskObstacleAvoid = NULL;
TaskHandle_t xTaskTelemetry = NULL;
TaskHandle_t xTaskHeartbeat = NULL;

QueueHandle_t xEventQueue = NULL;       // Inter-task event communication
QueueHandle_t xTelemetryQueue = NULL;   // Data for external logging

SemaphoreHandle_t xMutexSystemState = NULL;  // Protect SystemState_t
SemaphoreHandle_t xBinarySemEmergency = NULL; // Emergency stop signal
SemaphoreHandle_t xBinarySemMotorReady = NULL; // Motor initialization done

SystemState_t gSystemState = {0};       // Global shared system state
volatile uint64_t gTaskStartTime_us[5] = {0}; // For deadline monitoring
volatile uint32_t gDeadlineMisses = 0;  // Count hard deadline violations

/* ========== UART LOGGING ========== */
void log_msg(const char *prefix, const char *msg) {
  uint64_t now_us = esp_timer_get_time();
  Serial.printf("[%llu us] %s: %s\n", now_us, prefix, msg);
}

void log_deadline_check(uint8_t task_id, uint64_t deadline_us, uint8_t is_hard) {
  uint64_t elapsed = esp_timer_get_time() - gTaskStartTime_us[task_id];
  const char *deadline_type = is_hard ? "HARD" : "SOFT";
  if (elapsed > deadline_us) {
    Serial.printf("[DEADLINE MISS] Task %d (%s): elapsed=%llu us, deadline=%llu us, overage=%llu us\n",
                  task_id, deadline_type, elapsed, deadline_us, elapsed - deadline_us);
    if (is_hard) gDeadlineMisses++;
    digitalWrite(PIN_LED_HARD_DEADLINE, is_hard);
  }
}

/* ========== INTERRUPT SERVICE ROUTINE: Emergency Stop Button ========== */
/* HARD DEADLINE: ISR must respond within 5µs and signal main tasks within 50µs */
volatile uint8_t gEmergencyStopTriggered = 0;

void IRAM_ATTR ISR_EmergencyStop() {
  // ISR latency < 1µs on ESP32 (IRAM placement)
  gEmergencyStopTriggered = 1;

  // Immediately cut motor signals
  ledcWrite(PIN_MOTOR_BRUSH, 0);      // Kill brush motor
  ledcWrite(PIN_PUMP_VALVE, 0);       // Kill pump
  ledcWrite(PIN_PROP_MOTOR_L, 0);     // Kill left prop
  ledcWrite(PIN_PROP_MOTOR_R, 0);     // Kill right prop

  // Signal binary semaphore to wakeup motor task
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xBinarySemEmergency, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }

  // Blink LED to indicate emergency state
  digitalWrite(PIN_LED_HEARTBEAT, HIGH);
}

/* ========== TASK 1: MOTOR CONTROL (Priority 4 - HIGHEST) ========== */
/*
   Company Use-Case: Motor actuation for propellers and brush
   HARD DEADLINE: Must complete within 20ms (50Hz update rate)
   Reason: Drone stability requires consistent motor control refresh
   Missing deadline could cause oscillation, instability, or loss of altitude
*/
void vTaskMotorControl(void *pvParameters) {
  log_msg("MOTOR_CTRL", "Initializing motor control task");

  // Setup PWM for motor outputs
  ledcAttach(PIN_MOTOR_BRUSH, 1000, 8);    // pin, frequency, resolution
  ledcAttach(PIN_PUMP_VALVE, 1000, 8);
  ledcAttach(PIN_PROP_MOTOR_L, 1000, 8);
  ledcAttach(PIN_PROP_MOTOR_R, 1000, 8);

  xSemaphoreGive(xBinarySemMotorReady);

  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t ctrl_iteration = 0;

  for (;;) {
    gTaskStartTime_us[0] = esp_timer_get_time();

    // Check for emergency stop signal
    if (xSemaphoreTake(xBinarySemEmergency, 0) == pdTRUE) {
      log_msg("MOTOR_CTRL", "EMERGENCY STOP ACTIVATED - Landing sequence initiated");
      gSystemState.prop_thrust_L = 0;
      gSystemState.prop_thrust_R = 0;
      gSystemState.motor_brush_pwm = 0;
      gSystemState.pump_valve_state = 0;
      gSystemState.system_healthy = 0;
      gEmergencyStopTriggered = 0;
    }

    // Acquire mutex to safely read system state
    if (xSemaphoreTake(xMutexSystemState, pdMS_TO_TICKS(5)) == pdTRUE) {
      // Apply motor control based on current system state
      ledcWrite(0, gSystemState.motor_brush_pwm);     // Brush motor
      ledcWrite(1, gSystemState.pump_valve_state ? 255 : 0);  // Pump valve (on/off)
      ledcWrite(2, gSystemState.prop_thrust_L);       // Left propeller
      ledcWrite(3, gSystemState.prop_thrust_R);       // Right propeller

      xSemaphoreGive(xMutexSystemState);
    } else {
      log_msg("MOTOR_CTRL", "Mutex timeout - potential priority inversion!");
    }

    // HARD DEADLINE CHECK
    log_deadline_check(0, CTRL_PERIOD_MS * 1000, 1);  // 1 = HARD

    // Periodic telemetry
    if (ctrl_iteration++ % 5 == 0) {
      Serial.printf("MOTOR_CTRL: Brush=%d%%, Pump=%s, PropL=%d%%, PropR=%d%%\n",
                    gSystemState.motor_brush_pwm / 255 * 100,
                    gSystemState.pump_valve_state ? "ON" : "OFF",
                    gSystemState.prop_thrust_L / 255 * 100,
                    gSystemState.prop_thrust_R / 255 * 100);
    }

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CTRL_PERIOD_MS));
  }
}

/* ========== TASK 2: SENSOR FUSION (Priority 3) ========== */
/*
   Company Use-Case: Real-time dust/contamination detection & sensor processing
   HARD DEADLINE: Must complete within 50ms (20Hz)
   Reason: Dirty solar panels detected late = inefficient cleaning
   Safety: Inaccurate sensor data could cause drone to attempt cleaning already-clean panels
*/
void vTaskSensorFusion(void *pvParameters) {
  log_msg("SENSOR_FUSION", "Initializing sensor fusion task");

  // Setup ADC for dust sensor and LiDAR
  pinMode(PIN_DUST_SENSOR, INPUT);
  pinMode(PIN_LIDAR_SIM, INPUT);
  pinMode(PIN_BATTERY_SENSE, INPUT);

  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint16_t dust_readings[5] = {0};
  uint8_t read_idx = 0;

  for (;;) {
    gTaskStartTime_us[1] = esp_timer_get_time();

    // Read sensors (variable execution time)
    uint16_t dust_raw = analogRead(PIN_DUST_SENSOR);     // ~1-2ms
    uint16_t lidar_raw = analogRead(PIN_LIDAR_SIM);      // ~1-2ms
    uint16_t battery_raw = analogRead(PIN_BATTERY_SENSE); // ~1-2ms

    // Moving average filter for dust (5-sample window)
    dust_readings[read_idx++ % 5] = dust_raw;
    uint32_t dust_sum = 0;
    for (int i = 0; i < 5; i++) {
      dust_sum += dust_readings[i];
    }
    uint16_t dust_filtered = dust_sum / 5;

    // Update shared system state (acquire mutex)
    if (xSemaphoreTake(xMutexSystemState, pdMS_TO_TICKS(10)) == pdTRUE) {
      gSystemState.dust_level = dust_filtered;
      gSystemState.obstacle_distance = lidar_raw;
      gSystemState.battery_voltage = battery_raw * 4;  // Simulate millivolts
      gSystemState.timestamp_us = esp_timer_get_time();

      // Decision logic: If dust > 2000 (out of 4095) AND obstacle > 1000, start cleaning
      if (dust_filtered > 2000 && lidar_raw > 1000 && gSystemState.system_healthy) {
        gSystemState.motor_brush_pwm = 200;  // 78% brush speed
        gSystemState.pump_valve_state = 1;   // Activate water pump
        gSystemState.prop_thrust_L = 100;    // Steady hover
        gSystemState.prop_thrust_R = 100;
      } else if (dust_filtered < 500) {
        gSystemState.motor_brush_pwm = 0;
        gSystemState.pump_valve_state = 0;
        gSystemState.prop_thrust_L = 80;
        gSystemState.prop_thrust_R = 80;
      }

      xSemaphoreGive(xMutexSystemState);
    }

    // Send event to queue for logging
    Event_t event;
    event.task_id = 1;
    event.timestamp_us = esp_timer_get_time();
    event.event_code = (dust_filtered > 2000) ? 0x10 : 0x00;
    xQueueSend(xEventQueue, &event, 0);

    // HARD DEADLINE CHECK
    log_deadline_check(1, SENSOR_PERIOD_MS * 1000, 1);  // 1 = HARD

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
  }
}

/* ========== TASK 3: OBSTACLE AVOIDANCE (Priority 3) ========== */
/*
   Company Use-Case: Collision prevention during autonomous cleaning
   HARD DEADLINE: Must complete within 30ms
   Reason: Obstacle detected late = potential collision/crash
   Safety: HARD RT because drone crashes are catastrophic
*/
void vTaskObstacleAvoid(void *pvParameters) {
  log_msg("OBSTACLE_AVOID", "Initializing obstacle avoidance task");

  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t obstacle_warnings = 0;
  uint8_t in_recovery = 0;

  for (;;) {
    gTaskStartTime_us[2] = esp_timer_get_time();

    // Read obstacle proximity (simulated LiDAR)
    if (xSemaphoreTake(xMutexSystemState, pdMS_TO_TICKS(5)) == pdTRUE) {
      uint16_t obs_dist = gSystemState.obstacle_distance;

      if (obs_dist < 500 && !in_recovery) { 
        log_msg("OBSTACLE_AVOID", "COLLISION WARNING - Altering trajectory");
        gSystemState.prop_thrust_L = 200;
        gSystemState.prop_thrust_R = 50;
        gSystemState.system_healthy = 0;
        in_recovery = 1;  // Only log once
        obstacle_warnings++;
      } else if (obs_dist > 2000) {
        gSystemState.system_healthy = 1;
        in_recovery = 0;  // Can warn again when obstacle returns
      }

      xSemaphoreGive(xMutexSystemState);
    }

    // HARD DEADLINE CHECK
    log_deadline_check(2, OBSTACLE_PERIOD_MS * 1000, 1);

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(OBSTACLE_PERIOD_MS));
  }
}

/* ========== TASK 4: TELEMETRY & UART LOGGING (Priority 1 - LOWEST) ========== */
/*
   Company Use-Case: System status logging for diagnostics & debugging
   SOFT DEADLINE: Must complete within 1000ms (1Hz)
   Reason: Logs are useful for post-flight analysis but not time-critical
   Missing deadline doesn't affect drone safety or performance
*/
void vTaskTelemetry(void *pvParameters) {
  log_msg("TELEMETRY", "Initializing telemetry task");

  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t log_count = 0;

  for (;;) {
    gTaskStartTime_us[3] = esp_timer_get_time();

    // Acquire mutex and log system state
    if (xSemaphoreTake(xMutexSystemState, pdMS_TO_TICKS(50)) == pdTRUE) {
      Serial.printf("\n=== TELEMETRY REPORT #%lu ===\n", ++log_count);
      Serial.printf("Dust Level: %u (0-4095)\n", gSystemState.dust_level);
      Serial.printf("Obstacle Distance: %u\n", gSystemState.obstacle_distance);
      Serial.printf("Battery: %u mV\n", gSystemState.battery_voltage);
      Serial.printf("Brush Motor: %d%% (PWM %u)\n",
                    gSystemState.motor_brush_pwm / 255 * 100,
                    gSystemState.motor_brush_pwm);
      Serial.printf("Pump Valve: %s\n", gSystemState.pump_valve_state ? "ACTIVE" : "IDLE");
      Serial.printf("Propeller L: %u, R: %u\n",
                    gSystemState.prop_thrust_L,
                    gSystemState.prop_thrust_R);
      Serial.printf("System Health: %s\n", gSystemState.system_healthy ? "OK" : "RECOVERY");
      Serial.printf("Hard Deadline Misses: %lu\n", gDeadlineMisses);
      Serial.printf("Timestamp: %lu µs\n", gSystemState.timestamp_us);
      Serial.printf("===========================\n\n");

      xSemaphoreGive(xMutexSystemState);
    }

    // Check event queue for high-priority events
    Event_t evt;
    while (xQueueReceive(xEventQueue, &evt, 0) == pdTRUE) {
      if (evt.event_code == 0x10) {
        Serial.printf("EVENT LOG: Dirty panel detected at task %d, t=%llu µs\n",
                      evt.task_id, evt.timestamp_us);
      }
    }

    // SOFT DEADLINE CHECK (not critical)
    log_deadline_check(3, TELEMETRY_PERIOD_MS * 1000, 0);  // 0 = SOFT

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
  }
}

/* ========== TASK 5: HEARTBEAT & STATUS LED (Priority 2) ========== */
/*
   Company Use-Case: Visual system health indicator
   SOFT DEADLINE: Must complete within 100ms
   Reason: LED blink is cosmetic; missing deadline doesn't affect safety
*/
void vTaskHeartbeat(void *pvParameters) {
  log_msg("HEARTBEAT", "Initializing heartbeat task");

  pinMode(PIN_LED_HEARTBEAT, OUTPUT);
  pinMode(PIN_LED_HARD_DEADLINE, OUTPUT);
  pinMode(PIN_LED_SOFT_DEADLINE, OUTPUT);

  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t led_state = 0;

  for (;;) {
    gTaskStartTime_us[4] = esp_timer_get_time();

    // Blink heartbeat LED
    digitalWrite(PIN_LED_HEARTBEAT, led_state);
    led_state = !led_state;

    // Soft deadline indicator
    if (gDeadlineMisses > 0) {
      digitalWrite(PIN_LED_SOFT_DEADLINE, HIGH);
    } else {
      digitalWrite(PIN_LED_SOFT_DEADLINE, LOW);
    }

    // SOFT DEADLINE CHECK
    log_deadline_check(4, STATUS_BLINK_MS * 1000, 0);  // 0 = SOFT

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(STATUS_BLINK_MS));
  }
}

/* ========== SETUP & SCHEDULER INITIALIZATION ========== */
void setup() {
  Serial.begin(115200);
  vTaskDelay(pdMS_TO_TICKS(100));

  Serial.println("\n\n====================================");
  Serial.println("SOLAR PANEL CLEANING DRONE");
  Serial.println("Real-Time System - FreeRTOS ESP32");
  Serial.println("====================================\n");

  // Initialize hardware
  pinMode(PIN_EMERGENCY_STOP, INPUT_PULLUP);
  pinMode(PIN_LED_HEARTBEAT, OUTPUT);

  // Create synchronization primitives
  xMutexSystemState = xSemaphoreCreateMutex();
  xBinarySemEmergency = xSemaphoreCreateBinary();
  xBinarySemMotorReady = xSemaphoreCreateBinary();
  xEventQueue = xQueueCreate(10, sizeof(Event_t));
  xTelemetryQueue = xQueueCreate(5, sizeof(SystemState_t));

  if (!xMutexSystemState || !xBinarySemEmergency || !xEventQueue) {
    log_msg("SETUP", "FATAL: Failed to create synchronization objects");
    while (1);
  }

  // Attach emergency stop ISR to GPIO25
  attachInterrupt(digitalPinToInterrupt(PIN_EMERGENCY_STOP), ISR_EmergencyStop, FALLING);

  log_msg("SETUP", "Creating FreeRTOS tasks...");

  // Create all tasks with proper priorities (higher number = higher priority)
  xTaskCreate(vTaskMotorControl,    "MotorCtrl",  4096, NULL, 4, &xTaskMotorControl);
  xTaskCreate(vTaskSensorFusion,    "Sensors",    3072, NULL, 3, &xTaskSensorFusion);
  xTaskCreate(vTaskObstacleAvoid,   "Obstacle",   3072, NULL, 3, &xTaskObstacleAvoid);
  xTaskCreate(vTaskHeartbeat,       "Heartbeat",  2048, NULL, 2, &xTaskHeartbeat);
  xTaskCreate(vTaskTelemetry,       "Telemetry",  3072, NULL, 1, &xTaskTelemetry);

  // Wait for motor initialization to complete
  xSemaphoreTake(xBinarySemMotorReady, pdMS_TO_TICKS(5000));

  log_msg("SETUP", "All tasks created - Scheduler starting");
  log_msg("SETUP", "Press Emergency Stop button (GPIO25) to trigger ISR");
}

void loop() {
  // FreeRTOS scheduler manages everything
  vTaskDelete(NULL);
}