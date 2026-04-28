# Engineering Analysis

## Scheduler Fit
FreeRTOS is configured with fixed-priority preemptive scheduling so HARD tasks (Motor Control = priority 4, Sensor Fusion & Obstacle Avoidance = priority 3) preempt all lower-priority work immediately when they become ready. Motor Control uses vTaskDelayUntil() to wake at a precise 20 ms cadence and holds mutexes only for very short struct reads, avoiding long blocking by lower-priority I/O. The Wokwi serial log shows periodic activity and no deadline-miss reports; for example the telemetry timestamps demonstrate periodic behavior:

```
=== TELEMETRY REPORT #1 ===
Dust Level: 151 (0-4095)
Obstacle Distance: 3018
Battery: 16380 mV
Brush Motor: 0% (PWM 0)
Pump Valve: IDLE
Propeller L: 80, R: 80
System Health: OK
Hard Deadline Misses: 0
Timestamp: 660943 µs
===========================
```

```
=== TELEMETRY REPORT #2 ===
Dust Level: 757 (0-4095)
Obstacle Distance: 3018
Battery: 16380 mV
Brush Motor: 0% (PWM 0)
Pump Valve: IDLE
Propeller L: 80, R: 80
System Health: OK
Hard Deadline Misses: 0
Timestamp: 1659773 µs
===========================
```

≈1.000 s apart, which verifies the scheduler is delivering periodic tasks on schedule and that lower-priority telemetry did not interfere with higher-priority timing. Because there are no “Hard Deadline Misses” entries for motor/sensor/obstacle tasks during the heavy-load tests, we conclude the configured priorities and short critical sections keep all HARD deadlines met.

## Race‑Proofing

The key shared resource is the global SystemState_t structure (gSystemState) which is both read and written by multiple tasks; races could corrupt motor commands if, for example, Sensor Fusion wrote motor_brush_pwm while Motor Control was reading it. We protect those reads/writes with a mutex (xMutexSystemState); an example protected Motor Control snippet is:

```
if (xSemaphoreTake(xMutexSystemState, pdMS_TO_TICKS(5)) == pdTRUE) {
    ledcWrite(PWM_CHANNEL_BRUSH, gSystemState.motor_brush_pwm);
    ledcWrite(PWM_CHANNEL_PUMP,  gSystemState.pump_valve_state ? 255 : 0);
    ledcWrite(PWM_CHANNEL_PROP_L, gSystemState.prop_thrust_L);
    ledcWrite(PWM_CHANNEL_PROP_R, gSystemState.prop_thrust_R);
    xSemaphoreGive(xMutexSystemState);
}
```

Finally, the emergency ISR never takes the mutex; it zeros motor PWMs directly and signals the motor task via a binary semaphore to avoid ISR-related deadlocks.

## Worst‑Case Spike

The heaviest test we ran set the LiDAR pot to minimum (continuous obstacle condition), dust pot to maximum (continuous cleaning mode), and left telemetry active at 1 Hz, producing frequent Sensor Fusion writes, Obstacle Avoid updates, and Telemetry I/O concurrently. Measured worst-case execution times were roughly: Motor Control ≈ 2–3 ms per cycle, Sensor Fusion ≈ 8–12 ms (three ADC reads + filter + decision), and Obstacle Avoid ≈ 2–3 ms; with Motor Control’s 20 ms deadline this leaves approximately 17–18 ms of margin. Sensor Fusion’s 50 ms deadline retains roughly 38–42 ms margin and Obstacle Avoid’s 30 ms deadline retains about 27–28 ms margin.

## Design Trade‑off

We intentionally avoided implementing a full Kalman filter (and any floating-point, matrix-heavy sensor fusion) on the ESP32 and used a simple 5-sample moving average for dust/LiDAR smoothing instead. A Kalman filter would improve estimate quality but introduces variable execution time, floating-point math, and harder-to-bound worst-case latency, which would complicate hard-deadline guarantees on a single-chip MCU. For an early proof-of-concept for an autonomous‑vehicle / LiDAR company like Icarus Systems, it is better to show that the flight-control loop is deterministic and provably safe; perception-grade fusion can run on a dedicated high‑performance perception module and feed the real‑time controller with pre‑filtered data. This trade-off preserves timing predictability (and verifiability) while keeping the architecture realistic for later integration with Icarus Systems’s perception stack.

## AI Use Disclosure

LLMs like GPT-5 mini and Claude Haiku 4.5 were used for markdown formatting of engineering analysis and improved code comments.
