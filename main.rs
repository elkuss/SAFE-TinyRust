// ================================================================
// SAFE-TinyRust: Adaptive Distance Monitoring System
// ESP32-S3-WROOM-1 N16R8 | HC-SR04 | 3x LEDs
// No std, no main — bare metal embedded Rust
// ================================================================

// Tell Rust: do NOT use the standard library (no heap, no OS)
#![no_std]
// Tell Rust: do NOT generate a standard main() entry point
#![no_main]

// Import panic handler — prints panic info over serial and halts
use esp_backtrace as _;
// Import println! macro for UART serial output
use esp_println::println;
// Import ESP-HAL hardware abstractions
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{Io, Level, Output, Input, Pull},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    timer::timg::TimerGroup,
};
// heapless::Vec — a fixed-size Vec that lives on the stack (no heap)
use heapless::Vec;

// ================================================================
// CONSTANTS — Thresholds and Configuration
// ================================================================

/// Distance threshold: below this = DANGER (cm)
const DANGER_THRESHOLD: f32 = 20.0;
/// Distance threshold: below this = WARNING (cm)
const WARNING_THRESHOLD: f32 = 50.0;
/// Moving average window size (number of samples to average)
const WINDOW_SIZE: usize = 5;
/// Maximum measurable distance (HC-SR04 max range = ~400cm)
const MAX_DISTANCE: f32 = 400.0;
/// Minimum measurable distance (HC-SR04 blind spot = 2cm)
const MIN_DISTANCE: f32 = 2.0;
/// Speed of sound in cm/microsecond
const SOUND_CM_PER_US: f32 = 0.0343;

// ================================================================
// CLASSIFICATION ENUM
// Represents the system safety state
// ================================================================

#[derive(Debug, Clone, Copy, PartialEq)]
enum SafetyClass {
    Safe,     // distance > WARNING_THRESHOLD → Blue LED
    Warning,  // DANGER_THRESHOLD < d ≤ WARNING_THRESHOLD → Green LED
    Danger,   // distance ≤ DANGER_THRESHOLD → Red LED
    Fault,    // sensor read error or out-of-range
}

// ================================================================
// MOVING AVERAGE FILTER
// Smooths out noisy ultrasonic readings
// ================================================================

/// Computes the average of a slice of f32 values
fn moving_average(buf: &Vec<f32, WINDOW_SIZE>) -> f32 {
    if buf.is_empty() {
        return 0.0;
    }
    // Sum all values, divide by count
    let sum: f32 = buf.iter().sum();
    sum / buf.len() as f32
}

// ================================================================
// TINYML-INSPIRED ADAPTIVE CLASSIFIER
// Uses thresholds that adapt based on recent average
// ================================================================

/// Classify distance with adaptive thresholds
/// The classifier adjusts its sensitivity based on the
/// moving average — this mimics a simple TinyML decision tree
fn classify_distance(distance: f32, avg: f32) -> SafetyClass {
    // Fault detection: out of sensor range
    if distance < MIN_DISTANCE || distance > MAX_DISTANCE {
        return SafetyClass::Fault;
    }

    // Adaptive factor: if average is high, we are more relaxed;
    // if average is low, we are more sensitive to danger.
    // This emulates a simple learned context from the environment.
    let adaptive_factor: f32 = if avg > 100.0 {
        1.2  // Open area: loosen thresholds slightly
    } else if avg > 50.0 {
        1.0  // Normal environment: use base thresholds
    } else {
        0.8  // Tight space: tighten thresholds (more sensitive)
    };

    let adaptive_danger  = DANGER_THRESHOLD  * adaptive_factor;
    let adaptive_warning = WARNING_THRESHOLD * adaptive_factor;

    if distance <= adaptive_danger {
        SafetyClass::Danger
    } else if distance <= adaptive_warning {
        SafetyClass::Warning
    } else {
        SafetyClass::Safe
    }
}

// ================================================================
// ULTRASONIC MEASUREMENT
// Sends a TRIG pulse and times the ECHO response
// ================================================================

/// Measures distance using HC-SR04 ultrasonic sensor
/// Returns Some(distance_cm) on success, None on timeout/fault
fn measure_distance(
    trig: &mut Output,
    echo: &mut Input,
    delay: &mut Delay,
) -> Option<f32> {

    // Step 1: Ensure TRIG is LOW before sending pulse
    trig.set_low();
    delay.delay_micros(2);

    // Step 2: Send 10µs HIGH pulse on TRIG
    // This tells HC-SR04 to emit ultrasonic burst
    trig.set_high();
    delay.delay_micros(10);
    trig.set_low();

    // Step 3: Wait for ECHO to go HIGH (max wait = 500ms)
    // HC-SR04 raises ECHO when it receives the reflection
    let mut timeout = 500_000u32; // 500ms in microseconds
    while echo.is_low() {
        delay.delay_micros(1);
        timeout -= 1;
        if timeout == 0 {
            return None; // Timeout — no object detected
        }
    }

    // Step 4: Count how many microseconds ECHO stays HIGH
    // Duration = time for sound to travel to object and back
    let mut echo_duration: u32 = 0;
    while echo.is_high() {
        delay.delay_micros(1);
        echo_duration += 1;
        if echo_duration > 30_000 {
            return None; // Too long = out of range
        }
    }

    // Step 5: Calculate distance
    // Distance = (echo_duration µs × speed of sound) / 2
    // Divide by 2 because sound travels TO object and BACK
    let distance = (echo_duration as f32 * SOUND_CM_PER_US) / 2.0;
    Some(distance)
}

// ================================================================
// MAIN ENTRY POINT
// #[entry] is the esp-hal macro that replaces fn main()
// ================================================================

#[entry]
fn main() -> ! {  // Return type "!" means this function never returns

    // ── Hardware initialization ──────────────────────────────────

    // Take ownership of all ESP32-S3 peripherals (GPIO, UART, etc.)
    let peripherals = Peripherals::take();
    let system      = SystemControl::new(peripherals.SYSTEM);

    // Configure clock — maximum speed (240MHz for ESP32-S3)
    let clocks = ClockControl::max(system.clock_control).freeze();

    // Initialize GPIO subsystem
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // Initialize hardware delay (backed by system timer)
    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut delay = Delay::new(&clocks);

    // ── GPIO Configuration ───────────────────────────────────────

    // TRIG pin: Output (we drive this to trigger HC-SR04)
    let mut trig = Output::new(io.pins.gpio5, Level::Low);
    // ECHO pin: Input (we read the pulse duration back)
    let mut echo = Input::new(io.pins.gpio18, Pull::None);

    // LED pins: Output, all start LOW (off)
    let mut led_blue  = Output::new(io.pins.gpio2,  Level::Low);
    let mut led_green = Output::new(io.pins.gpio4,  Level::Low);
    let mut led_red   = Output::new(io.pins.gpio15, Level::Low);

    // ── Moving Average Buffer ────────────────────────────────────
    // heapless::Vec with compile-time max size WINDOW_SIZE (= 5)
    let mut readings: Vec<f32, WINDOW_SIZE> = Vec::new();

    // ── Startup Banner ───────────────────────────────────────────
    println!("================================================");
    println!("  SAFE-TinyRust v1.0");
    println!("  ESP32-S3 Adaptive Distance Monitor");
    println!("  DANGER <{}cm | WARNING <{}cm | SAFE otherwise",
             DANGER_THRESHOLD as u32, WARNING_THRESHOLD as u32);
    println!("================================================");

    // ── Sample counter for adaptive monitoring ───────────────────
    let mut sample_count: u32 = 0;
    let mut fault_count: u32  = 0;

    // ── MAIN LOOP — runs forever ─────────────────────────────────
    loop {
        sample_count += 1;

        // ── 1. Measure distance ──────────────────────────────────
        let raw_distance = measure_distance(&mut trig, &mut echo, &mut delay);

        match raw_distance {
            None => {
                // Sensor fault or timeout
                fault_count += 1;
                println!("[#{:04}] FAULT: sensor timeout #{}", sample_count, fault_count);

                // Blink all LEDs to indicate fault
                led_blue.set_high(); led_green.set_high(); led_red.set_high();
                delay.delay_millis(100);
                led_blue.set_low(); led_green.set_low(); led_red.set_low();
            }

            Some(distance) => {
                // ── 2. Update moving average buffer ─────────────────
                if readings.is_full() {
                    readings.remove(0); // Remove oldest reading (sliding window)
                }
                let _ = readings.push(distance);

                // ── 3. Compute moving average ────────────────────────
                let avg = moving_average(&readings);

                // ── 4. TinyML-inspired classification ───────────────
                let class = classify_distance(avg, avg);

                // ── 5. LED control ───────────────────────────────────
                // Turn off all LEDs first, then light the correct one
                led_blue.set_low(); led_green.set_low(); led_red.set_low();

                match class {
                    SafetyClass::Safe    => { led_blue.set_high(); }
                    SafetyClass::Warning => { led_green.set_high(); }
                    SafetyClass::Danger  => { led_red.set_high(); }
                    SafetyClass::Fault   => {
                        led_blue.set_high();
                        led_red.set_high();
                    }
                }

                // ── 6. Serial output ─────────────────────────────────
                println!(
                    "[#{:04}] Raw:{:6.1}cm Avg:{:6.1}cm Status:{:?} Faults:{}",
                    sample_count, distance, avg, class, fault_count
                );
            }
        }

        // ── 7. Wait before next reading (100ms = 10 readings/sec) ──
        delay.delay_millis(100);
    }
}