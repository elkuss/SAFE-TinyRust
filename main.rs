#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    time::Instant,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal::main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let delay = Delay::new();

    // LED aktif-HIGH: set_high() = nyala, set_low() = mati
    let mut led_blue   = Output::new(peripherals.GPIO14, Level::Low, OutputConfig::default()); // SAFE
    let mut led_yellow = Output::new(peripherals.GPIO13, Level::Low, OutputConfig::default()); // ALERT
    let mut led_red    = Output::new(peripherals.GPIO12, Level::Low, OutputConfig::default()); // DANGER
    let mut trig        = Output::new(peripherals.GPIO5,  Level::Low, OutputConfig::default());

    let echo_cfg = InputConfig::default().with_pull(Pull::Down);
    let echo     = Input::new(peripherals.GPIO18, echo_cfg);

    println!("=== SAFE-TinyRust v1.0 ===");

    loop {
        // Trigger pulse 10us
        trig.set_low();
        delay.delay_micros(2u32);
        trig.set_high();
        delay.delay_micros(10u32);
        trig.set_low();

        // Tunggu echo naik (timeout 30ms)
        let wait_start = Instant::now();
        while echo.is_low() {
            if wait_start.elapsed().as_micros() > 30_000 {
                break;
            }
        }

        // Ukur durasi echo high pakai timer asli
        let pulse_start = Instant::now();
        while echo.is_high() {
            if pulse_start.elapsed().as_micros() > 30_000 {
                break;
            }
        }
        let pulse_us = pulse_start.elapsed().as_micros() as u32;

        let distance_cm = pulse_us / 58;

        // Matikan semua LED dulu tiap siklus, biar nggak nyisa nyala dari status sebelumnya
        led_blue.set_low();
        led_yellow.set_low();
        led_red.set_low();

        let status = if distance_cm < 20 {
            led_red.set_high();
            "DANGER"
        } else if distance_cm <= 50 {
            led_yellow.set_high();
            "ALERT"
        } else {
            led_blue.set_high();
            "SAFE"
        };

        println!("pulse_us: {} | Jarak: {} cm | Status: {}", pulse_us, distance_cm, status);

        delay.delay_millis(1000u32); // 1 detik
    }
}