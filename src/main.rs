#![no_std]
#![no_main]

extern crate panic_halt;

mod backlight;
mod delay;
mod monotonic_nrf52;

use nrf52832_hal::gpio::Level;
use nrf52832_hal::{self as p_hal, pac};
use p_hal::{delay::Delay, spim, twim};

use cortex_m_rt as rt;
use cortex_m_semihosting::hprintln;
use cst816s::CST816S;
use cstr_core::CStr;
use embedded_graphics::prelude::*;
use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use embedded_hal::digital::v2::OutputPin;
use numtoa::NumToA;
use rt::entry;
use st7789::{Orientation, ST7789};

use lvgl::input_device::{InputData, Pointer};
use lvgl::style::Style;
use lvgl::widgets::{Btn, Label};
use lvgl::{self, Align, Color, Part, State, Widget, UI};

use crate::monotonic_nrf52::Instant;
use core::panic::PanicInfo;
use core::time::Duration;
use display_interface_spi::SPIInterfaceNoCS;

///
/// This example was written and tested for the PineTime smart watch
///
#[entry]
fn main() -> ! {
    hprintln!("\nStarting...").unwrap();

    let cp = pac::CorePeripherals::take().unwrap();
    let mut delay_source = Delay::new(cp.SYST);

    // PineTime has a 32 MHz HSE (HFXO) and a 32.768 kHz LSE (LFXO)
    // Optimize clock config
    let dp = pac::Peripherals::take().unwrap();

    //let lcd_delay = delay::TimerDelay::new(dp.TIMER0);

    // Initialize monotonic timer on TIMER1 (for RTFM)
    monotonic_nrf52::Tim1::initialize(dp.TIMER1);

    // Set up clocks. On reset, the high frequency clock is already used,
    // but we also need to switch to the external HF oscillator. This is
    // needed for Bluetooth to work.
    let _clocks = p_hal::clocks::Clocks::new(dp.CLOCK).enable_ext_hfosc();

    let gpio = p_hal::gpio::p0::Parts::new(dp.P0);

    // Set up SPI pins
    let spi_clk = gpio.p0_02.into_push_pull_output(Level::Low).degrade();
    let spi_mosi = gpio.p0_03.into_push_pull_output(Level::Low).degrade();
    let spi_pins = spim::Pins {
        sck: spi_clk,
        miso: None,
        mosi: Some(spi_mosi),
    };

    // Enable backlight
    let mut backlight = backlight::Backlight::init(
        gpio.p0_14.into_push_pull_output(Level::High).degrade(),
        gpio.p0_22.into_push_pull_output(Level::High).degrade(),
        gpio.p0_23.into_push_pull_output(Level::High).degrade(),
        1,
    );
    backlight.set(5);

    delay_source.delay_ms(1u8);
    // internal i2c0 bus devices: BMA421 (accel), HRS3300 (hrs), CST816S (TouchPad)
    // BMA421-INT:  P0.08
    // TP-INT: P0.28
    let i2c0_pins = twim::Pins {
        scl: gpio.p0_07.into_floating_input().degrade(),
        sda: gpio.p0_06.into_floating_input().degrade(),
    };
    let i2c_port = twim::Twim::new(dp.TWIM1, i2c0_pins, twim::Frequency::K400);

    // setup touchpad external interrupt pin: P0.28/AIN4 (TP_INT)
    let touch_int = gpio.p0_28.into_pullup_input().degrade();
    // setup touchpad reset pin: P0.10/NFC2 (TP_RESET)
    let touch_rst = gpio.p0_10.into_push_pull_output(Level::High).degrade();

    let mut touchpad = CST816S::new(i2c_port, touch_int, touch_rst);
    touchpad.setup(&mut delay_source).unwrap();

    // Set up LCD pins
    // LCD_CS (P0.25): Chip select
    let mut lcd_cs = gpio.p0_25.into_push_pull_output(Level::Low);
    // LCD_RS (P0.18): Data/clock pin
    let lcd_dc = gpio.p0_18.into_push_pull_output(Level::Low);
    // LCD_RESET (P0.26): Display reset
    let lcd_rst = gpio.p0_26.into_push_pull_output(Level::Low);

    // Initialize SPI
    let spi = spim::Spim::new(
        dp.SPIM0,
        spi_pins,
        // Use SPI at 8MHz (the fastest clock available on the nRF52832)
        // because otherwise refreshing will be super slow.
        spim::Frequency::M8,
        // SPI must be used in mode 3. Mode 0 (the default) won't work.
        spim::MODE_3,
        122,
    );

    // display interface abstraction from SPI and DC
    let di = SPIInterfaceNoCS::new(spi, lcd_dc);

    // Chip select must be held low while driving the display. It must be high
    // when using other SPI devices on the same bus (such as external flash
    // storage) so that the display controller won't respond to the wrong
    // commands.
    lcd_cs.set_low().unwrap();

    // Initialize LCD
    let mut lcd = ST7789::new(
        di,
        lcd_rst,
        lvgl::HOR_RES_MAX as u16,
        lvgl::VER_RES_MAX as u16,
    );

    lcd.init(&mut delay_source).unwrap();
    lcd.set_orientation(Orientation::Portrait).unwrap();

    hprintln!("\nAll devices set!").unwrap();

    // Initialize LVGL
    let mut ui = UI::init().unwrap();
    ui.disp_drv_register(lcd).unwrap();

    // Define the initial state of the input
    let mut latest_touch_point = Point::new(0, 0);
    let mut latest_touch_status = InputData::Touch(latest_touch_point.clone())
        .released()
        .once();

    // Register a new input device that's capable of reading the current state of the input
    let mut touch_device = Pointer::new(|| latest_touch_status);
    ui.indev_drv_register(&mut touch_device).unwrap();

    // Create screen and widgets
    let mut screen = ui.scr_act().unwrap();

    // Draw a black background to the screen
    let mut screen_style = Style::default();
    screen_style.set_bg_color(State::DEFAULT, Color::from_rgb((0, 0, 0)));
    screen.add_style(Part::Main, screen_style).unwrap();

    // Create the button
    let text_click_me = CStr::from_bytes_with_nul("Click me!\0".as_bytes()).unwrap();
    let mut button = Btn::new(&mut screen).unwrap();
    button
        .set_align(&mut screen, Align::InLeftMid, 30, 0)
        .unwrap();
    button.set_size(180, 80).unwrap();
    let mut btn_lbl = Label::new(&mut button).unwrap();
    btn_lbl.set_text(text_click_me).unwrap();

    let mut time_style = Style::default();
    time_style.set_text_color(State::DEFAULT, Color::from_rgb((255, 255, 255)));

    let mut time_lbl = Label::new(&mut screen).unwrap();
    time_lbl
        .set_align(&mut button, Align::OutTopMid, 0, -50)
        .unwrap();
    let time_text = CStr::from_bytes_with_nul("TIME\0".as_bytes()).unwrap();
    time_lbl.set_text(time_text).unwrap();
    time_lbl.add_style(Part::Main, time_style).unwrap();

    button
        .on_event(|mut btn, event| {
            if let lvgl::Event::Clicked = event {
                btn.toggle().unwrap();
            }
        })
        .unwrap();

    let mut loop_time = Instant::now();
    let mut total_time = Duration::from_secs(0);
    let mut time_text_buf = [0u8; 20];

    let mut last_update_time_secs = 0;
    let mut last_inc_time_ms = 0;

    hprintln!("\nLVGL widgets built!").unwrap();
    loop {
        ui.task_handler();

        if let Some(evt) = touchpad.read_one_touch_event(true) {
            latest_touch_point = Point::new(evt.x, evt.y);
            // Pressed
            latest_touch_status = InputData::Touch(latest_touch_point.clone())
                .pressed()
                .once();
        } else {
            // Released
            latest_touch_status = InputData::Touch(latest_touch_point.clone())
                .released()
                .once();
            delay_source.delay_us(1u32);
        }

        total_time += Duration::from_micros(loop_time.elapsed().as_cycles() as u64);

        if total_time.as_secs() > last_update_time_secs {
            last_update_time_secs = total_time.as_secs();

            let text = (total_time.as_secs() as u32).numtoa(10, &mut time_text_buf);

            let time_text = unsafe { CStr::from_bytes_with_nul_unchecked(&text) };
            time_lbl.set_text(time_text).unwrap();
            time_lbl
                .set_align(&mut button, Align::OutTopMid, 0, -50)
                .unwrap();

            // Reset buffer
            for p in time_text_buf.iter_mut() {
                *p = '\0' as u8;
            }
        }

        if total_time.as_millis() > last_inc_time_ms {
            //let diff_ms = total_time.as_millis() - last_inc_time_ms;
            last_inc_time_ms = total_time.as_millis();
        }
        ui.tick_inc(Duration::from_millis(50));

        loop_time = Instant::now();
    }
}
