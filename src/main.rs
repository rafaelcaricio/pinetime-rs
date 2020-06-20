#![no_std]
#![no_main]

mod monotonic_nrf52;

use nrf52832_hal as p_hal;
use p_hal::gpio::{GpioExt, Level};
use p_hal::nrf52832_pac as pac;
use p_hal::{delay::Delay, spim, twim};

use cortex_m_rt as rt;
use cst816s::CST816S;
use embedded_graphics::prelude::*;
use rt::entry;
use st7789::Orientation;

use core::cell::Cell;
use cstr_core::CStr;
use lvgl::input_device::{BufferStatus, InputData, Pointer};
use lvgl::style::Style;
use lvgl::widgets::{Btn, Label};
use lvgl::{self, Align, Color, Part, State, Widget, UI};

use crate::monotonic_nrf52::Instant;
use core::panic::PanicInfo;
use core::time::Duration;
use embedded_hal::blocking::delay::DelayMs;
use nrf52832_hal::prelude::ClocksExt;

pub type HalSpimError = p_hal::spim::Error;

pub type Spim0PortType = p_hal::spim::Spim<pac::SPIM0>;
pub type DisplaySckPinType = p_hal::gpio::p0::P0_18<p_hal::gpio::Output<p_hal::gpio::PushPull>>;
pub type DisplayMosiPinType = p_hal::gpio::p0::P0_26<p_hal::gpio::Output<p_hal::gpio::PushPull>>;

///
/// This example was written and tested for the PineTime smart watch
///
#[entry]
fn main() -> ! {
    let cp = pac::CorePeripherals::take().unwrap();
    let mut delay_source = Delay::new(cp.SYST);

    // PineTime has a 32 MHz HSE (HFXO) and a 32.768 kHz LSE (LFXO)
    // Optimize clock config
    let dp = pac::Peripherals::take().unwrap();
    let _clockit = dp.CLOCK.constrain().enable_ext_hfosc();

    let port0 = dp.P0.split();

    // internal i2c0 bus devices: BMA421 (accel), HRS3300 (hrs), CST816S (TouchPad)
    // BMA421-INT:  P0.08
    // TP-INT: P0.28
    let i2c0_pins = twim::Pins {
        scl: port0.p0_07.into_floating_input().degrade(),
        sda: port0.p0_06.into_floating_input().degrade(),
    };
    let i2c_port = twim::Twim::new(dp.TWIM1, i2c0_pins, twim::Frequency::K400);
    // let i2c_bus0 = shared_bus::CortexMBusManager::new(i2c_port);

    delay_source.delay_ms(1u8);

    let spim0_pins = spim::Pins {
        sck: port0.p0_02.into_push_pull_output(Level::Low).degrade(),
        miso: None,
        mosi: Some(port0.p0_03.into_push_pull_output(Level::Low).degrade()),
    };

    // create SPIM0 interface, 8 Mbps, use 122 as "over read character"
    let spim0 = spim::Spim::new(dp.SPIM0, spim0_pins, spim::Frequency::M8, spim::MODE_3, 122);
    let spi_bus0 = shared_bus::CortexMBusManager::new(spim0);

    // backlight control pin for display: always on
    let mut _backlight = port0.p0_22.into_push_pull_output(Level::Low);
    // SPI chip select (CSN) for the display.
    let display_csn = port0.p0_25.into_push_pull_output(Level::High);
    // data/clock switch pin for display
    let display_dc = port0.p0_18.into_push_pull_output(Level::Low);
    // reset pin for display
    let display_rst = port0.p0_26.into_push_pull_output(Level::Low);

    // create display driver
    let mut display = st7789::new_display_driver(
        spi_bus0.acquire(),
        display_csn,
        display_dc,
        display_rst,
        lvgl::HOR_RES_MAX as u16,
        lvgl::VER_RES_MAX as u16,
    );
    display.init(&mut delay_source).unwrap();
    display.set_orientation(&Orientation::Portrait).unwrap();

    // setup touchpad external interrupt pin: P0.28/AIN4 (TP_INT)
    let touch_int = port0.p0_28.into_pullup_input().degrade();
    // setup touchpad reset pin: P0.10/NFC2 (TP_RESET)
    let touch_rst = port0.p0_10.into_push_pull_output(Level::High).degrade();

    let mut touchpad = CST816S::new(i2c_port, touch_int, touch_rst);
    touchpad.setup(&mut delay_source).unwrap();

    // Initialize LVGL
    let mut ui = UI::init().unwrap();
    ui.disp_drv_register(display).unwrap();

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

    let mut btn_state = false;
    button
        .on_event(|mut btn, event| {
            if let lvgl::Event::Clicked = event {
                btn_state = !btn_state;
                btn.toggle().unwrap();
            }
        })
        .unwrap();

    let mut loop_start = Instant::now();
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
        }

        ui.tick_inc(Duration::from_millis(50));
        //delay_source.delay_ms(2u32);
        loop_start = Instant::now();
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}
