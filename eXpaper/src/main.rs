#![no_std]
#![no_main]

use core::fmt::Write;
use display_interface_spi::SPIInterface;
use embassy_executor::Spawner;
use embedded_graphics::{
    geometry::Point,
    mono_font::MonoTextStyle,
    text::{Text, TextStyle},
    Drawable,
};
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    spi::{self, master::Spi},
    time::Rate,
};
use heapless::String;
use log::error;
use profont::PROFONT_24_POINT;
use weact_studio_epd::{graphics::Display290BlackWhite, Color};
use weact_studio_epd::{graphics::DisplayRotation, WeActStudio290BlackWhiteDriver};

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max());
    let peripherals: esp_hal::peripherals::Peripherals = esp_hal::init(config);

    // Pins for Seeedstudio XIAO ESP32-C6
    // let sclk = io.pins.gpio19; // D8 / GPIO19
    // let mosi = io.pins.gpio18; // D10 / GPIO18
    // let cs = io.pins.gpio20; // D9 / GPIO20
    // let dc = io.pins.gpio21; // D3 / GPIO21
    // let rst = io.pins.gpio22; // D4 / GPIO22
    // let busy = io.pins.gpio23; // D5 / GPIO23
    //
    let sclk_pin = peripherals.GPIO19;
    let mosi_pin = peripherals.GPIO18; //NOTE: marked as sda on board
    let cs_pin = peripherals.GPIO20;
    let dc_pin = peripherals.GPIO21;
    let rst_pin = peripherals.GPIO22;
    let busy_pin = peripherals.GPIO23;

    // // Convert pins into InputPins and OutputPins
    // /*
    //     CS: OutputPin,
    //     BUSY: InputPin,
    //     DC: OutputPin,
    //     RST: OutputPin,
    // */
    let cs = Output::new(cs_pin, Level::High, OutputConfig::default());
    let busy = Input::new(busy_pin, InputConfig::default().with_pull(Pull::Up));
    let dc = Output::new(dc_pin, Level::Low, OutputConfig::default());
    let rst = Output::new(rst_pin, Level::High, OutputConfig::default());

    let delay = Delay::new();

    let spi_bus = {
        // let spi_bus = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks)
        log::info!("Intializing SPI Bus...");

        let config = esp_hal::spi::master::Config::default()
            .with_frequency(Rate::from_khz(100))
            .with_mode(spi::Mode::_0);

        // let spi_bus = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0, &clocks).with_pins(
        //     Some(sclk),
        //     Some(mosi),
        //     NO_PIN,
        //     NO_PIN, // cs is handled by the exclusive device
        // );

        Spi::new(peripherals.SPI2, config)
            .inspect_err(|e| {
                error!("config error while setting up SPI: {e}");
            })
            .unwrap()
            .with_sck(sclk_pin)
            .with_mosi(mosi_pin)
    };

    log::info!("Intializing SPI Device...");
    let spi_device = ExclusiveDevice::new(spi_bus, cs, delay)
        .inspect_err(|e| error!("Error creating exclusive spi device: {e}"))
        .unwrap();

    let spi_interface = SPIInterface::new(spi_device, dc);

    // Setup EPD
    log::info!("Intializing EPD...");
    let mut driver = WeActStudio290BlackWhiteDriver::new(spi_interface, busy, rst, delay);
    let mut display = Display290BlackWhite::new();
    display.set_rotation(DisplayRotation::Rotate90);
    driver.init().unwrap();

    let style = MonoTextStyle::new(&PROFONT_24_POINT, Color::Black);
    let _ = Text::with_text_style(
        "Hello World!",
        Point::new(8, 68),
        style,
        TextStyle::default(),
    )
    .draw(&mut display);

    driver.full_update(&display).unwrap();

    log::info!("Sleeping for 5s...");
    driver.sleep().unwrap();
    delay.delay_millis(5_000);

    let mut n: u8 = 0;

    loop {
        log::info!("Wake up!");
        driver.wake_up().unwrap();

        display.clear(Color::White);

        let mut string_buf = String::<30>::new();
        write!(string_buf, "Hello World {}!", n).unwrap();
        let _ = Text::with_text_style(&string_buf, Point::new(8, 68), style, TextStyle::default())
            .draw(&mut display)
            .unwrap();
        string_buf.clear();

        // TODO: try fast update?
        // driver.full_update(&display).unwrap();
        driver.fast_update(&display).unwrap();

        n = n.wrapping_add(1); // Wrap from 0..255

        log::info!("Sleeping for 1s...");
        driver.sleep().unwrap();
        delay.delay_millis(1_000);
    }
}
