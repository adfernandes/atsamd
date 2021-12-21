#![no_std]

//! # Adafruit QT Py Board Support Package
//!
//! This crate provides a board support package for the Adafruit QT Py board.
//! This device is a small form-factor breadboard-compatible SAMD21E-based
//! device with an on-board WS2812 LED ("neopixel"), [STEMMA I2C][stemma]
//! ([Qwiic][qwiic]-compatible) connector, and USB-C running USB
//! 2.0 connectivity.
//!
//! # Useful External Links
//! - [Adafruit QT Py Product Page][qtpy]
//! - [QT Py schematics][schematics]
//!
//! [qtpy]: https://learn.adafruit.com/adafruit-qt-py
//! [stemma]: https://www.adafruit.com/category/1005
//! [qwiic]: https://www.sparkfun.com/qwiic
//! [schematics]: https://cdn-learn.adafruit.com/assets/assets/000/095/390/original/adafruit_products_QTPy_sch.png

#[cfg(feature = "rt")]
pub use cortex_m_rt::entry;

pub use atsamd_hal as hal;
use hal::clock::GenericClockController;
pub use hal::ehal;
pub use hal::ehal::spi::Mode;
pub use hal::pac;
use hal::sercom::v2::spi;
use hal::sercom::v2::uart::{self, BaudMode, Oversampling};
use hal::sercom::v2::{Sercom0, Sercom2};
use hal::sercom::I2CMaster1; // Sercom1
use hal::time::Hertz;

#[cfg(feature = "usb")]
use hal::usb::{usb_device::bus::UsbBusAllocator, UsbBus};

/// ## Pin Multiplex Groups
/// - board.A0 board.D0
/// - board.A1 board.D1
/// - board.A10 board.D10 board.MOSI
/// - board.A2 board.D2
/// - board.A3 board.D3
/// - board.A6 board.D6 board.TX
/// - board.A7 board.D7 board.RX
/// - board.A8 board.D8 board.SCLK
/// - board.A9 board.D9 board.MISO
/// - board.D4 board.SDA
/// - board.D5 board.SCL
/// - board.NEOPIXEL
/// - board.NEOPIXEL_POWER
pub mod pins {
    use super::hal;
    hal::bsp_pins!(

        // General Purpose Pins
        PA02 {
            name: a0,
            aliases: {
             // AlternateA: EicExtint2,
                AlternateB: AdcAin0,
                AlternateF: Tcc3Wo0,
            }
        }
        PA03 {
            name: a1,
            aliases: {
             // AlternateA: EicExtint3,
                AlternateB: AdcAin1,
                AlternateF: Tcc3Wo1,
            }
        }
        PA04 {
            name: a2,
            aliases: {
             // AlternateA: EicExtint4,
                AlternateB: AdcAin4,
                AlternateD: Sercom0Pad0,
                AlternateE: Tcc0Wo0,
                AlternateF: Tcc3Wo2,
            }
        }
        PA05 {
            name: a3,
            aliases: {
             // AlternateA: EicExtint5,
                AlternateB: AdcAin5,
                AlternateD: Sercom0Pad1,
                AlternateE: Tcc0Wo1,
                AlternateF: Tcc3Wo3,
            }
        }

        // UART Port Pins
        PA06 {
            // sercom0/pad[2]
            name: tx,
            aliases: {
                AlternateD: UartTx,
            }
        },
        PA07 {
            // sercom0/pad[3]
            name: rx,
            aliases: {
                AlternateD: UartRx,
            }
        }

        // SPI Port Pins
        PA09 {
            // sercom2/pad[1]
            name: miso,
            aliases: {
                AlternateD: SpiMiso,
            }
        }
        PA10 {
            // sercom2/pad[2]
            name: mosi,
            aliases: {
                AlternateD: SpiMosi,
            }
        }
        PA11 {
            // sercom2/pad[3]
            name: sclk,
            aliases: {
                AlternateD: SpiSclk,
            }
        }

        // I2C Port Pins
        PA16 {
            // sercom1/pad[0]
            name: sda,
            aliases: {
                AlternateC: I2cSda,
            }
        }
        PA17 {
            // sercom1/pad[1]
            name: scl,
            aliases: {
                AlternateC: I2cScl,
            }
        }

        // Neopixel Interface Pins
        PA15 {
            name: neopixel_power,
            aliases: {
                PushPullOutput: NeopixelPower,
            }
        }
        PA18 {
            name: neopixel_data,
            aliases: {
                PushPullOutput: NeopixelData,
            }
        }

        // USB Pins
        PA24 {
            name: usb_dm,
            aliases: {
                AlternateG: UsbDm,
            }
        }
        PA25 {
            name: usb_dp,
            aliases: {
                AlternateG: UsbDp,
            }
        }

        // Factory Non-Populated SPI Flash
        #[cfg(feature = "spi_flash")]
        PA08 {
            name: flash_cs,
            aliases: {
                PushPullOutput: FlashCs,
            }
        }
        #[cfg(feature = "spi_flash")]
        PA19 {
            name: flash_miso,
            aliases: {
                AlternateD: FlashMiso,
            }
        }
        #[cfg(feature = "spi_flash")]
        PA22 {
            name: flash_mosi,
            aliases: {
                AlternateC: FlashMosi,
            }
        }
        #[cfg(feature = "spi_flash")]
        PA23 {
            name: flash_sclk,
            aliases: {
                AlternateC: FlashSclk,
            }
        }
    );
}

pub use pins::*;

/// SPI pads for the labelled SPI peripheral
///
/// You can use these pads with other, user-defined [`spi::Config`]urations.
pub type SpiPads = spi::Pads<Sercom2, SpiMiso, SpiMosi, SpiSclk>;

/// SPI master for the labelled SPI peripheral
///
/// This type implements [`FullDuplex<u8>`](ehal::spi::FullDuplex).
pub type Spi = spi::Spi<spi::Config<SpiPads>, spi::Duplex>;

/// Convenience for setting up the labelled SPI peripheral.
/// This powers up SERCOM2 and configures it for use as an
/// SPI Master in the given SPI mode and baud rate.
pub fn spi_master(
    clocks: &mut GenericClockController,
    baud: impl Into<Hertz>,
    mode: ehal::spi::Mode,
    sercom2: pac::SERCOM2,
    pm: &mut pac::PM,
    sclk: impl Into<SpiSclk>,
    mosi: impl Into<SpiMosi>,
    miso: impl Into<SpiMiso>,
) -> Spi {
    let gclk0 = clocks.gclk0();
    let clock = clocks.sercom2_core(&gclk0).unwrap();
    let freq = clock.freq();
    let (miso, mosi, sclk) = (miso.into(), mosi.into(), sclk.into());
    let pads = spi::Pads::default().data_in(miso).data_out(mosi).sclk(sclk);
    spi::Config::new(pm, sercom2, pads, freq)
        .baud(baud)
        .spi_mode(mode)
        .enable()
}

/// I2C master for the labelled SDA & SCL pins on the
/// SERCOM1 peripheral and the I2CMaster1 controller.
pub type I2c = I2CMaster1<I2cSda, I2cScl>;

/// Convenience for setting up the labelled SDA, SCL pins to
/// operate as an I2C master running at the specified frequency.
pub fn i2c_master(
    clocks: &mut GenericClockController,
    baud: impl Into<Hertz>,
    sercom1: pac::SERCOM1,
    pm: &mut pac::PM,
    sda: impl Into<I2cSda>,
    scl: impl Into<I2cScl>,
) -> I2c {
    let gclk0 = clocks.gclk0();
    let clock = &clocks.sercom1_core(&gclk0).unwrap();
    let baud = baud.into();
    let sda = sda.into();
    let scl = scl.into();
    I2CMaster1::new(clock, baud, sercom1, pm, sda, scl)
}

/// UART pads for the labelled RX & TX pins
pub type UartPads = uart::Pads<Sercom0, UartRx, UartTx>;

/// UART device for the labelled RX & TX pins
pub type Uart = uart::Uart<uart::Config<UartPads>, uart::Duplex>;

/// Convenience for setting up the labelled RX, TX pins to
/// operate as a UART device running at the specified baud.
pub fn uart(
    clocks: &mut GenericClockController,
    baud: impl Into<Hertz>,
    sercom0: pac::SERCOM0,
    pm: &mut pac::PM,
    uart_rx: impl Into<UartRx>,
    uart_tx: impl Into<UartTx>,
) -> Uart {
    let gclk0 = clocks.gclk0();
    let clock = &clocks.sercom0_core(&gclk0).unwrap();
    let baud = baud.into();
    let pads = uart::Pads::default().rx(uart_rx.into()).tx(uart_tx.into());
    uart::Config::new(pm, sercom0, pads, clock.freq())
        .baud(baud, BaudMode::Fractional(Oversampling::Bits16))
        .enable()
}

#[cfg(feature = "usb")]
/// Convenience function for setting up USB
pub fn usb_allocator(
    usb: pac::USB,
    clocks: &mut GenericClockController,
    pm: &mut pac::PM,
    dm: impl Into<UsbDm>,
    dp: impl Into<UsbDp>,
) -> UsbBusAllocator<UsbBus> {
    let gclk0 = clocks.gclk0();
    let clock = &clocks.usb(&gclk0).unwrap();
    let (dm, dp) = (dm.into(), dp.into());
    UsbBusAllocator::new(UsbBus::new(clock, pm, dm, dp, usb))
}
