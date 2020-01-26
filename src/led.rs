//! On-board LEDs
//!
//! - D48 (upper) = PB5
//! - D50 (middle) = PB1
//! - D49 (lower) = PB0
use embedded_hal::digital::v2::OutputPin;
use gd32vf103xx_hal::gpio::gpiob::{PB0, PB1, PB5};
use gd32vf103xx_hal::gpio::{Output, PushPull};

/// D48 (upper) LED
pub type LED1 = PB5<Output<PushPull>>;

/// D50 (middle) LED
pub type LED2 = PB1<Output<PushPull>>;

/// D49 (lower) LED
pub type LED3 = PB0<Output<PushPull>>;

/// Generic LED
pub trait Led {
    /// Turns the LED off
    fn off(&mut self);

    /// Turns the LED on
    fn on(&mut self);
}

impl Led for LED1 {
    fn off(&mut self) {
        self.set_high().unwrap();
    }

    fn on(&mut self) {
        self.set_low().unwrap();
    }
}

impl Led for LED2 {
    fn off(&mut self) {
        self.set_high().unwrap();
    }

    fn on(&mut self) {
        self.set_low().unwrap();
    }
}

impl Led for LED3 {
    fn off(&mut self) {
        self.set_high().unwrap();
    }

    fn on(&mut self) {
        self.set_low().unwrap();
    }
}
