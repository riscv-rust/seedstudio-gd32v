//! TSC2046 touch screen controller driver
use gd32vf103xx_hal::gpio::{Output, PushPull, Input, Floating};
use gd32vf103xx_hal::gpio::gpiod::PD13;
use gd32vf103xx_hal::gpio::gpioe::{PE0, PE2, PE3, PE4};
use embedded_hal::digital::v2::{OutputPin, InputPin};
use gd32vf103xx_hal::rcu::Rcu;
use gd32vf103xx_hal::delay::McycleDelay;
use embedded_hal::blocking::delay::DelayUs;
use core::convert::TryInto;
use ili9341::Orientation;
use embedded_graphics::prelude::Point;

/// Sets up all the needed GPIO pins for the LCD
///
/// ```
/// let gpiod = dp.GPIOD.split(&mut rcu);
/// let gpioe = dp.GPIOE.split(&mut rcu);
/// let touch_pins = touch_pins!(gpiod, gpioe);
/// ```
#[macro_export]
macro_rules! touch_pins {
    ($gpiod:ident, $gpioe:ident) => {{
        use gd32vf103xx_hal::gpio::State;
        $crate::touch::TouchPins {
            clk: $gpioe.pe0.into_push_pull_output_with_state(State::Low),
            cs: $gpiod.pd13.into_push_pull_output_with_state(State::High),
            mosi: $gpioe.pe2.into_push_pull_output_with_state(State::Low),
            miso: $gpioe.pe3.into_floating_input(),
            penirq: $gpioe.pe4.into_floating_input(),
        }
    }}
}

pub struct TouchPins {
    pub clk: PE0<Output<PushPull>>,
    pub cs: PD13<Output<PushPull>>,
    pub mosi: PE2<Output<PushPull>>,
    pub miso: PE3<Input<Floating>>,
    pub penirq: PE4<Input<Floating>>,
}

/// TSC2046 touch screen controller driver
pub struct TouchController {
    pins: TouchPins,
    delay: McycleDelay,
    orientation: Orientation,
    matrix: [i16; 6],
}

impl TouchController {
    pub fn new(pins: TouchPins, rcu: &Rcu) -> Self {
        TouchController {
            pins,
            delay: McycleDelay::new(&rcu.clocks),
            orientation: Orientation::Portrait,
            matrix: [-240, 0, 240, 0, 320, 0]
        }
    }

    /// Set screen orientation
    pub fn set_orientation(&mut self, orientation: Orientation) {
        self.orientation = orientation;
    }

    /// Set calibration matrix
    pub fn set_matrix(&mut self, matrix: [i16; 6]) {
        self.matrix = matrix;
    }

    #[inline(always)]
    fn delay(&mut self) {
        self.delay.delay_us(1);
    }

    fn transact(&mut self, bytes: &mut [u8]) {
        self.pins.cs.set_low().ok();
        self.delay();

        for byte in bytes {
            let mut b = *byte;
            let mut r = 0;
            for _ in 0..8 {
                if b & 0x80 != 0 {
                    self.pins.mosi.set_high().ok();
                } else {
                    self.pins.mosi.set_low().ok();
                }
                b = b << 1;

                self.delay();
                self.pins.clk.set_high().ok();

                r = r << 1;
                if self.pins.miso.is_high().unwrap() {
                    r |= 1;
                }
                self.delay();
                self.pins.clk.set_low().ok();
            }
            *byte = r;
        }
        self.delay();

        self.pins.cs.set_high().ok();
        self.delay();
    }

    /// Get raw measurements from the touch screen controller
    pub fn get_raw(&mut self) -> RawMeasurement {
        let mut buf = [0u8; 3*4];

        let mode = 0; // 12-bit
        let ser_dfr = 0; // differential
        let pd = 0b01; // ref off, adc on
        let command_base = 0x80 + (mode << 3) + (ser_dfr << 2) + pd;

        buf[0*3] = command_base | (0b101 << 4);
        buf[1*3] = command_base | (0b001 << 4);
        buf[2*3] = command_base | (0b011 << 4);
        buf[3*3] = command_base | (0b100 << 4);

        self.transact(&mut buf);

        let x  = u16::from_be_bytes(buf[1*3-2..1*3].try_into().unwrap()) >> 3;
        let y  = u16::from_be_bytes(buf[2*3-2..2*3].try_into().unwrap()) >> 3;
        let z1 = u16::from_be_bytes(buf[3*3-2..3*3].try_into().unwrap()) >> 3;
        let z2 = u16::from_be_bytes(buf[4*3-2..4*3].try_into().unwrap()) >> 3;

        RawMeasurement {
            x,
            y,
            z1,
            z2
        }
    }

    /// Get raw touch position from the touch screen controller
    ///
    /// Returns None if touch wasn't detected.
    /// Ignores calibration matrix and screen orientation.
    pub fn get_touch_raw(&mut self) -> Option<Point> {
        let m = self.get_raw();
        if m.z1 > 0x200 || m.z2 < 0xf00 {
            Some(Point {
                x: m.x as i32,
                y: m.y as i32,
            })
        } else {
            None
        }
    }

    /// Get touch position from the touch screen controller
    ///
    /// Returns None if touch wasn't detected.
    /// Takes into account calibration matrix and screen orientation.
    pub fn get_touch(&mut self) -> Option<Point> {
        let m = self.get_raw();
        if m.z1 > 0x200 || m.z2 < 0xf00 {
            let x = ((self.matrix[0] as i32) * (m.x as i32) +
                (self.matrix[1] as i32) * (m.y as i32)) / 4096 +
                (self.matrix[2] as i32);
            let y = ((self.matrix[3] as i32) * (m.x as i32) +
                (self.matrix[4] as i32) * (m.y as i32)) / 4096 +
                (self.matrix[5] as i32);

            let t = match self.orientation {
                Orientation::Portrait => Point { x, y },
                Orientation::PortraitFlipped => Point { x: 240 - x - 1, y: 320 - y - 1 },
                Orientation::Landscape => Point { x: y, y: x },
                Orientation::LandscapeFlipped => Point { x: 320 - y - 1, y: 240 - x - 1 },
            };
            Some(t)
        } else {
            None
        }
    }
}


pub struct RawMeasurement {
    pub x: u16,
    pub y: u16,
    pub z1: u16,
    pub z2: u16,
}

pub struct CalibrationPoint {
    pub sample: Point,
    pub reference: Point,
}

// Taken from https://www.analog.com/media/en/technical-documentation/application-notes/AN-1021.pdf
/// Calculate a calibration matrix from a set of calibration points
///
/// Function panics is the number of passed calibration points is less than three.
pub fn calculate_matrix(points: &[CalibrationPoint]) -> [i16; 6] {
    assert!(points.len() >= 3);

    let mut a = [0f32; 3];
    let mut b = [0f32; 3];
    let mut c = [0f32; 3];
    let mut d = [0f32; 3];
    if points.len() == 3 {
        for i in 0..3 {
            a[i] = points[i].sample.x as f32;
            b[i] = points[i].sample.y as f32;
            c[i] = points[i].reference.x as f32;
            d[i] = points[i].reference.y as f32;
        }
    } else {
        for p in points {
            a[2] = a[2] + (p.sample.x as f32);
            b[2] = b[2] + (p.sample.y as f32);
            c[2] = c[2] + (p.reference.x as f32);
            d[2] = d[2] + (p.reference.y as f32);
            a[0] = a[0] + (p.sample.x as f32)*(p.sample.x as f32);
            a[1] = a[1] + (p.sample.x as f32)*(p.sample.y as f32);
            b[0] = a[1];
            b[1] = b[1] + (p.sample.y as f32)*(p.sample.y as f32);
            c[0] = c[0] + (p.sample.x as f32)*(p.reference.x as f32);
            c[1] = c[1] + (p.sample.y as f32)*(p.reference.x as f32);
            d[0] = d[0] + (p.sample.x as f32)*(p.reference.y as f32);
            d[1] = d[1] + (p.sample.y as f32)*(p.reference.y as f32);
        }
        a[0] = a[0] / a[2];
        a[1] = a[1] / b[2];
        b[0] = b[0] / a[2];
        b[1] = b[1] / b[2];
        c[0] = c[0] / a[2];
        c[1] = c[1] / b[2];
        d[0] = d[0] / a[2];
        d[1] = d[1] / b[2];
        a[2] = a[2] / (points.len() as f32);
        b[2] = b[2] / (points.len() as f32);
        c[2] = c[2] / (points.len() as f32);
        d[2] = d[2] / (points.len() as f32);
    }
    let k = (a[0]-a[2])*(b[1]-b[2])-(a[1]-a[2])*(b[0]-b[2]);
    let kx1 = ((c[0]-c[2])*(b[1]-b[2])-(c[1]-c[2])*(b[0]-b[2]))/k;
    let kx2 = ((c[1]-c[2])*(a[0]-a[2])-(c[0]-c[2])*(a[1]-a[2]))/k;
    let kx3 = (b[0]*(a[2]*c[1]-a[1]*c[2])+b[1]*(a[0]*c[2]-a[2]*c[0])+b[2]*(a[1]*c[0]-a[0]*c[1]))/k;
    let ky1 = ((d[0]-d[2])*(b[1]-b[2])-(d[1]-d[2])*(b[0]-b[2]))/k;
    let ky2 = ((d[1]-d[2])*(a[0]-a[2])-(d[0]-d[2])*(a[1]-a[2]))/k;
    let ky3 = (b[0]*(a[2]*d[1]-a[1]*d[2])+b[1]*(a[0]*d[2]-a[2]*d[0])+b[2]*(a[1]*d[0]-a[0]*d[1]))/k;

    let kx1 = (kx1 * 4096.0) as i16;
    let kx2 = (kx2 * 4096.0) as i16;
    let kx3 = kx3 as i16;
    let ky1 = (ky1 * 4096.0) as i16;
    let ky2 = (ky2 * 4096.0) as i16;
    let ky3 = ky3 as i16;

    [kx1, kx2, kx3, ky1, ky2, ky3]
}
