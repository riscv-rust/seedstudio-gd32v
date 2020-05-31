#![no_std]
#![no_main]

use panic_halt as _;

use riscv_rt::entry;
use gd32vf103xx_hal::prelude::*;
use gd32vf103xx_hal::pac;
use seedstudio_gd32v::{lcd_pins, touch_pins, sprintln};
use seedstudio_gd32v::lcd::{Lcd, ili9341::Orientation};
use embedded_graphics::prelude::*;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::primitives::{Rectangle, Circle};
use embedded_graphics::style::{PrimitiveStyle, PrimitiveStyleBuilder};
use seedstudio_gd32v::touch::{TouchController, CalibrationPoint, calculate_matrix};
use gd32vf103xx_hal::delay::McycleDelay;
use core::ops::DerefMut;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let mut rcu = dp.RCU.configure()
        .ext_hf_clock(8.mhz())
        .sysclk(108.mhz())
        .freeze();

    let mut afio = dp.AFIO.constrain(&mut rcu);

    let gpioa = dp.GPIOA.split(&mut rcu);
    seedstudio_gd32v::stdout::configure(dp.USART0, gpioa.pa9, gpioa.pa10, 115_200.bps(), &mut afio, &mut rcu);

    let gpiod = dp.GPIOD.split(&mut rcu);
    let gpioe = dp.GPIOE.split(&mut rcu);

    let lcd_pins = lcd_pins!(gpiod, gpioe);
    let mut lcd = Lcd::new(dp.EXMC, lcd_pins, &mut rcu);

    let touch_pins = touch_pins!(gpiod, gpioe);
    let mut touch = TouchController::new(touch_pins, &mut rcu);

    lcd.set_orientation(Orientation::Portrait).unwrap();
    touch.set_orientation(Orientation::Portrait);

    let p = Point::new(lcd.width() as i32 - 1, lcd.height() as i32 - 1);
    Rectangle::new(Point::new(0, 0), p)
        .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
        .draw(&mut *lcd).unwrap();

    let mut delay = McycleDelay::new(&rcu.clocks);

    //touch.set_matrix([-279, 2, 254, 0, 373, -21]);
    calibrate(&mut lcd, &mut touch, &mut delay);

    let circle_style = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb565::RED)
        .stroke_width(1)
        .fill_color(Rgb565::BLACK)
        .build();

    let mut i = 0;
    loop {
        let m = touch.get_raw();
        let t = touch.get_touch();
        sprintln!("{:5} {:04x} {:04x} {:04x} {:04x} {:?}", i, m.x, m.y, m.z1, m.z2, t);
        i += 1;

        if let Some(t) = t {
            Circle::new(t, 5)
                .into_styled(circle_style.clone())
                .draw(&mut *lcd).unwrap();
        }

        delay.delay_ms(100);
    }
}

pub fn calibrate(lcd: &mut Lcd, touch: &mut TouchController, delay: &mut McycleDelay) {
    lcd.set_orientation(Orientation::Portrait).unwrap();
    touch.set_orientation(Orientation::Portrait);

    let mut points = [
        CalibrationPoint {
            sample: Point { x: 30, y: 30},
            reference: Point { x: 30, y: 30},
        },
        CalibrationPoint {
            sample: Point { x: 30, y: 290},
            reference: Point { x: 30, y: 290},
        },
        CalibrationPoint {
            sample: Point { x: 210, y: 290},
            reference: Point { x: 210, y: 290},
        },
        CalibrationPoint {
            sample: Point { x: 210, y: 30},
            reference: Point { x: 210, y: 30},
        },
    ];

    let circle_style = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb565::RED)
        .stroke_width(1)
        .fill_color(Rgb565::BLACK)
        .build();
    let black_circle_style = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb565::BLACK)
        .stroke_width(1)
        .fill_color(Rgb565::BLACK)
        .build();

    for p in &mut points {
        Circle::new(p.reference, 5)
            .into_styled(circle_style.clone())
            .draw(lcd.deref_mut()).unwrap();

        let t = loop {
            if let Some(t) = touch.get_touch_raw() {
                break t;
            }
            delay.delay_ms(100);
        };
        p.sample = t;

        Circle::new(p.reference, 5)
            .into_styled(black_circle_style.clone())
            .draw(lcd.deref_mut()).unwrap();

        while touch.get_touch_raw().is_some() { delay.delay_ms(100); }
    }
    let m = calculate_matrix(&points);

    sprintln!("Matrix: {:?}", m);
    touch.set_matrix(m);
}
