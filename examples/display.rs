#![no_std]
#![no_main]

use panic_halt as _;

use riscv_rt::entry;
use gd32vf103xx_hal::prelude::*;
use gd32vf103xx_hal::pac;
use seedstudio_gd32v::lcd_pins;
use seedstudio_gd32v::lcd::{Lcd, ili9341::Orientation};
use embedded_graphics::prelude::*;
use embedded_graphics::fonts::{Font12x16, Text};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::style::{PrimitiveStyle, TextStyleBuilder};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let mut rcu = dp.RCU.configure()
        .ext_hf_clock(8.mhz())
        .sysclk(108.mhz())
        .freeze();

    let gpiod = dp.GPIOD.split(&mut rcu);
    let gpioe = dp.GPIOE.split(&mut rcu);

    let lcd_pins = lcd_pins!(gpiod, gpioe);
    let mut lcd = Lcd::new(dp.EXMC, lcd_pins, &mut rcu);

    lcd.set_orientation(Orientation::Landscape).unwrap();

    let p = Point::new(lcd.width() as i32 - 1, lcd.height() as i32 - 1);
    Rectangle::new(Point::new(0, 0), p)
        .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
        .draw(&mut *lcd).unwrap();

    let style = TextStyleBuilder::new(Font12x16)
        .background_color(Rgb565::GREEN)
        .text_color(Rgb565::WHITE)
        .build();
    Text::new(" Hello Rust! ", Point::new(40, 35))
        .into_styled(style)
        .draw(&mut *lcd).unwrap();

    loop { }
}
