#![no_std]
#![no_main]

use panic_halt as _;

use riscv_rt::entry;
use gd32vf103xx_hal as hal;
use hal::pac as pac;
use gd32vf103xx_hal::gpio::GpioExt;
use seedstudio_gd32v::led::Led;
use gd32vf103xx_hal::rcu::RcuExt;
use gd32vf103xx_hal::delay::McycleDelay;
use embedded_hal::blocking::delay::DelayMs;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let mut rcu = dp.RCU.configure().freeze();

    let gpiob = dp.GPIOB.split(&mut rcu);

    let mut led1 = gpiob.pb5.into_push_pull_output();
    led1.off();
    let mut led2 = gpiob.pb1.into_push_pull_output();
    led2.off();
    let mut led3 = gpiob.pb0.into_push_pull_output();
    led3.off();

    let leds: [&mut dyn Led; 3] = [&mut led1, &mut led2, &mut led3];

    let mut delay = McycleDelay::new(&rcu.clocks);

    let mut i = 0;
    loop {
        let inext = (i + 1) % leds.len();
        leds[i].off();
        leds[inext].on();
        delay.delay_ms(500);

        i = inext;
    }
}
