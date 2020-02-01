use gd32vf103xx_hal::exmc::{ExmcConfiguration, ExmcTimingConfiguration, NwaitPolarity, DataBusWidth, MemoryType, ExmcPins, ExmcExt, Exmc};
use gd32vf103xx_hal::rcu::Rcu;
use gd32vf103xx_hal::pac::EXMC;
use gd32vf103xx_hal::gpio::{Output, PushPull, gpioe::PE1, gpiod::PD12};
use ili9341::{Interface, Ili9341};
use core::convert::Infallible;
use gd32vf103xx_hal::delay::McycleDelay;
use core::ops::{Deref, DerefMut};

pub use ili9341;

/// Sets up all the needed GPIO pins for the LCD
///
/// ```
/// let gpiod = dp.GPIOD.split(&mut rcu);
/// let gpioe = dp.GPIOE.split(&mut rcu);
/// let lcd_pins = lcd_pins!(gpiod, gpioe);
/// ```
#[macro_export]
macro_rules! lcd_pins {
    ($gpiod:ident, $gpioe:ident) => {{
        let data = $crate::hal::exmc::ExmcPins {
            d0: $gpiod.pd14.into_alternate_push_pull(),
            d1: $gpiod.pd15.into_alternate_push_pull(),
            d2: $gpiod.pd0.into_alternate_push_pull(),
            d3: $gpiod.pd1.into_alternate_push_pull(),
            d4: $gpioe.pe7.into_alternate_push_pull(),
            d5: $gpioe.pe8.into_alternate_push_pull(),
            d6: $gpioe.pe9.into_alternate_push_pull(),
            d7: $gpioe.pe10.into_alternate_push_pull(),
            d8: Some($gpioe.pe11.into_alternate_push_pull()),
            d9: Some($gpioe.pe12.into_alternate_push_pull()),
            d10: Some($gpioe.pe13.into_alternate_push_pull()),
            d11: Some($gpioe.pe14.into_alternate_push_pull()),
            d12: Some($gpioe.pe15.into_alternate_push_pull()),
            d13: Some($gpiod.pd8.into_alternate_push_pull()),
            d14: Some($gpiod.pd9.into_alternate_push_pull()),
            d15: Some($gpiod.pd10.into_alternate_push_pull()),
            a16: Some($gpiod.pd11.into_alternate_push_pull()),
            a17: None,
            a18: None,
            a19: None,
            a20: None,
            a21: None,
            a22: None,
            a23: None,
            noe: Some($gpiod.pd4.into_alternate_push_pull()),
            nwe: Some($gpiod.pd5.into_alternate_push_pull()),
            nwait: None,
            ne0: Some($gpiod.pd7.into_alternate_push_pull()),
            nbl0: None,
            nbl1: None
        };
        $crate::lcd::LcdPins {
            data,
            rst: $gpioe.pe1.into_push_pull_output(),
            bl: $gpiod.pd12.into_push_pull_output(),
        }
    }}
}

pub struct LcdInterface {
    exmc: Exmc
}

impl LcdInterface {
    #[inline(always)]
    fn command(&mut self, v: u8) {
        self.exmc.as_u16_slice()[0].set(v as u16)
    }

    #[inline(always)]
    fn data(&mut self, v: u16) {
        let mem = self.exmc.as_u16_slice();
        mem[mem.len() - 1].set(v);
    }
}

impl Interface for LcdInterface {
    type Error = ili9341::Error<Infallible, Infallible>;

    fn write(&mut self, command: u8, data: &[u8]) -> Result<(), Self::Error> {
        self.command(command);
        for b in data {
            self.data(*b as u16);
        }
        Ok(())
    }

    fn write_iter(&mut self, command: u8, data: impl IntoIterator<Item=u16>) -> Result<(), Self::Error> {
        self.command(command);
        for b in data.into_iter() {
            self.data(b);
        }
        Ok(())
    }
}

pub struct LcdPins {
    pub data: ExmcPins,
    pub rst: PE1<Output<PushPull>>,
    pub bl: PD12<Output<PushPull>>,
}

pub struct Lcd {
    driver: Ili9341<LcdInterface, PE1<Output<PushPull>>>
}

impl Lcd {
    pub fn new(exmc: EXMC, pins: LcdPins, rcu: &mut Rcu) -> Lcd {
        let conf = ExmcConfiguration {
            async_wait_enabled: false,
            nwait_signal_enabled: false,
            memory_write_enabled: true,
            nwait_polarity: NwaitPolarity::ActiveLow,
            databus_width: DataBusWidth::Width16Bits,
            memory_type: MemoryType::SRAM,
            address_data_mux_enabled: true
        };
        let mut timing = ExmcTimingConfiguration::default();
        timing.bus_latency(1).data_setup_time(10).address_hold_time(2).address_setup_time(5);
        let exmc = exmc.configure(pins.data, conf, timing, rcu);

        // Turn on backlight
        let mut bl = pins.bl;
        use embedded_hal::digital::v2::OutputPin;
        bl.set_low().unwrap();

        let lcd_interface = LcdInterface {
            exmc
        };

        let mut delay = McycleDelay::new(&rcu.clocks);
        let driver = Ili9341::new(lcd_interface, pins.rst, &mut delay).unwrap();

        Lcd {
            driver,
        }
    }
}

impl Deref for Lcd {
    type Target = Ili9341<LcdInterface, PE1<Output<PushPull>>>;

    fn deref(&self) -> &Self::Target {
        &self.driver
    }
}

impl DerefMut for Lcd {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.driver
    }
}
