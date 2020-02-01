//! Board support crate for the GD32 RISC-V Dev Board from SeeedStudio

#![no_std]

pub use gd32vf103xx_hal as hal;

#[cfg(feature = "lcd")]
pub mod lcd;
pub mod led;
pub mod stdout;
