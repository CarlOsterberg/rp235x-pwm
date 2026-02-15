#![no_std]
#![no_main]

use defmt_rtt as _;
use fugit::RateExtU32;
use panic_probe as _;
use rp235x_hal::{
    self as hal, Clock,
    clocks::init_clocks_and_plls,
    pwm::Slices,
    sio::Sio,
    uart::{DataBits, StopBits, UartConfig},
    watchdog::Watchdog,
};
use rtic_monotonics::rp235x::prelude::*;

rp235x_timer_monotonic!(Mono);

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// Program metadata for `picotool info`
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [rp235x_hal::binary_info::EntryAddr; 5] = [
    rp235x_hal::binary_info::rp_cargo_bin_name!(),
    rp235x_hal::binary_info::rp_cargo_version!(),
    rp235x_hal::binary_info::rp_program_description!(c"RP2350 RTIC hello world"),
    rp235x_hal::binary_info::rp_cargo_homepage_url!(),
    rp235x_hal::binary_info::rp_program_build_attribute!(),
];

#[rtic::app(device = rp235x_hal::pac, peripherals = true,
    dispatchers =[
            TIMER0_IRQ_1,
            TIMER1_IRQ_1
    ]
)]
mod app {
    use super::*;
    use cortex_m::prelude::{_embedded_hal_PwmPin};

    type Uart0 = hal::uart::UartPeripheral<
        hal::uart::Enabled,
        hal::pac::UART0,
        (
            hal::gpio::Pin<hal::gpio::bank0::Gpio0, hal::gpio::FunctionUart, hal::gpio::PullDown>,
            hal::gpio::Pin<hal::gpio::bank0::Gpio1, hal::gpio::FunctionUart, hal::gpio::PullDown>,
        ),
    >;

    type Pwm =
        hal::pwm::Channel<hal::pwm::Slice<hal::pwm::Pwm2, hal::pwm::FreeRunning>, hal::pwm::A>;

    #[shared]
    struct Shared {uart: Uart0,}

    #[local]
    struct Local {
        pwm2channel_a: Pwm,
        buffer:  [u8; 256],
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        // -------------------------- CLOCKS --------------------------
        Mono::start(ctx.device.TIMER0, &ctx.device.RESETS);
        // Configure the clocks, watchdog - The default is to generate a 125 MHz system clock
        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);

        // External high-speed crystal on the pico board is 12Mhz
        let external_xtal_freq_hz = 12_000_000u32;
        let clocks = init_clocks_and_plls(
            external_xtal_freq_hz,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut ctx.device.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let sio = Sio::new(ctx.device.SIO);

        // -------------------------- UART TX(gpio0/pin1) RX(gpio1/pin2) --------------------------
        let pins = hal::gpio::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut ctx.device.RESETS,
        );

        let uart_pins = (
            pins.gpio0.into_function::<hal::gpio::FunctionUart>(),
            pins.gpio1.into_function::<hal::gpio::FunctionUart>(),
        );

        let baudrate: u32 = 115_200;

        let mut uart =
            hal::uart::UartPeripheral::new(ctx.device.UART0, uart_pins, &mut ctx.device.RESETS)
                .enable(
                    UartConfig::new(baudrate.Hz(), DataBits::Eight, None, StopBits::One),
                    clocks.peripheral_clock.freq(),
                )
                .unwrap();
        uart.enable_rx_interrupt();

        // -------------------------- PWM2 chA gpio4/pin6 --------------------------
        let pwm_slices = Slices::new(ctx.device.PWM, &mut ctx.device.RESETS);

        let funcPwm = pins.gpio4.into_function::<hal::gpio::FunctionPwm>();
        let mut pwm = pwm_slices.pwm2;
        pwm.set_ph_correct();
        pwm.set_div_int(35);
        pwm.set_top(20000);
        pwm.enable();
        let mut channel_a = pwm.channel_a;
        channel_a.output_to(funcPwm);
        // Set to minimum throttle for Brushless ESC, 30A XT60 Electronic speed regulator,
        // this will make it calibrate to be ready for use.
        channel_a.set_duty(20000 / 10);

        let buffer: [u8; 256] = [0; 256];

        uart.write_full_blocking(b"Write something and I will echo it back.\r\n");

        hello::spawn().ok();
        esc::spawn().ok();

        (
            Shared {uart},
            Local {
                pwm2channel_a: channel_a,
                buffer
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = UART0_IRQ, priority = 1, shared = [uart], local = [buffer])]
    fn uart_rx_int(mut ctx: uart_rx_int::Context) {
        ctx.shared.uart.lock(|uart| {
        uart.write_full_blocking(b"Int Trig.\r\n");
        let res = uart.read_raw(ctx.local.buffer);
        match res {
            Ok(_chars) => {
                uart.write_full_blocking(ctx.local.buffer);
            },
            Err(_) => {
                uart.write_full_blocking(b"Read Error\r\n");
            }
        }
        });
    }

    #[task(shared = [uart], priority = 2)]
    async fn hello(mut _ctx: hello::Context) {
        let mut _buffer: [u8; 256] = [0; 256];
        loop {
            Mono::delay(1000.millis()).await;
            // ctx.shared.uart.lock(|uart| {
            //     uart.write_full_blocking(b"Trying to read.\r\n");
            //     let res = uart.read_full_blocking(&mut buffer);
            //     match res {
            //         Ok(_) => {
            //             uart.write_full_blocking(& buffer);
            //         }
            //         Err(rp235x_hal::uart::ReadErrorType::Overrun) => {
            //             uart.write_full_blocking(b"ReadErrorType::Overrun\r\n");
            //         }
            //         Err(rp235x_hal::uart::ReadErrorType::Break) => {
            //             uart.write_full_blocking(b"ReadErrorType::Break\r\n");
            //         }
            //         Err(rp235x_hal::uart::ReadErrorType::Parity) => {
            //             uart.write_full_blocking(b"ReadErrorType::Parity\r\n");
            //         }
            //         Err(rp235x_hal::uart::ReadErrorType::Framing) => {
            //             uart.write_full_blocking(b"ReadErrorType::Framing\r\n");
            //         }
            //     }
            // });
        }
    }

    #[task(local = [pwm2channel_a], priority = 1)]
    async fn esc(_ctx: esc::Context) {
        // ctx.local.pwm2channel_a.set_duty(19_999 / 20);
        crate::Mono::delay(1000.millis()).await;
        // ctx.local.pwm2channel_a.set_duty(19_999 / 10);
        // crate::Mono::delay(5.millis()).await;
    }
}

// End of file
