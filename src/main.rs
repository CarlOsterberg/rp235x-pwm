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
            TIMER0_IRQ_1
    ]
)]
mod app {
    use super::*;
    use cortex_m::prelude::{_embedded_hal_PwmPin, _embedded_hal_serial_Write};
    use rp235x_hal::{gpio, uart};
    use rtic_sync::{channel::*, make_channel};

    type UartRx = uart::Reader<
        hal::pac::UART0,
        (
            gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionUart, gpio::PullDown>,
            gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionUart, gpio::PullDown>,
        ),
    >;

    type UartTx = uart::Writer<
        hal::pac::UART0,
        (
            gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionUart, gpio::PullDown>,
            gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionUart, gpio::PullDown>,
        ),
    >;
    const PWM_TOP: u16 = 20000;
    const UART_READER_CAPACITY: usize = 32;
    const MSG_Q_CAPACITY: usize = 32;
    type Pwm =
        hal::pwm::Channel<hal::pwm::Slice<hal::pwm::Pwm2, hal::pwm::FreeRunning>, hal::pwm::A>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        pwm2channel_a: Pwm,
        buffer: [u8; UART_READER_CAPACITY],
        msg_q_sender: Sender<'static, u8, MSG_Q_CAPACITY>,
        msg_q_receiver: Receiver<'static, u8, MSG_Q_CAPACITY>,
        uart_tx: UartTx,
        uart_rx: UartRx,
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
            pins.gpio0.into_function::<gpio::FunctionUart>(),
            pins.gpio1.into_function::<gpio::FunctionUart>(),
        );

        let baudrate: u32 = 115_200;

        let mut uart =
            uart::UartPeripheral::new(ctx.device.UART0, uart_pins, &mut ctx.device.RESETS)
                .enable(
                    UartConfig::new(baudrate.Hz(), DataBits::Eight, None, StopBits::One),
                    clocks.peripheral_clock.freq(),
                )
                .unwrap();
        uart.enable_rx_interrupt();
        let (uart_rx, uart_tx) = uart.split();

        // -------------------------- PWM2 chA gpio4/pin6 --------------------------
        let pwm_slices = Slices::new(ctx.device.PWM, &mut ctx.device.RESETS);

        let funcPwm = pins.gpio4.into_function::<gpio::FunctionPwm>();
        let mut pwm = pwm_slices.pwm2;
        pwm.set_ph_correct();
        pwm.set_div_int(35);
        pwm.set_top(PWM_TOP);
        pwm.enable();
        let mut channel_a = pwm.channel_a;
        channel_a.output_to(funcPwm);
        // Set to minimum throttle for Brushless ESC, 30A XT60 Electronic speed regulator,
        // this will make it calibrate to be ready for use.
        channel_a.set_duty(PWM_TOP / 10);

        // -------------------------- Message Queue --------------------------
        let (s, r) = make_channel!(u8, MSG_Q_CAPACITY);

        let buffer: [u8; UART_READER_CAPACITY] = [0; UART_READER_CAPACITY];

        uart_tx.write_full_blocking(
            b"Write any number 0-5 to adjust pwm. 0 is no throttle and 5 is maximum.\r\n",
        );

        esc::spawn().ok();

        (
            Shared {},
            Local {
                pwm2channel_a: channel_a,
                buffer,
                msg_q_sender: s,
                msg_q_receiver: r,
                uart_tx,
                uart_rx,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = UART0_IRQ, priority = 2, local = [uart_rx, buffer, msg_q_sender])]
    fn uart_rx_int(ctx: uart_rx_int::Context) {
        let res = ctx.local.uart_rx.read_raw(ctx.local.buffer);
        match res {
            Ok(chars) => {
                    for i in 0..chars {
                        ctx.local.msg_q_sender.try_send(ctx.local.buffer[i]).unwrap();
                    }
                }
            _ => {}
        }
    }

    #[task(local = [uart_tx, msg_q_receiver, pwm2channel_a], priority = 1)]
    async fn esc(ctx: esc::Context) {
        loop {
            match ctx.local.msg_q_receiver.recv().await {
                Ok(byte) => {
                    // Only accept input values 0-5
                    if matches!(byte, 48 | 49 | 50 | 51 | 52 | 53) {
                        // Flip the value so that 0 is minimum and 5 maximum throttle.
                        let divisor: u16 = (10 - (byte - 48)) as u16;
                        ctx.local
                            .uart_tx
                            .write_full_blocking(b"Updating PWM...\r\n");
                        ctx.local.pwm2channel_a.set_duty(PWM_TOP / divisor);
                    } else {
                        ctx.local.uart_tx.write_full_blocking(b"Unhandled data: ");
                        let res = ctx.local.uart_tx.write(byte);
                        if res.is_err()
                        {
                            ctx.local.uart_tx.write_full_blocking(b"Failed transmitt\r\n");
                        }
                        else
                        {
                            ctx.local.uart_tx.write_full_blocking(b"\r\n");
                        }
                    }
                }
                Err(_) => {
                    ctx.local.uart_tx.write_full_blocking(b"Msg Q error.\r\n");
                }
            }
        }
    }
}

// End of file
