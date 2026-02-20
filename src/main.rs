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
    use cortex_m::prelude::{_embedded_hal_PwmPin};
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
    type PwmPin6 =
        hal::pwm::Channel<hal::pwm::Slice<hal::pwm::Pwm2, hal::pwm::FreeRunning>, hal::pwm::A>;

    type PwmPin11 =
    hal::pwm::Channel<hal::pwm::Slice<hal::pwm::Pwm4, hal::pwm::FreeRunning>, hal::pwm::A>;

    type PwmPin16 =
    hal::pwm::Channel<hal::pwm::Slice<hal::pwm::Pwm6, hal::pwm::FreeRunning>, hal::pwm::A>;

    type PwmPin21 =
    hal::pwm::Channel<hal::pwm::Slice<hal::pwm::Pwm0, hal::pwm::FreeRunning>, hal::pwm::A>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        pwm_pin_6: PwmPin6,
        pwm_pin_11: PwmPin11,
        pwm_pin_16: PwmPin16,
        pwm_pin_21: PwmPin21,
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

        let pins = hal::gpio::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut ctx.device.RESETS,
        );

        let pwm_slices = Slices::new(ctx.device.PWM, &mut ctx.device.RESETS);

        // -------------------------- UART TX(gpio0/pin1) RX(gpio1/pin2) --------------------------
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
        let pin6 = pins.gpio4.into_function::<gpio::FunctionPwm>();
        let mut pwm2 = pwm_slices.pwm2;
        pwm2.set_ph_correct();
        pwm2.set_div_int(35);
        pwm2.set_top(PWM_TOP);
        pwm2.enable();
        let mut pwm_pin_6 = pwm2.channel_a;
        pwm_pin_6.output_to(pin6);
        // Set to minimum throttle for Brushless ESC, 30A XT60 Electronic speed regulator,
        // this will make it calibrate to be ready for use.
        pwm_pin_6.set_duty(PWM_TOP / 10);

        // -------------------------- PWM4 chA gpio8/pin11 --------------------------
        let pin11 = pins.gpio8.into_function::<gpio::FunctionPwm>();
        let mut pwm4 = pwm_slices.pwm4;
        pwm4.set_ph_correct();
        pwm4.set_div_int(35);
        pwm4.set_top(PWM_TOP);
        pwm4.enable();
        let mut pwm_pin_11 = pwm4.channel_a;
        pwm_pin_11.output_to(pin11);
        // Set to minimum throttle for Brushless ESC, 30A XT60 Electronic speed regulator,
        // this will make it calibrate to be ready for use.
        pwm_pin_11.set_duty(PWM_TOP / 10);

        // -------------------------- PWM6 chA gpio12/pin16 --------------------------
        let pin16 = pins.gpio12.into_function::<gpio::FunctionPwm>();
        let mut pwm6 = pwm_slices.pwm6;
        pwm6.set_ph_correct();
        pwm6.set_div_int(35);
        pwm6.set_top(PWM_TOP);
        pwm6.enable();
        let mut pwm_pin_16 = pwm6.channel_a;
        pwm_pin_16.output_to(pin16);
        // Set to minimum throttle for Brushless ESC, 30A XT60 Electronic speed regulator,
        // this will make it calibrate to be ready for use.
        pwm_pin_16.set_duty(PWM_TOP / 10);

        // -------------------------- PWM0 chA gpio16/pin21 --------------------------
        let pin21 = pins.gpio16.into_function::<gpio::FunctionPwm>();
        let mut pwm0 = pwm_slices.pwm0;
        pwm0.set_ph_correct();
        pwm0.set_div_int(35);
        pwm0.set_top(PWM_TOP);
        pwm0.enable();
        let mut pwm_pin_21 = pwm0.channel_a;
        pwm_pin_21.output_to(pin21);
        // Set to minimum throttle for Brushless ESC, 30A XT60 Electronic speed regulator,
        // this will make it calibrate to be ready for use.
        pwm_pin_21.set_duty(PWM_TOP / 10);

        // -------------------------- Message Queue --------------------------
        let (s, r) = make_channel!(u8, MSG_Q_CAPACITY);

        let buffer: [u8; UART_READER_CAPACITY] = [0; UART_READER_CAPACITY];

        uart_tx.write_full_blocking(
            b"Write any number 0-5 to adjust all 4 pwms. 0 is no throttle and 5 is maximum.\r\n",
        );

        esc::spawn().ok();

        (
            Shared {},
            Local {
                pwm_pin_6,
                pwm_pin_11,
                pwm_pin_16,
                pwm_pin_21,
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

    #[task(
        local = [
            uart_tx,
            msg_q_receiver,
            pwm_pin_6,
            pwm_pin_11,
            pwm_pin_16,
            pwm_pin_21,
        ],
        priority = 1
    )]
    async fn esc(ctx: esc::Context) {
        let mut buffer: [u8;1] = [0];
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
                        ctx.local.pwm_pin_6.set_duty(PWM_TOP / divisor);
                        ctx.local.pwm_pin_11.set_duty(PWM_TOP / divisor);
                        ctx.local.pwm_pin_16.set_duty(PWM_TOP / divisor);
                        ctx.local.pwm_pin_21.set_duty(PWM_TOP / divisor);

                    } else {
                        buffer[0] = byte;
                        ctx.local.uart_tx.write_full_blocking(b"Unhandled data: ");
                        ctx.local.uart_tx.write_full_blocking(&buffer);
                        ctx.local.uart_tx.write_full_blocking(b"\r\n");
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
