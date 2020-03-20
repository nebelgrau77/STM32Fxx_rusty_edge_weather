//! Edge computing weather station - WORK IN PROGRESS
//! 
//! Platform: STM32F411 ("black pill" board)
//! 
//! The idea is to have temperature and humidity read from a sensor
//! and based on that classify the weather as good, bad or so-so.
//! 
//! The equations for decision boundaries between these classes were calculated
//! using logistic regression from the scikit.learn ML package
//! 
//! Dataset is purely synthetic, labelled according to some simple rules with mild randomization
//! 
//! At the moment readings from the sensor are simulated with trimpots read by the ADC
//! 
//! Best results when using `--release`.

#![no_std]
#![no_main]

// import all the necessary crates and components

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate stm32f4xx_hal as hal;
extern crate stm32f4;
extern crate panic_halt;

use cortex_m_rt::entry;
use cortex_m::interrupt::{Mutex, free};

use core::fmt;
use core::fmt::Write;
use arrayvec::ArrayString;

use core::ops::DerefMut;
use core::cell::{Cell, RefCell};

use stm32f4::stm32f411::interrupt;

use ssd1306::{prelude::*, Builder as SSD1306Builder};

use crate::hal::{
    prelude::*,
    gpio::{gpioa::{PA3, PA4}, Edge, ExtiPin, Input, PullUp, Analog},
    i2c::I2c,
    stm32,
    timer::{Timer, Event},
    delay::Delay,
    time::Hertz,
    stm32::{Interrupt,EXTI},
    adc::{Adc, config::{AdcConfig, SampleTime, Clock, Resolution}}
};

use embedded_graphics::{
    fonts::{Font12x16, Text},
    pixelcolor::BinaryColor,
    prelude::*,
    style::TextStyleBuilder,
    };

// create two globally accessible values
static TEMP: Mutex<Cell<u32>> = Mutex::new(Cell::new(0u32));
static HUM: Mutex<Cell<u32>> = Mutex::new(Cell::new(0u32));

// interrupt and peripheral for ADC
static TIMER_TIM3: Mutex<RefCell<Option<Timer<stm32::TIM3>>>> = Mutex::new(RefCell::new(None));

static GADC: Mutex<RefCell<Option<Adc<stm32::ADC1>>>> = Mutex::new(RefCell::new(None));

static ANALOG1: Mutex<RefCell<Option<PA3<Analog>>>> = Mutex::new(RefCell::new(None));
static ANALOG2: Mutex<RefCell<Option<PA4<Analog>>>> = Mutex::new(RefCell::new(None));

static UPPER: [f32; 3] = [-0.34641723,  0.16089022, -3.66223611]; //W0, W1, b; values above this line are BAD 
static LOWER: [f32; 3] = [0.34539674, -0.42790601, 3.90048735]; //W0, W1, b; values below this line are GOOD

const BOOT_DELAY_MS: u16 = 100; 

#[entry]
fn main() -> ! {
    if let (Some(mut dp), Some(cp)) = (
        stm32::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        
        // necessary to enable this for the external interrupt to work
        // dp.RCC.apb2enr.write(|w| w.syscfgen().enabled()); 

        // set up clocks
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(25.mhz()).sysclk(100.mhz()).pclk2(25.mhz()).freeze();
        
        //delay necessary for the I2C to initiate correctly and start on boot without having to reset the board
        let mut delay = Delay::new(cp.SYST, clocks);
        delay.delay_ms(BOOT_DELAY_MS);

        // Set up I2C - SCL is PB8 and SDA is PB9; they are set to Alternate Function 4, open drain
        let gpiob = dp.GPIOB.split();
        let scl = gpiob.pb8.into_alternate_af4().set_open_drain();
        let sda = gpiob.pb9.into_alternate_af4().set_open_drain();
        let i2c = I2c::i2c1(dp.I2C1, (scl, sda), 400.khz(), clocks);

        let gpioa = dp.GPIOA.split();

        // Set up ADC
        let adcconfig = AdcConfig::default().clock(Clock::Pclk2_div_8).resolution(Resolution::Twelve);
        let adc = Adc::adc1(dp.ADC1, true, adcconfig);
                
        let pa3 = gpioa.pa3.into_analog();
        let pa4 = gpioa.pa4.into_analog();

        // move the PA3 pin and the ADC into the 'global storage'

        free(|cs| {
            *GADC.borrow(cs).borrow_mut() = Some(adc);
        
            *ANALOG1.borrow(cs).borrow_mut() = Some(pa3);
            *ANALOG2.borrow(cs).borrow_mut() = Some(pa4);
        });

        // Set up the display
        let mut disp: GraphicsMode<_> = SSD1306Builder::new().connect_i2c(i2c).into();
        disp.init().unwrap();
        
        // set up timers and external interrupt
        let mut adctimer = Timer::tim3(dp.TIM3, Hertz(10), clocks); //adc update every 100ms
        adctimer.listen(Event::TimeOut);
                
        free(|cs| {
            TIMER_TIM3.borrow(cs).replace(Some(adctimer));
            });

        let mut nvic = cp.NVIC;
            unsafe {            
                nvic.set_priority(Interrupt::TIM3, 1);
                cortex_m::peripheral::NVIC::unmask(Interrupt::TIM3);
            }
                        
            cortex_m::peripheral::NVIC::unpend(Interrupt::TIM3);
            
        
        loop {
           
            for x in 0..128 {
                for y in 0..50 {
                    disp.set_pixel(x,y,0);
                }
            }
            
            let text_style = TextStyleBuilder::new(Font12x16).text_color(BinaryColor::On).build();
            let mut buf_temp = ArrayString::<[u8; 7]>::new();
            let mut buf_hum = ArrayString::<[u8; 7]>::new();
            let mut text_status = ArrayString::<[u8; 16]>::new();
            
            // get the values from the global variables
            let temperature = free(|cs| TEMP.borrow(cs).get()); 
            let humidity = free(|cs| HUM.borrow(cs).get()); 

            let h_sucks = -(temperature as f32 * UPPER[0] + UPPER[2]) / UPPER[1];
            let h_nice = -(temperature as f32 * LOWER[0] + LOWER[2]) / LOWER[1];

            /*
            // format the current values and write them on the display
            format(&mut text_buf1, temperature);
            Text::new(text_buf1.as_str(), Point::new(0, 0)).into_styled(text_style).draw(&mut disp);
            format(&mut text_buf2, humidity);
            Text::new(text_buf2.as_str(), Point::new(0, 16)).into_styled(text_style).draw(&mut disp);
            */

            // format and send the buffer to the display
            // ASCII codes: C: 67, H: 72, T: 84, %: 37
            
            format(&mut buf_temp, temperature as u8, 84 as char, 67 as char);

            Text::new(buf_temp.as_str(), Point::new(0, 0)).into_styled(text_style).draw(&mut disp);

            format(&mut buf_hum, humidity as u8, 72 as char, 37 as char);
        
            Text::new(buf_hum.as_str(), Point::new(0, 16)).into_styled(text_style).draw(&mut disp);

            let mut status: u8 = 1;
            
            if humidity >= h_sucks as u32 {
                status = 0 }
            else if humidity <= h_nice as u32 {
                status = 2 }
            else {
                status = 1
            }
            
            status_display(&mut text_status, status);
            Text::new(text_status.as_str(), Point::new(0, 32)).into_styled(text_style).draw(&mut disp);

            disp.flush().unwrap();

            delay.delay_ms(50_u16);

            }

        }
       
    loop {}
}



#[interrupt]

// the ADCVALs get updated every time the interrupt fires 
// read from ADC on pins PA3 and PA4

fn TIM3() {
        
    free(|cs| {
        stm32::NVIC::unpend(Interrupt::TIM3);
        if let (Some(ref mut tim3), Some(ref mut adc), Some(ref mut analog1), Some(ref mut analog2)) = (
        TIMER_TIM3.borrow(cs).borrow_mut().deref_mut(),
        GADC.borrow(cs).borrow_mut().deref_mut(),
        
        ANALOG1.borrow(cs).borrow_mut().deref_mut(),
        ANALOG2.borrow(cs).borrow_mut().deref_mut())
        {
            tim3.clear_interrupt(Event::TimeOut);
            let sample1 = adc.convert(analog1, SampleTime::Cycles_480);
            let mut temp: u32 = sample1 as u32 * 30;
            temp = temp / 4096;
            TEMP.borrow(cs).replace(temp);

            let sample2 = adc.convert(analog2, SampleTime::Cycles_480);
            let mut hum: u32 = sample2 as u32 * 100;
            hum = hum / 4096;
            HUM.borrow(cs).replace(hum);
        }
    });
}

// helper function to display "temperature" and "humidity"

*/
fn format(buf: &mut ArrayString<[u8; 7]>, val: u8, feature: char, unit: char) {
    fmt::write(buf, format_args!("{}: {:02} {}", 
    feature, val, unit)).unwrap();
}

// helper function to display the weather status

fn status_display(buf: &mut ArrayString<[u8;16]>, status: u8) {
    if status == 2 {
        fmt::write(buf, format_args!("IT'S NICE! :)   ")).unwrap();
    }
    else if status == 0 {
        fmt::write(buf, format_args!("BUMMER :(       ")).unwrap();
    } 
    else {
        fmt::write(buf, format_args!("IT'S OK.        ")).unwrap();
    }
}