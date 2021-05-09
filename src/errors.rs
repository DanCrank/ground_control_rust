error_chain! {
    errors {
        DisplayError(t: String) {
            description("display error")
            display("display error: '{}'", t)
        }
        RadioError(t: String) {
            description("radio error")
            display("radio error: '{}'", t)
        }
        SendError(t: String) {
            description("send protocol error")
            display("send protocol error: '{}'", t)
        }
        ReceiveError(t: String) {
            description("receive protocol error")
            display("receive protocol error: '{}'", t)
        }
    }
    foreign_links {
        RppalGpio(::rppal::gpio::Error);
        RppalI2c(::rppal::i2c::Error);
        RppalSpi(::rppal::spi::Error);
        //Ssd1306 does not impl Display on its error types, so we can't include it here
        //bad, bad Ssd1306
        //Ssd1306(::ssd1306::mode::terminal::TerminalModeError);
    }
}
