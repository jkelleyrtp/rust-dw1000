use crate::{configs::TxContinuation, time::Instant, Error, Ready, RxConfig, Sending, DW1000};
use embedded_hal::spi::SpiDevice;
use nb;

use super::{AutoDoubleBufferReceiving, SingleBufferReceiving};

impl<SPI> DW1000<SPI, Sending>
where
    SPI: SpiDevice,
{
    /// Wait for the transmission to finish
    ///
    /// This method returns an `nb::Result` to indicate whether the transmission
    /// has finished, or whether it is still ongoing. You can use this to busily
    /// wait for the transmission to finish, for example using `nb`'s `block!`
    /// macro, or you can use it in tandem with [`DW1000::enable_tx_interrupts`]
    /// and the DW1000 IRQ output to wait in a more energy-efficient manner.
    ///
    /// Handling the DW1000's IRQ output line is out of the scope of this
    /// driver, but please note that if you're using the DWM1001 module or
    /// DWM1001-Dev board, that the `dwm1001` crate has explicit support for
    /// this.
    pub fn wait_transmit(&mut self) -> nb::Result<Instant, Error<SPI>> {
        // Check Half Period Warning Counter. If this is a delayed transmission,
        // this will indicate that the delay was too short, and the frame was
        // sent too late.
        let evc_hpw = self
            .ll
            .evc_hpw()
            .read()
            .map_err(|error| nb::Error::Other(Error::Spi(error.0)))?
            .value();
        if evc_hpw != 0 {
            return Err(nb::Error::Other(Error::DelayedSendTooLate));
        }

        // Check Transmitter Power-Up Warning Counter. If this is a delayed
        // transmission, this indicates that the transmitter was still powering
        // up while sending, and the frame preamble might not have transmit
        // correctly.
        let evc_tpw = self
            .ll
            .evc_tpw()
            .read()
            .map_err(|error| nb::Error::Other(Error::Spi(error.0)))?
            .value();
        if evc_tpw != 0 {
            return Err(nb::Error::Other(Error::DelayedSendPowerUpWarning));
        }

        // ATTENTION:
        // If you're changing anything about which SYS_STATUS flags are being
        // checked in this method, also make sure to update `enable_interrupts`.
        let sys_status = self
            .ll
            .sys_status()
            .read()
            .map_err(|error| nb::Error::Other(Error::Spi(error.0)))?;

        // Has the frame been sent?
        if sys_status.txfrs() == 0b0 {
            // Frame has not been sent
            return Err(nb::Error::WouldBlock);
        }

        // Frame sent
        self.reset_flags().map_err(nb::Error::Other)?;
        self.state.finished = true;

        let tx_timestamp = self
            .ll
            .tx_time()
            .read()
            .map_err(|error| nb::Error::Other(Error::Spi(error.0)))?
            .tx_stamp();
        // This is safe because the value read from the device will never be higher than the allowed value.
        let tx_timestamp = unsafe { Instant::new_unchecked(tx_timestamp) };

        Ok(tx_timestamp)
    }

    /// Finishes sending and returns to the `Ready` state.
    ///
    /// If the used tx continuation was not set to ready, this function returns an error.
    ///
    /// If the send operation has finished, as indicated by `wait`, this is a
    /// no-op. If the send operation is still ongoing, it will be aborted.
    #[allow(clippy::type_complexity)]
    pub fn finish_sending(self) -> Result<DW1000<SPI, Ready>, (Self, Error<SPI>)> {
        if self.state.continuation != TxContinuation::Ready {
            return Err((self, Error::WrongTxContinuation));
        }

        self.abort_sending()
    }

    /// Finishes sending and returns to the `Ready` state (no matter the tx continuation that was configured)
    ///
    /// If the send operation has finished, as indicated by `wait`, this is a
    /// no-op. If the send operation is still ongoing, it will be aborted.
    #[allow(clippy::type_complexity)]
    pub fn abort_sending(mut self) -> Result<DW1000<SPI, Ready>, (Self, Error<SPI>)> {
        if !self.state.finished {
            // Can't use `map_err` and `?` here, as the compiler will complain
            // about `self` moving into the closure.
            match self.force_idle(false) {
                Ok(()) => (),
                Err(error) => return Err((self, error)),
            }
            match self.reset_flags() {
                Ok(()) => (),
                Err(error) => return Err((self, error)),
            }
        }

        // Turn off the external transmit synchronization
        match self.ll.ec_ctrl().modify(|_, w| w.ostsm(0)) {
            Ok(_) => {}
            Err(e) => return Err((self, Error::Spi(e.0))),
        }

        Ok(DW1000 {
            ll: self.ll,
            seq: self.seq,
            state: Ready,
        })
    }

    /// Continue on in the receiving state.
    ///
    /// This can only be called when the tx config specified this should be the continuation.
    /// This function will not abort the send operation, so make sure to call [Self::wait_transmit] first.
    pub fn continue_receiving(
        mut self,
    ) -> Result<DW1000<SPI, SingleBufferReceiving>, (Self, Error<SPI>)> {
        let TxContinuation::Rx {
            frame_filtering,
            auto_ack,
        } = self.state.continuation
        else {
            return Err((self, Error::WrongTxContinuation));
        };

        if !self.state.finished {
            return Err((self, Error::TxNotFinishedyet));
        }

        match self.reset_flags() {
            Ok(()) => (),
            Err(error) => return Err((self, error)),
        }

        // Turn off the external transmit synchronization
        match self.ll.ec_ctrl().modify(|_, w| w.ostsm(0)) {
            Ok(_) => {}
            Err(e) => return Err((self, Error::Spi(e.0))),
        }

        Ok(DW1000 {
            ll: self.ll,
            seq: self.seq,
            state: SingleBufferReceiving {
                finished: false,
                config: RxConfig::from_tx_config(self.state.config, frame_filtering, auto_ack),
            },
        })
    }

    /// Continue on in the double buffered receiving state.
    ///
    /// This can only be called when the tx config specified this should be the continuation.
    /// This function will not abort the send operation, so make sure to call [Self::wait_transmit] first.
    pub fn continue_receiving_double_buffered(
        mut self,
    ) -> Result<DW1000<SPI, AutoDoubleBufferReceiving>, (Self, Error<SPI>)> {
        let TxContinuation::RxDoubleBuffered {
            frame_filtering,
            auto_ack,
        } = self.state.continuation
        else {
            return Err((self, Error::WrongTxContinuation));
        };

        if !self.state.finished {
            return Err((self, Error::TxNotFinishedyet));
        }

        let status = match self.ll.sys_status().read() {
            Ok(status) => status,
            Err(e) => return Err((self, Error::Spi(e.0))),
        };
        if status.hsrbp() != status.icrbp() {
            // The RX Buffer Pointer of the host and the ic side don't point to the same one.
            // We need to switch over
            match self.ll.sys_ctrl().modify(|_, w| w.hrbpt(1)) {
                Ok(()) => {}
                Err(e) => return Err((self, Error::Spi(e.0))),
            };
        }

        match self.reset_flags() {
            Ok(()) => (),
            Err(error) => return Err((self, error)),
        }

        // Turn off the external transmit synchronization
        match self.ll.ec_ctrl().modify(|_, w| w.ostsm(0)) {
            Ok(_) => {}
            Err(e) => return Err((self, Error::Spi(e.0))),
        }

        Ok(DW1000 {
            ll: self.ll,
            seq: self.seq,
            state: AutoDoubleBufferReceiving {
                finished: false,
                config: RxConfig::from_tx_config(self.state.config, frame_filtering, auto_ack),
            },
        })
    }

    fn reset_flags(&mut self) -> Result<(), Error<SPI>> {
        self.ll.sys_status().write(|w| {
            w.txfrb(0b1) // Transmit Frame Begins
                .txprs(0b1) // Transmit Preamble Sent
                .txphs(0b1) // Transmit PHY Header Sent
                .txfrs(0b1) // Transmit Frame Sent
        })?;

        Ok(())
    }
}
