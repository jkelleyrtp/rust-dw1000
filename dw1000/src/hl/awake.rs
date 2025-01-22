use super::Awake;
use crate::{
    ll, mac,
    time::{Duration, Instant},
    Error, DW1000,
};
use embedded_hal::spi::SpiDevice;

impl<SPI, State> DW1000<SPI, State>
where
    SPI: SpiDevice,
    State: Awake,
{
    /// Returns the TX antenna delay
    pub fn get_tx_antenna_delay(&mut self) -> Result<Duration, Error<SPI>> {
        let tx_antenna_delay = self.ll.tx_antd().read()?.value();

        // Since `tx_antenna_delay` is `u16`, the following will never panic.
        let tx_antenna_delay = Duration::new(tx_antenna_delay.into()).unwrap();

        Ok(tx_antenna_delay)
    }

    /// Returns the RX antenna delay
    pub fn get_rx_antenna_delay(&mut self) -> Result<Duration, Error<SPI>> {
        let rx_antenna_delay = self.ll.lde_rxantd().read()?.value();

        // Since `rx_antenna_delay` is `u16`, the following will never panic.
        let rx_antenna_delay = Duration::new(rx_antenna_delay.into()).unwrap();

        Ok(rx_antenna_delay)
    }

    /// Returns the network id and address used for sending and receiving
    pub fn get_address(&mut self) -> Result<mac::Address, Error<SPI>> {
        let panadr = self.ll.panadr().read()?;

        Ok(mac::Address::Short(
            mac::PanId(panadr.pan_id()),
            mac::ShortAddress(panadr.short_addr()),
        ))
    }

    /// Returns the current system time
    pub fn sys_time(&mut self) -> Result<Instant, Error<SPI>> {
        let sys_time = self.ll.sys_time().read()?.value();

        // Since hardware timestamps fit within 40 bits, the following should
        // never panic.
        Ok(Instant::new(sys_time).unwrap())
    }

    /// Set the value of the pending bit used by the auto acks
    pub fn set_pending_bit(&mut self, value: bool) -> Result<(), Error<SPI>> {
        self.ll.sys_cfg().modify(|_, w| w.aackpend(value as u8))?;
        Ok(())
    }

    /// Provides direct access to the register-level API
    ///
    /// Be aware that by using the register-level API, you can invalidate
    /// various assumptions that the high-level API makes about the operation of
    /// the DW1000. Don't use the register-level and high-level APIs in tandem,
    /// unless you know what you're doing.
    pub fn ll(&mut self) -> &mut ll::DW1000<SPI> {
        &mut self.ll
    }

    /// Force the DW1000 into IDLE mode
    ///
    /// Any ongoing RX/TX operations will be aborted.
    pub(super) fn force_idle(&mut self, double_buffered: bool) -> Result<(), Error<SPI>> {
        let mut saved_sys_mask = [0; 5];

        if double_buffered {
            // Mask the double buffered status bits
            self.ll.sys_mask().modify(|r, w| {
                saved_sys_mask = r.0;
                w.mrxfce(0).mrxfcg(0).mrxdfr(0).mldedone(0)
            })?;
        }

        self.ll.sys_ctrl().write(|w| w.trxoff(0b1))?;
        while self.ll.sys_ctrl().read()?.trxoff() == 0b1 {}

        if double_buffered {
            // Clear the bits
            self.ll().sys_status().write(
                |w| {
                    w.rxprd(0b1) // Receiver Preamble Detected
                        .rxsfdd(0b1) // Receiver SFD Detected
                        .ldedone(0b1) // LDE Processing Done
                        .rxphd(0b1) // Receiver PHY Header Detected
                        .rxphe(0b1) // Receiver PHY Header Error
                        .rxdfr(0b1) // Receiver Data Frame Ready
                        .rxfcg(0b1) // Receiver FCS Good
                        .rxfce(0b1) // Receiver FCS Error
                        .rxrfsl(0b1) // Receiver Reed Solomon Frame Sync Loss
                        .rxrfto(0b1) // Receiver Frame Wait Timeout
                        .ldeerr(0b1) // Leading Edge Detection Processing Error
                        .rxovrr(0b1) // Receiver Overrun
                        .rxpto(0b1) // Preamble Detection Timeout
                        .rxsfdto(0b1) // Receiver SFD Timeout
                        .rxrscs(0b1) // Receiver Reed-Solomon Correction Status
                        .rxprej(0b1)
                }, // Receiver Preamble Rejection
            )?;

            // Restore the mask
            self.ll.sys_mask().write(|w| {
                w.0.copy_from_slice(&saved_sys_mask);
                w
            })?;
        }

        Ok(())
    }

    pub(crate) fn read_otp(&mut self, address: u16) -> Result<u32, Error<SPI>> {
        // Set address
        self.ll.otp_addr().write(|w| w.value(address))?;
        // Switch into read mode
        self.ll.otp_ctrl().write(|w| w.otprden(0b1).otpread(0b1))?;
        self.ll.otp_ctrl().write(|w| w.otprden(0b1))?;
        // Read back value
        let value = self.ll.otp_rdat().read()?.value();
        // End read mode
        self.ll.otp_ctrl().write(|w| w)?;
        Ok(value)
    }
}
