use super::{AutoDoubleBufferReceiving, Receiving};
use crate::{
    configs::{BitRate, SfdSequence, TxContinuation},
    time::Instant,
    Error, Ready, RxConfig, Sending, SingleBufferReceiving, Sleeping, TxConfig, DW1000,
};
use byte::BytesExt as _;
use core::num::Wrapping;
use embedded_hal::spi::SpiDevice;
use ieee802154::mac::{self, FooterMode, FrameSerDesContext};

/// The behaviour of the sync pin
pub enum SyncBehaviour {
    /// The sync pin does nothing
    None,
    /// The radio time will reset to 0 when the sync pin is high and the clock gives a rising edge
    TimeBaseReset,
    /// When receiving, instead of reading the internal timestamp, the time since the last sync
    /// is given back.
    ExternalSync,
    /// When receiving, instead of reading the internal timestamp, the time since the last sync
    /// is given back. Also resets the internal timebase back to 0.
    ExternalSyncWithReset,
}

/// The time at which the transmission will start
pub enum SendTime {
    /// As fast as possible
    Now,
    /// After some time
    Delayed(Instant),
    /// After the sync pin is engaged. (Only works when sync setup is in ExternalSync mode)
    OnSync,
}

/// The polarity of the irq signal
pub enum IrqPolarity {
    /// The signal will be high when the interrupt is active
    ActiveHigh = 1,
    /// The signal will be low when the interrupt is active
    ActiveLow = 0,
}

impl<SPI> DW1000<SPI, Ready>
where
    SPI: SpiDevice,
{
    /// Sets the RX and TX antenna delays
    pub fn set_antenna_delay(&mut self, rx_delay: u16, tx_delay: u16) -> Result<(), Error<SPI>> {
        self.ll.lde_rxantd().write(|w| w.value(rx_delay))?;
        self.ll.tx_antd().write(|w| w.value(tx_delay))?;

        Ok(())
    }

    /// Sets the network id and address used for sending and receiving
    pub fn set_address(
        &mut self,
        pan_id: mac::PanId,
        addr: mac::ShortAddress,
    ) -> Result<(), Error<SPI>> {
        self.ll
            .panadr()
            .write(|w| w.pan_id(pan_id.0).short_addr(addr.0))?;

        Ok(())
    }

    /// Sets up the sync pin functionality
    ///
    /// After init, it is set to None
    pub fn set_sync_behaviour(&mut self, behaviour: SyncBehaviour) -> Result<(), Error<SPI>> {
        match behaviour {
            SyncBehaviour::None => {
                // Disable all
                self.ll.ec_ctrl().modify(|_, w| w.osrsm(0).ostrm(0))?;
                // Disable the rx pll
                self.ll.pmsc_ctrl1().modify(|_, w| w.pllsyn(0))?;
            }
            SyncBehaviour::TimeBaseReset => {
                // Enable the rx pll
                self.ll.pmsc_ctrl1().modify(|_, w| w.pllsyn(1))?;

                // Enable the time base reset mode
                self.ll
                    .ec_ctrl()
                    .modify(|_, w| w.pllldt(0b1).osrsm(0).ostrm(1).wait(33))?;
            }
            SyncBehaviour::ExternalSync => {
                // Enable the rx pll
                self.ll.pmsc_ctrl1().modify(|_, w| w.pllsyn(1))?;

                // Enable the external receive synchronisation mode
                self.ll
                    .ec_ctrl()
                    .modify(|_, w| w.pllldt(0b1).osrsm(1).ostrm(0).wait(33))?;
            }
            SyncBehaviour::ExternalSyncWithReset => {
                // Enable the rx pll
                self.ll.pmsc_ctrl1().modify(|_, w| w.pllsyn(1))?;

                // Enable the external receive synchronisation mode
                self.ll
                    .ec_ctrl()
                    .modify(|_, w| w.pllldt(0b1).osrsm(1).ostrm(1).wait(33))?;
            }
        }

        Ok(())
    }

    /// Set the polarity of the interrupt pin.
    ///
    /// The default is ActiveHigh, which is also recommended for power savings.
    pub fn set_irq_polarity(&mut self, polarity: IrqPolarity) -> Result<(), Error<SPI>> {
        self.ll
            .sys_cfg()
            .modify(|_, w| w.hirq_pol(polarity as u8))?;
        Ok(())
    }

    /// Send an IEEE 802.15.4 MAC frame
    ///
    /// The `data` argument is wrapped into an IEEE 802.15.4 MAC frame and sent
    /// to `destination`.
    ///
    /// This operation can be delayed to aid in distance measurement, by setting
    /// `delayed_time` to `Some(instant)`. If you want to send the frame as soon
    /// as possible, just pass `None` instead.
    ///
    /// The config parameter struct allows for setting the channel, bitrate, and
    /// more. This configuration needs to be the same as the configuration used
    /// by the receiver, or the message may not be received.
    /// The defaults are a sane starting point.
    ///
    /// This method starts the transmission and returns immediately thereafter.
    /// It consumes this instance of `DW1000` and returns another instance which
    /// is in the `Sending` state, and can be used to wait for the transmission
    /// to finish and check its result.
    pub fn send(
        mut self,
        data: &[u8],
        destination: Option<mac::Address>,
        send_time: SendTime,
        config: TxConfig,
    ) -> Result<DW1000<SPI, Sending>, Error<SPI>> {
        let seq = self.next_seq();

        let frame = mac::Frame {
            header: mac::Header {
                frame_type: mac::FrameType::Data,
                version: mac::FrameVersion::Ieee802154_2006,
                auxiliary_security_header: None,
                ie_present: false,
                seq_no_suppress: false,
                frame_pending: false,
                ack_request: false,
                pan_id_compress: false,
                destination,
                source: Some(self.get_address()?),
                seq,
            },
            content: mac::FrameContent::Data,
            payload: data,
            footer: [0; 2],
        };

        self.send_raw(
            |data| {
                let mut len = 0;
                let result = data.write_with(
                    &mut len,
                    frame,
                    &mut FrameSerDesContext::no_security(FooterMode::None),
                );

                if let Err(err) = result {
                    panic!("Failed to write frame: {:?}", err);
                }

                len
            },
            send_time,
            config,
        )
    }

    /// Get the sequence number for the next frame to be sent
    ///
    /// This also automatically increases the sequence number.
    pub fn next_seq(&mut self) -> u8 {
        let seq = self.seq.0;
        self.seq += Wrapping(1);
        seq
    }

    /// Send raw bytes
    ///
    /// The `writer` closure receives a buffer to write the data to be sent into.
    ///
    /// This operation can be delayed to aid in distance measurement, by setting
    /// `delayed_time` to `Some(instant)`. If you want to send the frame as soon
    /// as possible, just pass `None` instead.
    ///
    /// The config parameter struct allows for setting the channel, bitrate, and
    /// more. This configuration needs to be the same as the configuration used
    /// by the receiver, or the message may not be received.
    /// The defaults are a sane starting point.
    ///
    /// This method starts the transmission and returns immediately thereafter.
    /// It consumes this instance of `DW1000` and returns another instance which
    /// is in the `Sending` state, and can be used to wait for the transmission
    /// to finish and check its result.
    pub fn send_raw(
        mut self,
        writer: impl FnOnce(&mut [u8]) -> usize,
        send_time: SendTime,
        config: TxConfig,
    ) -> Result<DW1000<SPI, Sending>, Error<SPI>> {
        match config.continuation {
            TxContinuation::Ready => {
                // Clear event counters
                self.ll.evc_ctrl().write(|w| w.evc_clr(0b1))?;
                while self.ll.evc_ctrl().read()?.evc_clr() == 0b1 {}

                // (Re-)Enable event counters
                self.ll.evc_ctrl().write(|w| w.evc_en(0b1))?;
                while self.ll.evc_ctrl().read()?.evc_en() == 0b1 {}

                // Sometimes, for unknown reasons, the DW1000 gets stuck in RX mode.
                // Starting the transmitter won't get it to enter TX mode, which means
                // all subsequent send operations will fail. Let's disable the
                // transceiver and force the chip into IDLE mode to make sure that
                // doesn't happen.
                self.force_idle(false)?;
            }
            TxContinuation::Rx { frame_filtering } => {
                self.config_receiving::<SingleBufferReceiving>(RxConfig::from_tx_config(
                    config,
                    frame_filtering,
                ))?;
            }
            TxContinuation::RxDoubleBuffered { frame_filtering } => {
                self.config_receiving::<AutoDoubleBufferReceiving>(RxConfig::from_tx_config(
                    config,
                    frame_filtering,
                ))?;
            }
        }

        match send_time {
            SendTime::Delayed(time) => {
                // Put the time into the delay register
                // By setting this register, the chip knows to delay before transmitting
                self.ll.dx_time().write(|w| w.value(time.value()))?;
            }
            SendTime::OnSync => {
                self.ll.ec_ctrl().modify(|_, w| w.wait(33).ostsm(1))?;
            }
            _ => {}
        }

        // Prepare transmitter
        let mut len = 0;
        self.ll.tx_buffer().write(|w| {
            len = writer(w.data());
            w
        })?;
        self.ll.tx_fctrl().modify(|_, w| {
            let tflen = len as u8 + 2;
            w.tflen(tflen) // data length + two-octet CRC
                .tfle(0) // no non-standard length extension
                .txboffs(0) // no offset in TX_BUFFER
                .txbr(config.bitrate as u8) // configured bitrate
                .tr(config.ranging_enable as u8) // configured ranging bit
                .txprf(config.pulse_repetition_frequency as u8) // configured PRF
                .txpsr(((config.preamble_length as u8) & 0b1100) >> 2) // first two bits of configured preamble length
                .pe((config.preamble_length as u8) & 0b0011) // last two bits of configured preamble length
        })?;

        // Set the channel and sfd settings
        self.ll.chan_ctrl().modify(|_, w| {
            w.tx_chan(config.channel as u8)
                .rx_chan(config.channel as u8)
                .dwsfd(
                    (config.sfd_sequence == SfdSequence::Decawave
                        || config.sfd_sequence == SfdSequence::DecawaveAlt)
                        as u8,
                )
                .rxprf(config.pulse_repetition_frequency as u8)
                .tnssfd(
                    (config.sfd_sequence == SfdSequence::User
                        || config.sfd_sequence == SfdSequence::DecawaveAlt)
                        as u8,
                )
                .rnssfd(
                    (config.sfd_sequence == SfdSequence::User
                        || config.sfd_sequence == SfdSequence::DecawaveAlt)
                        as u8,
                )
                .tx_pcode(
                    config
                        .channel
                        .get_recommended_preamble_code(config.pulse_repetition_frequency),
                )
                .rx_pcode(
                    config
                        .channel
                        .get_recommended_preamble_code(config.pulse_repetition_frequency),
                )
        })?;

        match config.sfd_sequence {
            SfdSequence::IEEE => {} // IEEE has predefined sfd lengths and the register has no effect.
            SfdSequence::Decawave => self.ll.sfd_length().write(|w| w.value(8))?, // This isn't entirely necessary as the Decawave8 settings in chan_ctrl already force it to 8
            SfdSequence::DecawaveAlt => self.ll.sfd_length().write(|w| w.value(16))?, // Set to 16
            SfdSequence::User => {} // Users are responsible for setting the lengths themselves
        }

        // Tune for the correct channel
        self.ll
            .rf_txctrl()
            .write(|w| w.value(config.channel.get_recommended_rf_txctrl()))?;
        self.ll
            .tc_pgdelay()
            .write(|w| w.value(config.channel.get_recommended_tc_pgdelay()))?;
        self.ll
            .fs_pllcfg()
            .write(|w| w.value(config.channel.get_recommended_fs_pllcfg()))?;
        self.ll
            .fs_plltune()
            .write(|w| w.value(config.channel.get_recommended_fs_plltune()))?;

        // Set the LDE registers
        self.ll
            .lde_cfg2()
            .modify(|_, w| w.value(config.pulse_repetition_frequency.get_recommended_lde_cfg2()))?;
        self.ll.lde_repc().write(|w| {
            w.value(
                config.channel.get_recommended_lde_repc_value(
                    config.pulse_repetition_frequency,
                    config.bitrate,
                ),
            )
        })?;

        // Todo: Power control (register 0x1E)

        self.ll.sys_ctrl().modify(|_, w| {
            // Do we want to suppress crc generation?
            let w = w.sfcst(!config.append_crc as u8);

            if !matches!(send_time, SendTime::OnSync) {
                // Start transmission
                if matches!(send_time, SendTime::Delayed(_)) {
                    w.txdlys(0b1)
                } else {
                    w
                }
                .txstrt(0b1)
                .wait4resp(!matches!(config.continuation, TxContinuation::Ready) as u8)
            } else {
                w
            }
        })?;

        Ok(DW1000 {
            ll: self.ll,
            seq: self.seq,
            state: Sending {
                finished: false,
                continuation: config.continuation,
                config,
            },
        })
    }

    pub(super) fn config_receiving<RECEIVING: Receiving>(
        &mut self,
        config: RxConfig,
    ) -> Result<(), Error<SPI>> {
        // Really weird thing about double buffering I can't find anything about.
        // When a message is received in double buffer mode that should be filtered out,
        // the radio gives a really short fake interrupt.
        // This messes up all the logic, so unless a solution can be found we simply don't support it.
        if RECEIVING::DOUBLE_BUFFERED && config.frame_filtering {
            return Err(Error::RxConfigFrameFilteringUnsupported);
        }

        // For unknown reasons, the DW1000 gets stuck in RX mode without ever
        // receiving anything, after receiving one good frame. Reset the
        // receiver to make sure its in a valid state before attempting to
        // receive anything.
        self.ll.pmsc_ctrl0().modify(
            |_, w| w.softreset(0b1110), // reset receiver
        )?;
        self.ll.pmsc_ctrl0().modify(
            |_, w| w.softreset(0b1111), // clear reset
        )?;

        // We're already resetting the receiver in the previous step, and that's
        // good enough to make my example program that's both sending and
        // receiving work very reliably over many hours (that's not to say it
        // becomes unreliable after those hours, that's just when my test
        // stopped). However, I've seen problems with an example program that
        // only received, never sent, data. That got itself into some weird
        // state where it couldn't receive anymore.
        // I suspect that's because that example didn't have the following line
        // of code, while the send/receive example had that line of code, being
        // called from `send`.
        // While I haven't, as of this writing, run any hours-long tests to
        // confirm this does indeed fix the receive-only example, it seems
        // (based on my eyeball-only measurements) that the RX/TX example is
        // dropping fewer frames now.
        self.force_idle(false)?;

        self.ll.sys_cfg().modify(|_, w| {
            w.ffen(config.frame_filtering as u8) // enable or disable frame filtering
                .ffab(0b1) // receive beacon frames
                .ffad(0b1) // receive data frames
                .ffaa(0b1) // receive acknowledgement frames
                .ffam(0b1) // receive MAC command frames
                // Set the double buffering and auto re-enable
                .dis_drxb(!RECEIVING::DOUBLE_BUFFERED as u8)
                .rxautr(RECEIVING::AUTO_RX_REENABLE as u8)
                // Set whether the receiver should look for 110kbps or 850/6800kbps messages
                .rxm110k((config.bitrate == BitRate::Kbps110) as u8)
        })?;

        // Set PLLLDT bit in EC_CTRL. According to the documentation of the
        // CLKPLL_LL bit in SYS_STATUS, this bit needs to be set to ensure the
        // reliable operation of the CLKPLL_LL bit. Since I've seen that bit
        // being set, I want to make sure I'm not just seeing crap.
        self.ll.ec_ctrl().modify(|_, w| w.pllldt(0b1))?;

        // Now that PLLLDT is set, clear all bits in SYS_STATUS that depend on
        // it for reliable operation. After that is done, these bits should work
        // reliably.
        self.ll
            .sys_status()
            .write(|w| w.cplock(0b1).clkpll_ll(0b1))?;

        // Apply the config
        self.ll.chan_ctrl().modify(|_, w| {
            w.tx_chan(config.channel as u8)
                .rx_chan(config.channel as u8)
                .dwsfd(
                    (config.sfd_sequence == SfdSequence::Decawave
                        || config.sfd_sequence == SfdSequence::DecawaveAlt)
                        as u8,
                )
                .rxprf(config.pulse_repetition_frequency as u8)
                .tnssfd(
                    (config.sfd_sequence == SfdSequence::User
                        || config.sfd_sequence == SfdSequence::DecawaveAlt)
                        as u8,
                )
                .rnssfd(
                    (config.sfd_sequence == SfdSequence::User
                        || config.sfd_sequence == SfdSequence::DecawaveAlt)
                        as u8,
                )
                .tx_pcode(
                    config
                        .channel
                        .get_recommended_preamble_code(config.pulse_repetition_frequency),
                )
                .rx_pcode(
                    config
                        .channel
                        .get_recommended_preamble_code(config.pulse_repetition_frequency),
                )
        })?;

        match config.sfd_sequence {
            SfdSequence::IEEE => {} // IEEE has predefined sfd lengths and the register has no effect.
            SfdSequence::Decawave => self.ll.sfd_length().write(|w| w.value(8))?, // This isn't entirely necessary as the Decawave8 settings in chan_ctrl already force it to 8
            SfdSequence::DecawaveAlt => self.ll.sfd_length().write(|w| w.value(16))?, // Set to 16
            SfdSequence::User => {} // Users are responsible for setting the lengths themselves
        }

        // Set general tuning
        self.ll.drx_tune0b().write(|w| {
            w.value(
                config
                    .bitrate
                    .get_recommended_drx_tune0b(config.sfd_sequence),
            )
        })?;
        self.ll.drx_tune1a().write(|w| {
            w.value(
                config
                    .pulse_repetition_frequency
                    .get_recommended_drx_tune1a(),
            )
        })?;
        let drx_tune1b = config
            .expected_preamble_length
            .get_recommended_drx_tune1b(config.bitrate)?;
        self.ll.drx_tune1b().write(|w| w.value(drx_tune1b))?;
        let drx_tune2 = config
            .pulse_repetition_frequency
            .get_recommended_drx_tune2(
                config.expected_preamble_length.get_recommended_pac_size(),
            )?;
        self.ll.drx_tune2().write(|w| w.value(drx_tune2))?;
        self.ll
            .drx_tune4h()
            .write(|w| w.value(config.expected_preamble_length.get_recommended_dxr_tune4h()))?;

        // Set channel tuning
        self.ll
            .rf_rxctrlh()
            .write(|w| w.value(config.channel.get_recommended_rf_rxctrlh()))?;
        self.ll
            .fs_pllcfg()
            .write(|w| w.value(config.channel.get_recommended_fs_pllcfg()))?;
        self.ll
            .fs_plltune()
            .write(|w| w.value(config.channel.get_recommended_fs_plltune()))?;

        // Set the LDE registers
        self.ll
            .lde_cfg2()
            .write(|w| w.value(config.pulse_repetition_frequency.get_recommended_lde_cfg2()))?;
        self.ll.lde_repc().write(|w| {
            w.value(
                config.channel.get_recommended_lde_repc_value(
                    config.pulse_repetition_frequency,
                    config.bitrate,
                ),
            )
        })?;

        // Check if the rx buffer pointer is correct
        let status = self.ll.sys_status().read()?;
        if status.hsrbp() != status.icrbp() {
            // The RX Buffer Pointer of the host and the ic side don't point to the same one.
            // We need to switch over
            self.ll.sys_ctrl().modify(|_, w| w.hrbpt(1))?;
        }

        Ok(())
    }

    /// Attempt to receive a single IEEE 802.15.4 MAC frame
    ///
    /// Initializes the receiver. The method consumes this instance of `DW1000`
    /// and returns another instance which is in the [SingleBufferReceiving] state, and can
    /// be used to wait for a message.
    ///
    /// The config parameter allows for the configuration of bitrate, channel
    /// and more. Make sure that the values used are the same as of the frames
    /// that are transmitted. The default works with the TxConfig's default and
    /// is a sane starting point.
    pub fn receive(
        mut self,
        config: RxConfig,
    ) -> Result<DW1000<SPI, SingleBufferReceiving>, Error<SPI>> {
        self.config_receiving::<SingleBufferReceiving>(config)?;

        let mut rx_radio = DW1000 {
            ll: self.ll,
            seq: self.seq,
            state: SingleBufferReceiving {
                finished: false,
                config,
            },
        };

        // Start rx'ing
        rx_radio.start_receiving()?;

        // Return the double buffer state
        Ok(rx_radio)
    }

    /// Attempt to receive many IEEE 802.15.4 MAC frames. This is done in double buffered mode and auto re-enable.
    /// This means that once a message has been received, the radio will switch receive buffer and continue receiving.
    ///
    /// If the double buffer is full while another message comes in, then the buffers will be corrupted.
    /// In that case, you will get the radio back in the [Ready] state and have to manually call this function again.
    ///
    /// Initializes the receiver. The method consumes this instance of `DW1000`
    /// and returns another instance which is in the [AutoDoubleBufferReceiving] state, and can
    /// be used to wait for a message.
    ///
    /// The config parameter allows for the configuration of bitrate, channel
    /// and more. Make sure that the values used are the same as of the frames
    /// that are transmitted. The default works with the TxConfig's default and
    /// is a sane starting point.
    pub fn receive_auto_double_buffered(
        mut self,
        config: RxConfig,
    ) -> Result<DW1000<SPI, AutoDoubleBufferReceiving>, Error<SPI>> {
        self.config_receiving::<AutoDoubleBufferReceiving>(config)?;

        let mut rx_radio = DW1000 {
            ll: self.ll,
            seq: self.seq,
            state: AutoDoubleBufferReceiving {
                finished: false,
                config,
            },
        };

        // Start rx'ing
        rx_radio.start_receiving()?;

        // Return the double buffer state
        Ok(rx_radio)
    }

    /// Enables transmit interrupts for the events that `wait` checks
    ///
    /// Overwrites any interrupt flags that were previously set.
    pub fn enable_tx_interrupts(&mut self) -> Result<(), Error<SPI>> {
        self.ll.sys_mask().modify(|_, w| w.mtxfrs(0b1))?;
        Ok(())
    }

    /// Enables receive interrupts for the events that `wait` checks
    ///
    /// Overwrites any interrupt flags that were previously set.
    pub fn enable_rx_interrupts(&mut self) -> Result<(), Error<SPI>> {
        self.ll().sys_mask().modify(|_, w| {
            w.mrxdfr(0b1) // Data Frame Ready
                .mrxfce(0b1) // FCS Error
                .mrxphe(0b1) // PHY Header Error
                .mrxrfsl(0b1) // Reed Solomon Frame Sync Loss
                .mrxrfto(0b1) // Receive Frame Wait Timeout
                .mrxovrr(0b1) // Overrun
                .mrxpto(0b1) // Preamble detection timeout
                .mrxsfdto(0b1) // SFD Timeout
                .maffrej(0b1) // Automatic Frame Filter Rejection
        })?;

        Ok(())
    }

    /// Disables all interrupts
    pub fn disable_interrupts(&mut self) -> Result<(), Error<SPI>> {
        self.ll.sys_mask().write(|w| w)?;
        Ok(())
    }

    /// Configures the gpio pins to operate as LED output.
    ///
    /// - Note: This means that the function of the gpio pins change
    /// - Note: Both the kilohertz and debounce clock will be turned on or off
    /// ---
    /// - RXOKLED will change GPIO0
    /// - SFDLED will change GPIO1
    /// - RXLED will change GPIO2
    /// - TXLED will change GPIO3
    ///
    /// blink_time is in units of 14 ms
    pub fn configure_leds(
        &mut self,
        enable_rx_ok: bool,
        enable_sfd: bool,
        enable_rx: bool,
        enable_tx: bool,
        blink_time: u8,
    ) -> Result<(), Error<SPI>> {
        // Turn on the timer that will control the blinking (The debounce clock)
        self.ll.pmsc_ctrl0().modify(|_, w| {
            w.gpdce((enable_rx_ok || enable_sfd || enable_rx || enable_tx) as u8)
                .khzclken((enable_rx_ok || enable_sfd || enable_rx || enable_tx) as u8)
        })?;

        // Turn on the led blinking
        self.ll.pmsc_ledc().modify(|_, w| {
            w.blnken((enable_rx_ok || enable_sfd || enable_rx || enable_tx) as u8)
                .blink_tim(blink_time)
        })?;

        // Set the proper gpio mode
        self.ll.gpio_mode().modify(|_, w| {
            w.msgp0(enable_rx_ok as u8)
                .msgp1(enable_sfd as u8)
                .msgp2(enable_rx as u8)
                .msgp3(enable_tx as u8)
        })?;

        Ok(())
    }

    /// Puts the dw1000 into sleep mode.
    ///
    /// - `irq_on_wakeup`: When set to true, the IRQ pin will be asserted when the radio wakes up
    /// - `sleep_duration`: When `None`, the radio will not wake up by itself and go into the deep sleep mode.
    ///   When `Some`, then the radio will wake itself up after the given time. Every tick is ~431ms, but there may
    ///   be a significant deviation from this due to the chip's manufacturing process.
    ///
    /// *Note: The SPI speed may be at most 3 Mhz when calling this function.*
    pub fn enter_sleep(
        mut self,
        irq_on_wakeup: bool,
        sleep_duration: Option<u16>,
    ) -> Result<DW1000<SPI, Sleeping>, Error<SPI>> {
        // Set the sleep timer
        if let Some(sd) = sleep_duration {
            self.ll.pmsc_ctrl0().modify(|_, w| {
                w
                    // Force the 19.2Mhz clock
                    .sysclks(0b01)
            })?;

            // Disable the sleep counter
            self.ll
                .aon_cfg1()
                .write(|w| w.sleep_cen(0).smxx(0).lposc_cal(0))?;
            // Set the counter
            self.ll.aon_cfg0().write(|w| w.sleep_tim(sd))?;
            // Enable the sleep counter
            self.ll.aon_cfg1().write(|w| w.sleep_cen(1).lposc_cal(1))?;
            // Upload array
            self.ll.aon_ctrl().write(|w| w.upl_cfg(1))?;
            self.ll.aon_ctrl().write(|w| w.upl_cfg(0))?;

            self.ll.pmsc_ctrl0().modify(|_, w| {
                w
                    // Auto clock
                    .sysclks(0b00)
            })?;
        }

        // Save the settings that the
        let tx_antenna_delay = self.get_tx_antenna_delay()?;

        // Setup the interrupt.
        if irq_on_wakeup {
            self.ll
                .sys_mask()
                .modify(|_, w| w.mslp2init(1).mcplock(1))?;
        }

        // Does the chip have the ldo tune calibrated?
        let lldo = self.read_otp(0x004)? != 0;

        // Setup everything that needs to be stored in AON
        self.ll
            .aon_wcfg()
            .modify(|_, w| w.onw_ldc(1).onw_llde(1).onw_lldo(lldo as u8).onw_l64p(1))?;

        // Setup the wakeup sources.
        self.ll.aon_cfg0().modify(|_, w| {
            w.wake_spi(1)
                .wake_cnt(sleep_duration.is_some() as u8)
                .sleep_en(1)
        })?;

        // Upload always on array configuration and enter sleep
        self.ll.aon_ctrl().write(|w| w)?;
        self.ll.aon_ctrl().write(|w| w.save(1))?;

        Ok(DW1000 {
            ll: self.ll,
            seq: self.seq,
            state: Sleeping { tx_antenna_delay },
        })
    }
}
