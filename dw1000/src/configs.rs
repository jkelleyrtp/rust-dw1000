//! Configuration structs for sending and receiving
//!
//! This module houses the datastructures that control how frames are transmitted and received.
//! The configs are passed to the send and receive functions.

use crate::Error;
use embedded_hal::spi::SpiDevice;
use num_enum::TryFromPrimitive;
use serde::{Deserialize, Serialize};

#[derive(Copy, Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
/// Transmit configuration
pub struct TxConfig {
    /// Sets the bitrate of the transmission.
    pub bitrate: BitRate,
    /// Sets the ranging bit in the transmitted frame.
    /// This has no effect on the capabilities of the DW1000.
    pub ranging_enable: bool,
    /// Sets the PRF value of the transmission.
    pub pulse_repetition_frequency: PulseRepetitionFrequency,
    /// The length of the preamble.
    pub preamble_length: PreambleLength,
    /// The channel that the DW1000 will transmit at.
    pub channel: UwbChannel,
    /// The SFD sequence that is used to transmit a frame.
    pub sfd_sequence: SfdSequence,
    /// When true, a CRC will be appended to the message
    pub append_crc: bool,
    /// Setting for how the transmission should be continued
    pub continuation: TxContinuation,
}

impl Default for TxConfig {
    fn default() -> Self {
        TxConfig {
            bitrate: Default::default(),
            ranging_enable: false,
            pulse_repetition_frequency: Default::default(),
            preamble_length: Default::default(),
            channel: Default::default(),
            sfd_sequence: Default::default(),
            append_crc: true,
            continuation: TxContinuation::Ready,
        }
    }
}

/// Setting for how the transmission should be continued
#[derive(Copy, Clone, Debug, Eq, PartialEq, Serialize, Deserialize, Default)]
pub enum TxContinuation {
    #[default]
    /// After the transmission the radio should go back to ready
    Ready,
    /// After the transmission the radio should go to the receiving state
    Rx {
        /// Enable frame filtering. See [RxConfig::frame_filtering]
        frame_filtering: bool,
        /// Used as the [RxConfig::auto_ack] value
        auto_ack: AutoAck,
    },
    /// After the transmission the radio should go to the double buffered receiving state
    RxDoubleBuffered {
        /// Enable frame filtering. See [RxConfig::frame_filtering]
        frame_filtering: bool,
        /// Used as the [RxConfig::auto_ack] value
        auto_ack: AutoAck,
    },
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
/// Receive configuration
pub struct RxConfig {
    /// The bitrate that will be used for reception.
    pub bitrate: BitRate,
    /// Enable frame filtering
    ///
    /// If true, only frames directly addressed to this node and broadcasts will
    /// be received.
    ///
    /// Defaults to `true`.
    pub frame_filtering: bool,
    /// Sets the PRF value of the reception
    pub pulse_repetition_frequency: PulseRepetitionFrequency,
    /// The expected preamble length.
    ///
    /// This affects the chosen PAC size.
    /// This should be the same as the preamble length that is used to send the messages.
    /// It is not a filter, though, so other preamble lengths may still be received.
    pub expected_preamble_length: PreambleLength,
    /// The channel that the DW1000 will listen at.
    pub channel: UwbChannel,
    /// The type of SFD sequence that will be scanned for.
    pub sfd_sequence: SfdSequence,
    /// When true, a CRC will be expected to be appended to the message
    pub append_crc: bool,
    /// When enabled, the radio itself will send acks to messages with the ack bit enabled.
    pub auto_ack: AutoAck,
}

impl RxConfig {
    /// Create an [RxConfig] by copying the properties from a [TxConfig] and adding missing properties
    pub const fn from_tx_config(tx_config: TxConfig, frame_filtering: bool, auto_ack: AutoAck) -> Self {
        let TxConfig {
            bitrate,
            ranging_enable: _,
            pulse_repetition_frequency,
            preamble_length,
            channel,
            sfd_sequence,
            append_crc,
            continuation: _,
        } = tx_config;

        RxConfig {
            bitrate,
            frame_filtering,
            pulse_repetition_frequency,
            expected_preamble_length: preamble_length,
            channel,
            sfd_sequence,
            append_crc,
            auto_ack,
        }
    }
}

impl Default for RxConfig {
    fn default() -> Self {
        Self {
            bitrate: Default::default(),
            frame_filtering: true,
            pulse_repetition_frequency: Default::default(),
            expected_preamble_length: Default::default(),
            channel: Default::default(),
            sfd_sequence: Default::default(),
            append_crc: true,
            auto_ack: Default::default(),
        }
    }
}

/// The auto acknowledge behavior
#[derive(Copy, Clone, Debug, Eq, PartialEq, Serialize, Deserialize, Default)]
pub enum AutoAck {
    #[default]
    /// No automatic acks are sent
    Disabled,
    /// An automatic Ack will be sent if:
    /// - Frame filtering is on
    /// - The received frame passes through the filter
    /// - The frame has the ack bit set
    Enabled {
        /// The turnaround time in number of symbols.
        ///
        /// Recommended minimal value:
        /// - 110 kbps: 0
        /// - 850 kbps: 2
        /// - 6.8 Mbps: 3
        turnaround_time: u8,
    },
}

#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Serialize, Deserialize, TryFromPrimitive)]
#[repr(u8)]
/// The bitrate at which a message is transmitted
pub enum BitRate {
    /// 110 kilobits per second.
    /// This is an unofficial extension from decawave.
    Kbps110 = 0b00,
    /// 850 kilobits per second.
    Kbps850 = 0b01,
    /// 6.8 megabits per second.
    #[default]
    Kbps6800 = 0b10,
}

impl BitRate {
    /// Gets the recommended drx_tune0b value for the bitrate and sfd.
    pub fn get_recommended_drx_tune0b(&self, sfd_sequence: SfdSequence) -> u16 {
        // Values are taken from Table 30 of the DW1000 User Manual.
        match (self, sfd_sequence) {
            (BitRate::Kbps110, SfdSequence::IEEE) => 0x000A,
            (BitRate::Kbps110, _) => 0x0016,
            (BitRate::Kbps850, SfdSequence::IEEE) => 0x0001,
            (BitRate::Kbps850, _) => 0x0006,
            (BitRate::Kbps6800, SfdSequence::IEEE) => 0x0001,
            (BitRate::Kbps6800, _) => 0x0002,
        }
    }
}

#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Serialize, Deserialize, TryFromPrimitive)]
#[repr(u8)]
/// The PRF value
pub enum PulseRepetitionFrequency {
    /// 16 megahertz
    #[default]
    Mhz16 = 0b01,
    /// 64 megahertz
    Mhz64 = 0b10,
}

impl PulseRepetitionFrequency {
    /// Gets the recommended value for the drx_tune1a register based on the PRF
    pub fn get_recommended_drx_tune1a(&self) -> u16 {
        // Values taken from Table 31 of the DW1000 User Manual.
        match self {
            PulseRepetitionFrequency::Mhz16 => 0x0087,
            PulseRepetitionFrequency::Mhz64 => 0x008D,
        }
    }

    /// Gets the recommended value for the drx_tune2 register based on the PRF and PAC size
    pub fn get_recommended_drx_tune2<SPI>(&self, pac_size: u8) -> Result<u32, Error<SPI>>
    where
        SPI: SpiDevice,
    {
        // Values taken from Table 33 of the DW1000 User Manual.
        match (self, pac_size) {
            (PulseRepetitionFrequency::Mhz16, 8) => Ok(0x311A002D),
            (PulseRepetitionFrequency::Mhz64, 8) => Ok(0x313B006B),
            (PulseRepetitionFrequency::Mhz16, 16) => Ok(0x331A0052),
            (PulseRepetitionFrequency::Mhz64, 16) => Ok(0x333B00BE),
            (PulseRepetitionFrequency::Mhz16, 32) => Ok(0x351A009A),
            (PulseRepetitionFrequency::Mhz64, 32) => Ok(0x353B015E),
            (PulseRepetitionFrequency::Mhz16, 64) => Ok(0x371A011D),
            (PulseRepetitionFrequency::Mhz64, 64) => Ok(0x373B0296),
            // The PAC size is something we didn't expect
            _ => Err(Error::InvalidConfiguration),
        }
    }

    /// Get the recommended value for the lde_cfg2 register
    pub fn get_recommended_lde_cfg2(&self) -> u16 {
        // Values taken from from the user manual (found at the register description)
        match self {
            PulseRepetitionFrequency::Mhz16 => 0x1607,
            PulseRepetitionFrequency::Mhz64 => 0x0607,
        }
    }
}

#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Serialize, Deserialize, TryFromPrimitive)]
#[repr(u8)]
/// An enum that specifies the length of the preamble.
///
/// Longer preambles improve the reception quality and thus range.
/// This comes at the cost of longer transmission times and thus power consumption and bandwidth use.
///
/// For the bit pattern, see table 16 in the user manual. Two bits TXPSR,then two bits PE.
pub enum PreambleLength {
    /// 64 symbols of preamble.
    /// Only supported at Bitrate::Kbps6800.
    Symbols64 = 0b0100,
    /// 128 symbols of preamble.
    /// Only supported at Bitrate::Kbps850 & Bitrate::Kbps6800.
    /// Unofficial extension from decawave.
    #[default]
    Symbols128 = 0b0101,
    /// 256 symbols of preamble.
    /// Only supported at Bitrate::Kbps850 & Bitrate::Kbps6800.
    /// Unofficial extension from decawave.
    Symbols256 = 0b0110,
    /// 512 symbols of preamble.
    /// Only supported at Bitrate::Kbps850 & Bitrate::Kbps6800.
    /// Unofficial extension from decawave.
    Symbols512 = 0b0111,
    /// 1024 symbols of preamble.
    /// Only supported at Bitrate::Kbps850 & Bitrate::Kbps6800.
    Symbols1024 = 0b1000,
    /// 1536 symbols of preamble.
    /// Only supported at Bitrate::Kbps110.
    /// Unofficial extension from decawave.
    Symbols1536 = 0b1001,
    /// 2048 symbols of preamble.
    /// Only supported at Bitrate::Kbps110.
    /// Unofficial extension from decawave.
    Symbols2048 = 0b1010,
    /// 4096 symbols of preamble.
    /// Only supported at Bitrate::Kbps110.
    Symbols4096 = 0b1100,
}

impl PreambleLength {
    /// Gets the recommended PAC size based on the preamble length.
    pub fn get_recommended_pac_size(&self) -> u8 {
        // Values are taken from Table 6 of the DW1000 User manual
        match self {
            PreambleLength::Symbols64 => 8,
            PreambleLength::Symbols128 => 8,
            PreambleLength::Symbols256 => 16,
            PreambleLength::Symbols512 => 16,
            PreambleLength::Symbols1024 => 32,
            PreambleLength::Symbols1536 => 64,
            PreambleLength::Symbols2048 => 64,
            PreambleLength::Symbols4096 => 64,
        }
    }

    /// Gets the recommended drx_tune1b register value based on the preamble length and the bitrate.
    pub fn get_recommended_drx_tune1b<SPI>(&self, bitrate: BitRate) -> Result<u16, Error<SPI>>
    where
        SPI: SpiDevice,
    {
        // Values are taken from Table 32 of the DW1000 User manual
        match (self, bitrate) {
            (PreambleLength::Symbols64, BitRate::Kbps6800) => Ok(0x0010),
            (PreambleLength::Symbols128, BitRate::Kbps6800) => Ok(0x0020),
            (PreambleLength::Symbols256, BitRate::Kbps6800) => Ok(0x0020),
            (PreambleLength::Symbols512, BitRate::Kbps6800) => Ok(0x0020),
            (PreambleLength::Symbols1024, BitRate::Kbps6800) => Ok(0x0020),
            (PreambleLength::Symbols128, BitRate::Kbps850) => Ok(0x0020),
            (PreambleLength::Symbols256, BitRate::Kbps850) => Ok(0x0020),
            (PreambleLength::Symbols512, BitRate::Kbps850) => Ok(0x0020),
            (PreambleLength::Symbols1024, BitRate::Kbps850) => Ok(0x0020),
            (PreambleLength::Symbols1536, BitRate::Kbps110) => Ok(0x0064),
            (PreambleLength::Symbols2048, BitRate::Kbps110) => Ok(0x0064),
            (PreambleLength::Symbols4096, BitRate::Kbps110) => Ok(0x0064),
            _ => Err(Error::InvalidConfiguration),
        }
    }

    /// Gets the recommended dxr_tune4h register value based on the preamble length.
    pub fn get_recommended_dxr_tune4h(&self) -> u16 {
        // Values are taken from Table 34 of the DW1000 User manual
        match self {
            PreambleLength::Symbols64 => 0x0010,
            _ => 0x0028,
        }
    }
}

#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Serialize, Deserialize, TryFromPrimitive)]
#[repr(u8)]
/// An enum that allows the selection between different SFD sequences
///
/// The difference between the two Decawave sequences is that there are two ways
/// to enable it in the chip. Decawave will only set the DWSFD bit and
/// DecawaveAlt set the DWSFD and the \[T,R\]NSSFD bits.
pub enum SfdSequence {
    /// The standard sequence defined by the IEEE standard.
    /// Most likely the best choice for 6.8 Mbps connections.
    ///
    /// Using a given data rate, this is what the register get set to:
    ///
    /// | Data rate | DWSFD | TNSSFD | RNSSFD | Symbol Length| Raw SFD |
    /// |:----------|:-----:|:------:|:------:|:------------:|:--------|
    /// | 110 kbps  | 0     | 0      | 0      | Not set      | IEEE 64 symbols, not recommended |
    /// | 850 kbps  | 0     | 0      | 0      | Not set      | IEEE 8 symbols, not recommended |
    /// | 6.8 mbps  | 0     | 0      | 0      | Not set      | IEEE 8 symbol |
    #[default]
    IEEE,
    /// A sequence defined by Decawave that is supposed to be more robust.
    /// This is an unofficial addition.
    /// Most likely the best choice for 110 Kbps connections.
    ///
    /// Using a given data rate, this is what the register get set to:
    ///
    /// | Data rate | DWSFD | TNSSFD | RNSSFD | Symbol Length| Raw SFD |
    /// |:----------|:-----:|:------:|:------:|:------------:|:--------|
    /// | 110 kbps  | 1     | 0      | 0      | 8            | Deca 64 symbols |
    /// | 850 kbps  | 1     | 0      | 0      | 8            | Deca 8 symbols, not recommended |
    /// | 6.8 mbps  | 1     | 0      | 0      | 8            | Undefined by manual, not recommended |
    Decawave,
    /// A sequence defined by Decawave that is supposed to be more robust.
    /// This is an unofficial addition.
    /// Most likely the best choice for 850 Kbps connections.
    ///
    /// Using a given data rate, this is what the register get set to:
    ///
    /// | Data rate | DWSFD | TNSSFD | RNSSFD | Symbol Length| Raw SFD |
    /// |:----------|:-----:|:------:|:------:|:------------:|:--------|
    /// | 110 kbps  | 1     | 0      | 0      | 16           | Deca 64 symbols, not recommended |
    /// | 850 kbps  | 1     | 0      | 0      | 16           | Deca 16 symbols |
    /// | 6.8 mbps  | 1     | 0      | 0      | 16           | Undefined by manual, not recommended |
    DecawaveAlt,
    /// Uses the sequence that is programmed in by the user.
    /// This is an unofficial addition.
    ///
    /// Using a given data rate, this is what the register get set to:
    ///
    /// | Data rate | DWSFD | TNSSFD | RNSSFD | Symbol Length| Raw SFD |
    /// |:----------|:-----:|:------:|:------:|:------------:|:--------|
    /// | 110 kbps  | 0     | 1      | 1      | Set by user  | Custom, not recommended |
    /// | 850 kbps  | 0     | 1      | 1      | Set by user  | Custom, not recommended |
    /// | 6.8 mbps  | 0     | 1      | 1      | Set by user  | Custom, not recommended |
    User,
}

impl SfdSequence {
    /// Gets the adjustment that needs to be made to the rxpacc field of the RX Frame Information Register.
    /// Follows table 18 of the user manual.
    pub fn get_rxpacc_adjustment(&self, bit_rate: BitRate) -> i8 {
        match self {
            SfdSequence::IEEE => {
                match bit_rate {
                    BitRate::Kbps110 => 64,                     // 64 Symbols
                    BitRate::Kbps850 | BitRate::Kbps6800 => -5, // 8 Symbols
                }
            }
            SfdSequence::Decawave => {
                match bit_rate {
                    BitRate::Kbps110 => 82,  // 64 Symbols
                    BitRate::Kbps850 => -10, // 8 Symbols
                    BitRate::Kbps6800 => 0,  // Undefined setting
                }
            }
            SfdSequence::DecawaveAlt => {
                match bit_rate {
                    BitRate::Kbps110 => 82,  // 64 Symbols
                    BitRate::Kbps850 => -18, // 16 Symbols
                    BitRate::Kbps6800 => 0,  // Undefined setting
                }
            }
            SfdSequence::User => 0, // Undefined setting
        }
    }
}

#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Serialize, Deserialize, TryFromPrimitive)]
#[repr(u8)]
/// All the available UWB channels.
///
/// Note that while a channel may have more bandwidth than ~900 Mhz, the DW1000 can only send up to ~900 Mhz
pub enum UwbChannel {
    /// Channel 1
    /// - Center frequency: 3494.4 Mhz
    /// - Bandwidth: 499.2 Mhz
    Channel1 = 1,
    /// Channel 2
    /// - Center frequency: 3993.6 Mhz
    /// - Bandwidth: 499.2 Mhz
    Channel2 = 2,
    /// Channel 3
    /// - Center frequency: 4492.8 Mhz
    /// - Bandwidth: 499.2 Mhz
    Channel3 = 3,
    /// Channel 4
    /// - Center frequency: 3993.6 Mhz
    /// - Bandwidth: 1331.2 Mhz
    Channel4 = 4,
    /// Channel 5
    /// - Center frequency: 6489.6 Mhz
    /// - Bandwidth: 499.2 Mhz
    #[default]
    Channel5 = 5,
    /// Channel 7
    /// - Center frequency: 6489.6 Mhz
    /// - Bandwidth: 1081.6 Mhz
    Channel7 = 7,
}

impl UwbChannel {
    /// Gets the recommended preamble code
    pub fn get_recommended_preamble_code(&self, prf_value: PulseRepetitionFrequency) -> u8 {
        // Many have overlapping possibilities, so the numbers have been chosen so that there's no overlap here
        match (self, prf_value) {
            (UwbChannel::Channel1, PulseRepetitionFrequency::Mhz16) => 1,
            (UwbChannel::Channel2, PulseRepetitionFrequency::Mhz16) => 3,
            (UwbChannel::Channel3, PulseRepetitionFrequency::Mhz16) => 5,
            (UwbChannel::Channel4, PulseRepetitionFrequency::Mhz16) => 7,
            (UwbChannel::Channel5, PulseRepetitionFrequency::Mhz16) => 4,
            (UwbChannel::Channel7, PulseRepetitionFrequency::Mhz16) => 8,
            (UwbChannel::Channel1, PulseRepetitionFrequency::Mhz64) => 9,
            (UwbChannel::Channel2, PulseRepetitionFrequency::Mhz64) => 10,
            (UwbChannel::Channel3, PulseRepetitionFrequency::Mhz64) => 11,
            (UwbChannel::Channel4, PulseRepetitionFrequency::Mhz64) => 17,
            (UwbChannel::Channel5, PulseRepetitionFrequency::Mhz64) => 12,
            (UwbChannel::Channel7, PulseRepetitionFrequency::Mhz64) => 18,
        }
    }

    /// Get the recommended lde_repc register value
    pub fn get_recommended_lde_repc_value(
        &self,
        pulse_repetition_frequency: PulseRepetitionFrequency,
        bitrate: BitRate,
    ) -> u16 {
        // Values taken from user manual register description
        const VALUES: [u16; 24] = [
            0x5998, 0x5998, 0x51EA, 0x428E, 0x451E, 0x2E14, 0x8000, 0x51EA, 0x28F4, 0x3332, 0x3AE0,
            0x3D70, 0x3AE0, 0x35C2, 0x2B84, 0x35C2, 0x3332, 0x35C2, 0x35C2, 0x47AE, 0x3AE0, 0x3850,
            0x30A2, 0x3850,
        ];

        let pcode = self.get_recommended_preamble_code(pulse_repetition_frequency);
        let value = VALUES[pcode as usize];

        if bitrate != BitRate::Kbps110 {
            value
        } else {
            value / 8
        }
    }

    /// Gets the recommended value for the rf_txctrl register
    pub fn get_recommended_rf_txctrl(&self) -> u32 {
        // Values based on Table 38 of the DW1000 User Manual
        match self {
            UwbChannel::Channel1 => 0x00005C40,
            UwbChannel::Channel2 => 0x00045CA0,
            UwbChannel::Channel3 => 0x00086CC0,
            UwbChannel::Channel4 => 0x00045C80,
            UwbChannel::Channel5 => 0x001E3FE0,
            UwbChannel::Channel7 => 0x001E7DE0,
        }
    }

    /// Gets the recommended value for the tc_pgdelay register
    pub fn get_recommended_tc_pgdelay(&self) -> u8 {
        // Values based on Table 40 of the DW1000 User Manual
        match self {
            UwbChannel::Channel1 => 0xC9,
            UwbChannel::Channel2 => 0xC2,
            UwbChannel::Channel3 => 0xC5,
            UwbChannel::Channel4 => 0x95,
            UwbChannel::Channel5 => 0xC0,
            UwbChannel::Channel7 => 0x93,
        }
    }

    /// Gets the recommended value for the fs_pllcfg register
    pub fn get_recommended_fs_pllcfg(&self) -> u32 {
        // Values based on Table 43 of the DW1000 User Manual
        match self {
            UwbChannel::Channel1 => 0x09000407,
            UwbChannel::Channel2 | UwbChannel::Channel4 => 0x08400508,
            UwbChannel::Channel3 => 0x08401009,
            UwbChannel::Channel5 | UwbChannel::Channel7 => 0x0800041D,
        }
    }

    /// Gets the recommended value for the fs_plltune register
    pub fn get_recommended_fs_plltune(&self) -> u8 {
        // Values based on Table 44 of the DW1000 User Manual
        match self {
            UwbChannel::Channel1 => 0x1E,
            UwbChannel::Channel2 | UwbChannel::Channel4 => 0x26,
            UwbChannel::Channel3 => 0x56,
            UwbChannel::Channel5 | UwbChannel::Channel7 => 0xBE,
        }
    }

    /// Gets the recommended value for the rf_rxctrlh register
    pub fn get_recommended_rf_rxctrlh(&self) -> u8 {
        // Values based on Table 37 of the DW1000 User Manual
        match self {
            UwbChannel::Channel1
            | UwbChannel::Channel2
            | UwbChannel::Channel3
            | UwbChannel::Channel5 => 0xD8,
            UwbChannel::Channel4 | UwbChannel::Channel7 => 0xBC,
        }
    }
}
