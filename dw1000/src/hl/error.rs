use crate::ll;
use core::fmt;
use embedded_hal::spi::SpiDevice;
use ssmarshal;

/// An error that can occur when sending or receiving data
pub enum Error<SPI>
where
    SPI: SpiDevice,
{
    /// Error occured while using SPI bus
    Spi(SPI::Error),

    /// Receiver FCS error
    Fcs,

    /// PHY header error
    Phy,

    /// Buffer too small
    BufferTooSmall {
        /// Indicates how large a buffer would have been required
        required_len: usize,
    },

    /// Receiver Reed Solomon Frame Sync Loss
    ReedSolomon,

    /// Receiver Frame Wait Timeout
    FrameWaitTimeout,

    /// Receiver Overrun
    Overrun,

    /// Preamble Detection Timeout
    PreambleDetectionTimeout,

    /// Receiver SFD Timeout
    SfdTimeout,

    /// Frame was rejected because due to automatic frame filtering
    ///
    /// It seems that frame filtering is typically handled transparently by the
    /// hardware, and filtered frames aren't usually visible to the driver.
    /// However, sometimes a filtered frame bubbles up and disrupts an ongoing
    /// receive operation, which then causes this error.
    FrameFilteringRejection,

    /// Frame could not be decoded
    Frame(byte::Error),

    /// A delayed frame could not be sent in time
    ///
    /// Please note that the frame was still sent. Replies could still arrive,
    /// and if it was a ranging frame, the resulting range measurement will be
    /// wrong.
    DelayedSendTooLate,

    /// Transmitter could not power up in time for delayed send
    ///
    /// The frame was still transmitted, but the first bytes of the preamble
    /// were likely corrupted.
    DelayedSendPowerUpWarning,

    /// An error occured while serializing or deserializing data
    Ssmarshal(ssmarshal::Error),

    /// The configuration was not valid. Some combinations of settings are not allowed.
    InvalidConfiguration,

    /// The receive operation hasn't finished yet
    RxNotFinished,

    /// It was expected that the radio would have woken up, but it hasn't.
    StillAsleep,

    /// The RSSI was not calculable.
    BadRssiCalculation,

    /// There are issues with frame filtering in double buffer mode.
    /// So it's not supported now.
    RxConfigFrameFilteringUnsupported,

    /// The wrong continuation was called on the radio
    WrongTxContinuation,

    /// The transmission has not yet finished
    TxNotFinishedyet,
}

impl<SPI> From<ll::Error<SPI>> for Error<SPI>
where
    SPI: SpiDevice,
{
    fn from(error: ll::Error<SPI>) -> Self {
        Error::Spi(error.0)
    }
}

impl<SPI> From<ssmarshal::Error> for Error<SPI>
where
    SPI: SpiDevice,
{
    fn from(error: ssmarshal::Error) -> Self {
        Error::Ssmarshal(error)
    }
}

// We can't derive this implementation, as `Debug` is only implemented
// conditionally for `ll::Debug`.
impl<SPI> fmt::Debug for Error<SPI>
where
    SPI: SpiDevice,
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Error::Spi(error) => write!(f, "Spi({:?})", error),
            Error::Fcs => write!(f, "Fcs"),
            Error::Phy => write!(f, "Phy"),
            Error::BufferTooSmall { required_len } => {
                write!(f, "BufferTooSmall {{ required_len: {:?} }}", required_len,)
            }
            Error::ReedSolomon => write!(f, "ReedSolomon"),
            Error::FrameWaitTimeout => write!(f, "FrameWaitTimeout"),
            Error::Overrun => write!(f, "Overrun"),
            Error::PreambleDetectionTimeout => write!(f, "PreambleDetectionTimeout"),
            Error::SfdTimeout => write!(f, "SfdTimeout"),
            Error::FrameFilteringRejection => write!(f, "FrameFilteringRejection"),
            Error::Frame(error) => write!(f, "Frame({:?})", error),
            Error::DelayedSendTooLate => write!(f, "DelayedSendTooLate"),
            Error::DelayedSendPowerUpWarning => write!(f, "DelayedSendPowerUpWarning"),
            Error::Ssmarshal(error) => write!(f, "Ssmarshal({:?})", error),
            Error::InvalidConfiguration => write!(f, "InvalidConfiguration"),
            Error::RxNotFinished => write!(f, "RxNotFinished"),
            Error::StillAsleep => write!(f, "StillAsleep"),
            Error::BadRssiCalculation => write!(f, "BadRssiCalculation"),
            Error::RxConfigFrameFilteringUnsupported => {
                write!(f, "RxConfigFrameFilteringUnsupported")
            }
            Error::WrongTxContinuation => {
                write!(f, "WrongTxContinuation")
            }
            Error::TxNotFinishedyet => {
                write!(f, "TxNotFinishedyet")
            }
        }
    }
}
