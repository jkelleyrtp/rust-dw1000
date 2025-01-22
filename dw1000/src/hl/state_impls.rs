use crate::{configs::TxContinuation, time::Duration, RxConfig, TxConfig};

/// Indicates that the `DW1000` instance is not initialized yet
#[derive(Debug)]
pub struct Uninitialized;

/// Indicates that the `DW1000` instance is ready to be used
#[derive(Debug)]
pub struct Ready;

/// Indicates that the `DW1000` instance is currently sending
#[derive(Debug)]
pub struct Sending {
    pub(super) finished: bool,
    pub(super) continuation: TxContinuation,
    pub(super) config: TxConfig,
}

/// Indicates that the `DW1000` instance is currently receiving in single buffer mode (default)
#[derive(Debug)]
pub struct SingleBufferReceiving {
    pub(super) finished: bool,
    pub(super) config: RxConfig,
}

/// Indicates that the `DW1000` instance is currently receiving in double buffer mode
#[derive(Debug)]
pub struct AutoDoubleBufferReceiving {
    pub(super) finished: bool,
    pub(super) config: RxConfig,
}

/// Indicates that the `DW1000` instance is currently sleeping
#[derive(Debug)]
pub struct Sleeping {
    /// Tx antenna delay isn't stored in AON, so we'll do it ourselves.
    pub(super) tx_antenna_delay: Duration,
}

/// Any state struct that implements this trait signals that the radio is **not** sleeping.
pub trait Awake {}
impl Awake for Uninitialized {}
impl Awake for Ready {}
impl Awake for Sending {}
impl Awake for SingleBufferReceiving {}
impl Awake for AutoDoubleBufferReceiving {}
/// Any state struct that implements this trait signals that the radio is sleeping.
pub trait Asleep {}
impl Asleep for Sleeping {}

/// Any state struct that implements this trait shares a number of rx operations
pub trait Receiving: Awake {
    /// When true, the radio will re-enable the receive operation after it has received a message
    const AUTO_RX_REENABLE: bool;
    /// When true, the radio will use both receive buffers.
    /// This can help decrease the downtime between receiving messages.
    const DOUBLE_BUFFERED: bool;

    /// Mark the receiving state as finished
    fn mark_finished(&mut self);
    /// Return true if the receiving state has been marked as finished
    fn is_finished(&self) -> bool;
    /// Get the rx radio config
    fn get_rx_config(&self) -> &RxConfig;
}
impl Receiving for SingleBufferReceiving {
    const AUTO_RX_REENABLE: bool = false;
    const DOUBLE_BUFFERED: bool = false;

    fn mark_finished(&mut self) {
        self.finished = true;
    }

    fn is_finished(&self) -> bool {
        self.finished
    }

    fn get_rx_config(&self) -> &RxConfig {
        &self.config
    }
}
impl Receiving for AutoDoubleBufferReceiving {
    const AUTO_RX_REENABLE: bool = true;
    const DOUBLE_BUFFERED: bool = true;

    fn mark_finished(&mut self) {
        self.finished = true;
    }

    fn is_finished(&self) -> bool {
        self.finished
    }

    fn get_rx_config(&self) -> &RxConfig {
        &self.config
    }
}
