//! Low-level interface to the DW1000
//!
//! This module implements a register-level interface to the DW1000. Users of
//! this library should typically not need to use this. Please consider using
//! the [high-level interface] instead.
//!
//! If you're using the low-level interface because the high-level interface
//! doesn't cover your use case, please consider [filing an issue].
//!
//! **NOTE**: Many field access methods accept types that have a larger number
//! of bits than the field actually consists of. If you use such a method to
//! pass a value that is too large to be written to the field, it will be
//! silently truncated.
//!
//! [high-level interface]: ../hl/index.html
//! [filing an issue]: https://github.com/braun-robotics/rust-dw1000/issues/new

use core::marker::PhantomData;

use embedded_hal::spi::{Operation, SpiDevice};

/// Entry point to the DW1000 driver's low-level API
///
/// Please consider using [hl::DW1000] instead.
///
/// [hl::DW1000]: ../hl/struct.DW1000.html
pub struct DW1000<SPI> {
    spi: Option<SPI>,
}

impl<SPI: SpiDevice> DW1000<SPI> {
    /// Create a new instance of `DW1000`
    ///
    /// Requires the SPI peripheral and the chip select pin that are connected
    /// to the DW1000.
    pub fn new(spi: SPI) -> Self {
        DW1000 { spi: Some(spi) }
    }

    /// Read a whole block of data
    ///
    /// The buffer must have the first 3 bytes free so the protocol header can be put there.
    /// The rest of the buffer will contain the actual data.
    ///
    /// The result contains the slice of the buffer that contains the actual data.
    fn block_read<'a>(
        &mut self,
        id: u8,
        start_sub_id: u16,
        buffer: &'a mut [u8],
    ) -> Result<&'a mut [u8], Error<SPI>> {
        // Make it simple and use the 3 byte header
        buffer[0] = (((start_sub_id as u8) << 6) & 0x40) | (id & 0x3f);
        buffer[1] = 0x80 | (start_sub_id & 0x7F) as u8;
        buffer[2] = ((start_sub_id & 0x7f80) >> 7) as u8;

        // Read the data
        self.spi().transfer_in_place(buffer).map_err(Error)?;

        Ok(&mut buffer[3..])
    }

    /// Reads the CIR accumulator.
    ///
    /// Starts reading from the start_index and puts all results in the buffer.
    /// A slice with the CIR data is returned
    ///
    /// *NOTE: The buffer passed in must be 4 bytes bigger than the cir buffer slice you want to get back due to protocol*
    pub fn cir<'a>(
        &mut self,
        start_index: u16,
        buffer: &'a mut [u8],
    ) -> Result<&'a mut [u8], Error<SPI>> {
        let block = self.block_read(0x25, start_index, buffer)?;
        Ok(&mut block[1..])
    }

    /// Allows for an access to the spi type.
    /// This can be used to change the speed.
    ///
    /// In closure you get ownership of the SPI
    /// so you can destruct it and build it up again if necessary.
    pub fn access_spi<F>(&mut self, f: F)
    where
        F: FnOnce(SPI) -> SPI,
    {
        // Get the spi in the struct.
        let spi = self.spi.take().expect("spi should always be set");
        // Give the spi to the closure and put the result back into the struct.
        self.spi = Some(f(spi));
    }

    fn spi(&mut self) -> &mut SPI {
        self.spi.as_mut().expect("spi should always be set")
    }

    /// Deasserts CS, waits the given amount of µs, and reasserts CS
    pub(crate) fn wake_up(&mut self, wait_time_us: u32) -> Result<(), Error<SPI>> {
        self.spi()
            .transaction(&mut [Operation::DelayNs(wait_time_us * 1000)])
            .map_err(Error)
    }
}

/// Provides access to a register
///
/// You can get an instance for a given register using one of the methods on
/// [`DW1000`].
pub struct RegAccessor<'s, R, SPI>(&'s mut DW1000<SPI>, PhantomData<R>);

impl<'s, R, SPI> RegAccessor<'s, R, SPI>
where
    SPI: SpiDevice,
{
    /// Read from the register
    pub fn read(&mut self) -> Result<R::Read, Error<SPI>>
    where
        R: Register + Readable,
    {
        let mut r = R::read();
        let buffer = R::buffer(&mut r);

        init_header::<R>(false, buffer);

        self.0.spi().transfer_in_place(buffer).map_err(Error)?;

        Ok(r)
    }

    /// Write to the register
    pub fn write<F>(&mut self, f: F) -> Result<(), Error<SPI>>
    where
        R: Register + Writable,
        F: FnOnce(&mut R::Write) -> &mut R::Write,
    {
        let mut w = R::write();
        f(&mut w);

        let buffer = R::buffer(&mut w);
        init_header::<R>(true, buffer);

        self.0.spi().write(buffer).map_err(Error)?;

        Ok(())
    }

    /// Modify the register
    pub fn modify<F>(&mut self, f: F) -> Result<(), Error<SPI>>
    where
        R: Register + Readable + Writable,
        F: for<'r> FnOnce(&mut R::Read, &'r mut R::Write) -> &'r mut R::Write,
    {
        let mut r = self.read()?;
        let mut w = R::write();

        <R as Writable>::buffer(&mut w).copy_from_slice(<R as Readable>::buffer(&mut r));

        f(&mut r, &mut w);

        let buffer = <R as Writable>::buffer(&mut w);
        init_header::<R>(true, buffer);

        self.0.spi().write(buffer).map_err(Error)?;

        Ok(())
    }
}

/// An SPI error that can occur when communicating with the DW1000
#[derive(Debug)]
#[repr(transparent)]
pub struct Error<SPI: SpiDevice>(pub SPI::Error);

/// Initializes the SPI message header
///
/// Initializes the SPI message header for accessing a given register, writing
/// the header directly into the provided buffer. Returns the length of the
/// header that was written.
fn init_header<R: Register>(write: bool, buffer: &mut [u8]) -> usize {
    let sub_id = R::SUB_ID > 0;

    buffer[0] = (((write as u8) << 7) & 0x80) | (((sub_id as u8) << 6) & 0x40) | (R::ID & 0x3f);

    if !sub_id {
        return 1;
    }

    let ext_addr = R::SUB_ID > 127;

    buffer[1] = (((ext_addr as u8) << 7) & 0x80) | (R::SUB_ID as u8 & 0x7f); // lower 7 bits (of 15)

    if !ext_addr {
        return 2;
    }

    buffer[2] = ((R::SUB_ID & 0x7f80) >> 7) as u8; // higher 8 bits (of 15)

    3
}

/// Implemented for all registers
///
/// This is a mostly internal crate that should not be implemented or used
/// directly by users of this crate. It is exposed through the public API
/// though, so it can't be made private.
///
/// The DW1000 user manual, section 7.1, specifies what the values of the
/// constant should be for each register.
pub trait Register {
    /// The register index
    const ID: u8;

    /// The registers's sub-index
    const SUB_ID: u16;

    /// The lenght of the register
    const LEN: usize;
}

/// Marker trait for registers that can be read from
///
/// This is a mostly internal crate that should not be implemented or used
/// directly by users of this crate. It is exposed through the public API
/// though, so it can't be made private.
pub trait Readable {
    /// The type that is used to read from the register
    type Read;

    /// Return the read type for this register
    fn read() -> Self::Read;

    /// Return the read type's internal buffer
    fn buffer(r: &mut Self::Read) -> &mut [u8];
}

/// Marker trait for registers that can be written to
///
/// This is a mostly internal crate that should not be implemented or used
/// directly by users of this crate. It is exposed through the public API
/// though, so it can't be made private.
pub trait Writable {
    /// The type that is used to write to the register
    type Write;

    /// Return the write type for this register
    fn write() -> Self::Write;

    /// Return the write type's internal buffer
    fn buffer(w: &mut Self::Write) -> &mut [u8];
}

/// Generates register implementations
macro_rules! impl_register {
    (
        $(
            $id:expr,
            $sub_id:expr,
            $len:expr,
            $rw:tt,
            $name:ident($name_lower:ident) {
            #[$doc:meta]
            $(
                $field:ident,
                $first_bit:expr,
                $last_bit:expr,
                $ty:ty;
                #[$field_doc:meta]
            )*
            }
        )*
    ) => {
        $(
            #[$doc]
            #[allow(non_camel_case_types)]
            pub struct $name;

            impl Register for $name {
                const ID:     u8    = $id;
                const SUB_ID: u16   = $sub_id;
                const LEN:    usize = $len;
            }

            impl $name {
                // You know what would be neat? Using `if` in constant
                // expressions! But that's not possible, so we're left with the
                // following hack.
                const SUB_INDEX_IS_NONZERO: usize =
                    (Self::SUB_ID > 0) as usize;
                const SUB_INDEX_NEEDS_SECOND_BYTE: usize =
                    (Self::SUB_ID > 127) as usize;
                const HEADER_LEN: usize =
                    1
                    + Self::SUB_INDEX_IS_NONZERO
                    + Self::SUB_INDEX_NEEDS_SECOND_BYTE;
            }

            #[$doc]
            pub mod $name_lower {
                use core::fmt;


                const HEADER_LEN: usize = super::$name::HEADER_LEN;


                /// Used to read from the register
                pub struct R(pub(crate) [u8; HEADER_LEN + $len]);

                impl R {
                    $(
                        #[$field_doc]
                        pub fn $field(&self) -> $ty {
                            use core::mem::size_of;
                            use crate::ll::FromBytes;

                            // The index (in the register data) of the first
                            // byte that contains a part of this field.
                            const START: usize = $first_bit / 8;

                            // The index (in the register data) of the byte
                            // after the last byte that contains a part of this
                            // field.
                            const END: usize = $last_bit  / 8 + 1;

                            // The numer of bytes in the register data that
                            // contain part of this field.
                            const LEN: usize = END - START;

                            // Get all bytes that contain our field. The field
                            // might fill out these bytes completely, or only
                            // some bits in them.
                            let mut bytes = [0; LEN];
                            bytes[..LEN].copy_from_slice(
                                &self.0[START+HEADER_LEN .. END+HEADER_LEN]
                            );

                            // Before we can convert the field into a number and
                            // return it, we need to shift it, to make sure
                            // there are no other bits to the right of it. Let's
                            // start by determining the offset of the field
                            // within a byte.
                            const OFFSET_IN_BYTE: usize = $first_bit % 8;

                            if OFFSET_IN_BYTE > 0 {
                                // Shift the first byte. We always have at least
                                // one byte here, so this always works.
                                bytes[0] >>= OFFSET_IN_BYTE;

                                // If there are more bytes, let's shift those
                                // too.
                                // We need to allow exceeding bitshifts (arithmetic_overflow)
                                // in this loop, as we run into that if `OFFSET_IN_BYTE`
                                // equals `0`. Please note that we never
                                // actually encounter that at runtime, due to
                                // the if condition above.
                                let mut i = 1;
                                #[allow(arithmetic_overflow)]
                                while i < LEN {
                                    bytes[i - 1] |=
                                        bytes[i] << 8 - OFFSET_IN_BYTE;
                                    bytes[i] >>= OFFSET_IN_BYTE;
                                    i += 1;
                                }
                            }

                            // If the field didn't completely fill out its last
                            // byte, we might have bits from unrelated fields
                            // there. Let's erase those before doing the final
                            // conversion into the field's data type.
                            const SIZE_IN_BITS: usize =
                                $last_bit - $first_bit + 1;
                            const BITS_ABOVE_FIELD: usize =
                                8 - (SIZE_IN_BITS % 8);
                            const SIZE_IN_BYTES: usize =
                                (SIZE_IN_BITS - 1) / 8 + 1;
                            const LAST_INDEX: usize =
                                SIZE_IN_BYTES - 1;
                            if BITS_ABOVE_FIELD < 8 {
                                // Need to allow exceeding bitshifts (arithmetic_overflow)
                                // to make the compiler happy. They're never actually
                                // encountered at runtime, due to the if
                                // condition.
                                #[allow(arithmetic_overflow)]
                                {
                                    bytes[LAST_INDEX] <<= BITS_ABOVE_FIELD;
                                    bytes[LAST_INDEX] >>= BITS_ABOVE_FIELD;
                                }
                            }

                            // Now all that's left is to convert the bytes into
                            // the field's type. Please note that methods for
                            // converting numbers to/from bytes are coming to
                            // stable Rust, so we might be able to remove our
                            // custom infrastructure here. Tracking issue:
                            // https://github.com/rust-lang/rust/issues/52963
                            let bytes = if bytes.len() > size_of::<$ty>() {
                                &bytes[..size_of::<$ty>()]
                            }
                            else {
                                &bytes
                            };
                            <$ty as FromBytes>::from_bytes(bytes)
                        }
                    )*
                }

                impl fmt::Debug for R {
                    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
                        write!(f, "0x")?;
                        for i in (0 .. $len).rev() {
                            write!(f, "{:02x}", self.0[HEADER_LEN + i])?;
                        }

                        Ok(())
                    }
                }


                /// Used to write to the register
                pub struct W(pub(crate) [u8; HEADER_LEN + $len]);

                impl W {
                    $(
                        #[$field_doc]
                        pub fn $field(&mut self, value: $ty) -> &mut Self {
                            use crate::ll::ToBytes;

                            // Convert value into bytes
                            let source = <$ty as ToBytes>::to_bytes(value);

                            // Now, let's figure out where the bytes are located
                            // within the register array.
                            const START:          usize = $first_bit / 8;
                            const END:            usize = $last_bit  / 8 + 1;
                            const OFFSET_IN_BYTE: usize = $first_bit % 8;

                            // Also figure out the length of the value in bits.
                            // That's going to come in handy.
                            const LEN: usize = $last_bit - $first_bit + 1;


                            // We need to track how many bits are left in the
                            // value overall, and in the value's current byte.
                            let mut bits_left         = LEN;
                            let mut bits_left_in_byte = 8;

                            // We also need to track how many bits have already
                            // been written to the current target byte.
                            let mut bits_written_to_byte = 0;

                            // Now we can take the bytes from the value, shift
                            // them, mask them, and write them into the target
                            // array.
                            let mut source_i  = 0;
                            let mut target_i  = START;
                            while target_i < END {
                                // Values don't always end at byte boundaries,
                                // so we need to mask the bytes when writing to
                                // the slice.
                                // Let's start out assuming we can write to the
                                // whole byte of the slice. This will be true
                                // for the middle bytes of our value.
                                let mut mask = 0xff;

                                // Let's keep track of the offset we're using to
                                // write to this byte. We're going to need it.
                                let mut offset_in_this_byte = 0;

                                // If this is the first byte we're writing to
                                // the slice, we need to remove the lower bits
                                // of the mask.
                                if target_i == START {
                                    mask <<= OFFSET_IN_BYTE;
                                    offset_in_this_byte = OFFSET_IN_BYTE;
                                }

                                // If this is the last byte we're writing to the
                                // slice, we need to remove the higher bits of
                                // the mask. Please note that we could be
                                // writing to _both_ the first and the last
                                // byte.
                                if target_i == END - 1 {
                                    let shift =
                                        8 - bits_left - offset_in_this_byte;
                                    mask <<= shift;
                                    mask >>= shift;
                                }

                                mask <<= bits_written_to_byte;

                                // Read the value from `source`
                                let value = source[source_i]
                                    >> 8 - bits_left_in_byte
                                    << offset_in_this_byte
                                    << bits_written_to_byte;

                                // Zero the target bits in the slice, then write
                                // the value.
                                self.0[HEADER_LEN + target_i] &= !mask;
                                self.0[HEADER_LEN + target_i] |= value & mask;

                                // The number of bits that were expected to be
                                // written to the target byte.
                                let bits_needed = mask.count_ones() as usize;

                                // The number of bits we actually wrote to the
                                // target byte.
                                let bits_used = bits_needed.min(
                                    bits_left_in_byte - offset_in_this_byte
                                );

                                bits_left -= bits_used;
                                bits_written_to_byte += bits_used;

                                // Did we use up all the bits in the source
                                // byte? If so, we can move on to the next one.
                                if bits_left_in_byte > bits_used {
                                    bits_left_in_byte -= bits_used;
                                }
                                else {
                                    bits_left_in_byte =
                                        8 - (bits_used - bits_left_in_byte);

                                    source_i += 1;
                                }

                                // Did we write all the bits in the target byte?
                                // If so, we can move on to the next one.
                                if bits_used == bits_needed {
                                    target_i += 1;
                                    bits_written_to_byte = 0;
                                }
                            }

                            self
                        }
                    )*
                }
            }

            impl_rw!($rw, $name, $name_lower, $len);
        )*


        impl<SPI> DW1000<SPI> {
            $(
                #[$doc]
                pub fn $name_lower(&mut self) -> RegAccessor<$name, SPI> {
                    RegAccessor(self, PhantomData)
                }
            )*
        }
    }
}

// Helper macro, used internally by `impl_register!`
macro_rules! impl_rw {
    (RO, $name:ident, $name_lower:ident, $len:expr) => {
        impl_rw!(@R, $name, $name_lower, $len);
    };
    (RW, $name:ident, $name_lower:ident, $len:expr) => {
        impl_rw!(@R, $name, $name_lower, $len);
        impl_rw!(@W, $name, $name_lower, $len);
    };

    (@R, $name:ident, $name_lower:ident, $len:expr) => {
        impl Readable for $name {
            type Read = $name_lower::R;

            fn read() -> Self::Read {
                $name_lower::R([0; Self::HEADER_LEN + $len])
            }

            fn buffer(r: &mut Self::Read) -> &mut [u8] {
                &mut r.0
            }
        }
    };
    (@W, $name:ident, $name_lower:ident, $len:expr) => {
        impl Writable for $name {
            type Write = $name_lower::W;

            fn write() -> Self::Write {
                $name_lower::W([0; Self::HEADER_LEN + $len])
            }

            fn buffer(w: &mut Self::Write) -> &mut [u8] {
                &mut w.0
            }
        }
    };
}

// All register are implemented in this macro invocation. It follows the
// following syntax:
// <id>, <sub-id>, <size-bytes>, <RO/RW>, <name-upper>(name-lower) { /// <doc>
//     <field 1>
//     <field 2>
//     ...
// }
//
// Each field follows the following syntax:
// <name>, <first-bit-index>, <last-bit-index>, <type>; /// <doc>
impl_register! {
    0x00, 0x00, 4, RO, DEV_ID(dev_id) { /// Device identifier
        rev,     0,  3, u8;  /// Revision
        ver,     4,  7, u8;  /// Version
        model,   8, 15, u8;  /// Model
        ridtag, 16, 31, u16; /// Register Identification Tag
    }
    0x01, 0x00, 8, RW, EUI(eui) { /// Extended Unique Identifier
        value, 0, 63, u64; /// Extended Unique Identifier
    }
    0x03, 0x00, 4, RW, PANADR(panadr) { /// PAN Identifier and Short Address
        short_addr,  0, 15, u16; /// Short Address
        pan_id,     16, 31, u16; /// PAN Identifier
    }
    0x04, 0x00, 4, RW, SYS_CFG(sys_cfg) { /// System Configuration
        ffen,        0,  0, u8; /// Frame Filtering Enable
        ffbc,        1,  1, u8; /// Frame Filtering Behave As Coordinator
        ffab,        2,  2, u8; /// Frame Filtering Allow Beacon
        ffad,        3,  3, u8; /// Frame Filtering Allow Data
        ffaa,        4,  4, u8; /// Frame Filtering Allow Acknowledgement
        ffam,        5,  5, u8; /// Frame Filtering Allow MAC Command Frame
        ffar,        6,  6, u8; /// Frame Filtering Allow Reserved
        ffa4,        7,  7, u8; /// Frame Filtering Allow Frame Type 4
        ffa5,        8,  8, u8; /// Frame Filtering Allow Frame Type 5
        hirq_pol,    9,  9, u8; /// Host Interrupt Polarity
        spi_edge,   10, 10, u8; /// SPI Data Launch Edge
        dis_fce,    11, 11, u8; /// Disable Frame Check Error Handling
        dis_drxb,   12, 12, u8; /// Disable Double RX Buffer
        dis_phe,    13, 13, u8; /// Disable Receiver Abort on PHR Error
        dis_rsde,   14, 14, u8; /// Disable Receiver Abort on RSD Error
        fcs_init2f, 15, 15, u8; /// FCS Seed Selection
        phr_mode,   16, 17, u8; /// PHR Mode
        dis_stxp,   18, 18, u8; /// Disable Smart TX Power Control
        rxm110k,    22, 22, u8; /// Receiver Mode 110kpbs Data Rate
        rxwtoe,     28, 28, u8; /// Receiver Wait Timeout Enable
        rxautr,     29, 29, u8; /// Receiver Auto-Re-Enable
        autoack,    30, 30, u8; /// Automatic Acknowledgement Enable
        aackpend,   31, 31, u8; /// Automatic Acknowledgement Pending
    }
    0x06, 0x00, 5, RO, SYS_TIME(sys_time) { /// System Time Counter
        value, 0, 39, u64; /// System Time Counter
    }
    0x08, 0x00, 5, RW, TX_FCTRL(tx_fctrl) { /// TX Frame Control
        tflen,     0,  6, u8;  /// TX Frame Length
        tfle,      7,  9, u8;  /// TX Frame Length Extension
        txbr,     13, 14, u8;  /// TX Bit Rate
        tr,       15, 15, u8;  /// TX Ranging Enable
        txprf,    16, 17, u8;  /// TX Pulse Repetition Frequency
        txpsr,    18, 19, u8;  /// TX Preamble Symbol Repetitions
        pe,       20, 21, u8;  /// Preamble Extension
        txboffs,  22, 31, u16; /// TX Buffer Index Offset
        ifsdelay, 32, 39, u8;  /// Inter-Frame Spacing
    }
    0x0A, 0x00, 5, RW, DX_TIME(dx_time) { /// Delayed Send or Receive Time
        value, 0, 39, u64; /// Delayed Send or Receive Time
    }
    0x0D, 0x00, 4, RW, SYS_CTRL(sys_ctrl) { /// System Control Register
        sfcst,      0,  0, u8; /// Suppress Auto-FCS Transmission
        txstrt,     1,  1, u8; /// Transmit Start
        txdlys,     2,  2, u8; /// Transmitter Delayed Sending
        cansfcs,    3,  3, u8; /// Cancel Auto-FCS Suppression
        trxoff,     6,  6, u8; /// Transceiver Off
        wait4resp,  7,  7, u8; /// Wait for Response
        rxenab,     8,  8, u8; /// Enable Receiver
        rxdlye,     9,  9, u8; /// Receiver Delayed Enable
        hrbpt,     24, 24, u8; /// Host Side RX Buffer Pointer Toggle
    }
    0x0E, 0x00, 4, RW, SYS_MASK(sys_mask) { /// System Event Mask Register
        mcplock,    1,  1, u8; /// Mask clock PLL lock
        mesyncr,    2,  2, u8; /// Mask external sync clock reset
        maat,       3,  3, u8; /// Mask automatic acknowledge trigger
        mtxfrbm,    4,  4, u8; /// Mask transmit frame begins
        mtxprs,     5,  5, u8; /// Mask transmit preamble sent
        mtxphs,     6,  6, u8; /// Mask transmit PHY Header Sent
        mtxfrs,     7,  7, u8; /// Mask transmit frame sent
        mrxprd,     8,  8, u8; /// Mask receiver preamble detected
        mrxsfdd,    9,  9, u8; /// Mask receiver SFD detected
        mldedone,  10, 10, u8; /// Mask LDE processing done
        mrxphd,    11, 11, u8; /// Mask receiver PHY header detect
        mrxphe,    12, 12, u8; /// Mask receiver PHY header error
        mrxdfr,    13, 13, u8; /// Mask receiver data frame ready
        mrxfcg,    14, 14, u8; /// Mask receiver FCS good
        mrxfce,    15, 15, u8; /// Mask receiver FCS error
        mrxrfsl,   16, 16, u8; /// Mask receiver Reed Solomon Frame Sync loss
        mrxrfto,   17, 17, u8; /// Mask Receive Frame Wait Timeout
        mldeerr,   18, 18, u8; /// Mask leading edge detection processing error
        mrxovrr,   20, 20, u8; /// Mask Receiver Overrun
        mrxpto,    21, 21, u8; /// Mask Preamble detection timeout
        mgpioirq,  22, 22, u8; /// Mask GPIO interrupt
        mslp2init, 23, 23, u8; /// Mask SLEEP to INIT event
        mrfpllll,  24, 24, u8; /// Mask RF PLL Losing Lock warning
        mcpllll,   25, 25, u8; /// Mask Clock PLL Losing Lock warning
        mrxsfdto,  26, 26, u8; /// Mask Receive SFD timeout
        mhpdwarn,  27, 27, u8; /// Mask Half Period Delay Warning
        mtxberr,   28, 28, u8; /// Mask Transmit Buffer Error
        maffrej,   29, 29, u8; /// Mask Automatic Frame Filtering rejection
    }
    0x0F, 0x00, 5, RW, SYS_STATUS(sys_status) { /// System Event Status Register
        irqs,       0,  0, u8; /// Interrupt Request Status
        cplock,     1,  1, u8; /// Clock PLL Lock
        esyncr,     2,  2, u8; /// External Sync Clock Reset
        aat,        3,  3, u8; /// Automatic Acknowledge Trigger
        txfrb,      4,  4, u8; /// TX Frame Begins
        txprs,      5,  5, u8; /// TX Preamble Sent
        txphs,      6,  6, u8; /// TX PHY Header Sent
        txfrs,      7,  7, u8; /// TX Frame Sent
        rxprd,      8,  8, u8; /// RX Preamble Detected
        rxsfdd,     9,  9, u8; /// RX SFD Detected
        ldedone,   10, 10, u8; /// LDE Processing Done
        rxphd,     11, 11, u8; /// RX PHY Header Detect
        rxphe,     12, 12, u8; /// RX PHY Header Error
        rxdfr,     13, 13, u8; /// RX Data Frame Ready
        rxfcg,     14, 14, u8; /// RX FCS Good
        rxfce,     15, 15, u8; /// RX FCS Error
        rxrfsl,    16, 16, u8; /// RX Reed-Solomon Frame Sync Loss
        rxrfto,    17, 17, u8; /// RX Frame Wait Timeout
        ldeerr,    18, 18, u8; /// Leading Edge Detection Error
        rxovrr,    20, 20, u8; /// RX Overrun
        rxpto,     21, 21, u8; /// Preamble Detection Timeout
        gpioirq,   22, 22, u8; /// GPIO Interrupt
        slp2init,  23, 23, u8; /// SLEEP to INIT
        rfpll_ll,  24, 24, u8; /// RF PLL Losing Lock
        clkpll_ll, 25, 25, u8; /// Clock PLL Losing Lock
        rxsfdto,   26, 26, u8; /// Receive SFD Timeout
        hpdwarn,   27, 27, u8; /// Half Period Delay Warning
        txberr,    28, 28, u8; /// TX Buffer Error
        affrej,    29, 29, u8; /// Auto Frame Filtering Rejection
        hsrbp,     30, 30, u8; /// Host Side RX Buffer Pointer
        icrbp,     31, 31, u8; /// IC Side RX Buffer Pointer
        rxrscs,    32, 32, u8; /// RX Reed-Solomon Correction Status
        rxprej,    33, 33, u8; /// RX Preamble Rejection
        txpute,    34, 34, u8; /// TX Power Up Time Error
    }
    0x10, 0x00, 4, RO, RX_FINFO(rx_finfo) { /// RX Frame Information
        rxflen,  0,  6, u8; /// Receive Frame Length
        rxfle,   7,  9, u8; /// Receive Frame Length Extension
        rxnspl, 11, 12, u8; /// Receive Non-Standard Preamble Length
        rxbr,   13, 14, u8; /// Receive Bit Rate Report
        rng,    15, 15, u8; /// Receiver Ranging
        rxprfr, 16, 17, u8; /// RX Pulse Repetition Rate Report
        rxpsr,  18, 19, u8; /// RX Preamble Repetition
        rxpacc, 20, 31, u16; /// Preamble Accumulation Count
    }
    0x12, 0x00, 8, RO, RX_FQUAL(rx_fqual) { /// Rx Frame Quality Information
        std_noise, 0, 15, u16; /// Standard Deviation of Noise
        fp_ampl2, 16, 31, u16; /// First Path Amplitude point 2
        fp_ampl3, 32, 47, u16; /// First Path Amplitude point 3
        cir_pwr,  48, 63, u16; /// Channel Impulse Response Power
    }
    0x13, 0x00, 4, RO, RX_TTCKI(rx_ttcki) { /// Receiver Time Tracking Interval
        value, 0, 31, u32; /// Value of the register
    }
    0x14, 0x00, 5, RO, RX_TTCKO(rx_ttcko) { /// Receiver Time Tracking Offset
        rxtofs,   0, 18, u32; /// RX time tracking offset (19-bit signed int)
        rsmpdel, 24, 31, u8;  /// Internal re-sampler delay value
        rcphase, 32, 39, u8;  /// Receive carrier phase adjustment
    }
    0x15, 0x00, 14, RO, RX_TIME(rx_time) { /// Receive Time Stamp
        rx_stamp,  0,  39, u64; /// Fully adjusted time stamp
        fp_index, 40,  55, u16; /// First Path Index
        fp_ampl1, 56,  71, u16; /// First Path Amplitude Point 1
        rx_rawst, 72, 111, u64; /// Raw time stamp
    }
    0x17, 0x00, 10, RO, TX_TIME(tx_time) { /// Transmit Time Stamp
        tx_stamp,  0, 39, u64; /// Fully adjusted time stamp
        tx_rawst, 40, 79, u64; /// Raw time stamp
    }
    0x18, 0x00, 2, RW, TX_ANTD(tx_antd) { /// TX Antenna Delay
        value, 0, 15, u16; /// TX Antenna Delay
    }
    0x19, 0x00, 5, RO, SYS_STATE(sys_state) { /// System State information
        tx_state,    0,  3, u8; /// Current Transmit State Machine value
        rx_state,    8, 12, u8; /// Current Receive State Machine value
        pmsc_state, 16, 23, u8; /// Current PMSC State Machine value
    }
    0x1E, 0x00, 4, RW, TX_POWER(tx_power) { /// TX Power Control
        // The TX_POWER register has multiple sets of fields defined, depending
        // on the smart TX power control setting. I don't know how to model
        // this, so I've opted to provide just a single `value` field for
        // maximum flexibility.
        value, 0, 31, u32; /// TX Power Control value
    }
    0x1F, 0x00, 4, RW, CHAN_CTRL(chan_ctrl) { /// Channel Control Register
        tx_chan, 0, 3, u8; /// Selects the transmit channel.
        rx_chan, 4, 7, u8; /// Selects the receive channel.
        dwsfd, 17, 17, u8; /// Enables the non-standard Decawave proprietary SFD sequence.
        rxprf, 18, 19, u8; /// Selects the PRF used in the receiver.
        tnssfd, 20, 20, u8; /// This bit enables the use of a user specified (non-standard) SFDin the transmitter.
        rnssfd, 21, 21, u8; /// This bit enables the use of a user specified (non-standard) SFDin the receiver.
        tx_pcode, 22, 26, u8; /// This field selects the preamble code used in the transmitter.
        rx_pcode, 27, 31, u8; /// This field selects the preamble code used in the receiver.
    }
    0x21, 0x00, 1, RW, SFD_LENGTH(sfd_length) { /// This is the length of the SFD sequence used when the data rate is 850kbps and higher.
        value, 0, 7, u8; /// This is the length of the SFD sequence used when the data rate is 850kbps and higher.
    }
    0x23, 0x04, 2, RW, AGC_TUNE1(agc_tune1) { /// AGC Tuning register 1
        value, 0, 15, u16; /// AGC Tuning register 1 value
    }
    0x23, 0x0C, 4, RW, AGC_TUNE2(agc_tune2) { /// AGC Tuning register 2
        value, 0, 31, u32; /// AGC Tuning register 2 value
    }
    0x24, 0x00, 4, RW, EC_CTRL(ec_ctrl) { /// External Clock Sync Counter Config
        ostsm,   0,  0, u8; /// External Transmit Synchronization Mode Enable
        osrsm,   1,  1, u8; /// External Receive Synchronization Mode Enable
        pllldt,  2,  2, u8; /// Clock PLL Lock Detect Tune
        wait,    3, 10, u8; /// Wait Counter
        ostrm,  11, 11, u8; /// External Timebase Reset Mode Enable
    }
    0x24, 0x04, 4, RO, EC_RXTC(ec_rxtc) { /// External clock synchronisation counter captured on RMARKER
        rx_ts_est, 0, 31, u32; /// External clock synchronisation counter captured on RMARKER
    }
    0x24, 0x08, 4, RO, EC_GOLP(ec_golp) { /// External clock offset to first path 1 GHz counter
        offset_ext, 0, 5, u8; /// This register contains the 1 GHz count from the arrival of the RMARKER and the next edge of the external clock.
    }
    0x26, 0x00, 4, RW, GPIO_MODE(gpio_mode) { /// GPIO Mode Control Register
        msgp0,  6,  7, u8; /// Mode Selection for GPIO0/RXOKLED
        msgp1,  8,  9, u8; /// Mode Selection for GPIO1/SFDLED
        msgp2, 10, 11, u8; /// Mode Selection for GPIO2/RXLED
        msgp3, 12, 13, u8; /// Mode Selection for GPIO3/TXLED
        msgp4, 14, 15, u8; /// Mode Selection for GPIO4/EXTPA
        msgp5, 16, 17, u8; /// Mode Selection for GPIO5/EXTTXE
        msgp6, 18, 19, u8; /// Mode Selection for GPIO6/EXTRXE
        msgp7, 20, 21, u8; /// Mode Selection for SYNC/GPIO7
        msgp8, 22, 23, u8; /// Mode Selection for IRQ/GPIO8
    }
    0x26, 0x08, 4, RW, GPIO_DIR(gpio_dir) { /// GPIO Direction Control Register
        gdp0,  0,  0, u8; /// Direction Selection for GPIO0
        gdp1,  1,  1, u8; /// Direction Selection for GPIO1
        gdp2,  2,  2, u8; /// Direction Selection for GPIO2
        gdp3,  3,  3, u8; /// Direction Selection for GPIO3
        gdm0,  4,  4, u8; /// Mask for setting the direction of GPIO0
        gdm1,  5,  5, u8; /// Mask for setting the direction of GPIO1
        gdm2,  6,  6, u8; /// Mask for setting the direction of GPIO2
        gdm3,  7,  7, u8; /// Mask for setting the direction of GPIO3
        gdp4,  8,  8, u8; /// Direction Selection for GPIO4
        gdp5,  9,  9, u8; /// Direction Selection for GPIO5
        gdp6, 10, 10, u8; /// Direction Selection for GPIO6
        gdp7, 11, 11, u8; /// Direction Selection for GPIO7
        gdm4, 12, 12, u8; /// Mask for setting the direction of GPIO4
        gdm5, 13, 13, u8; /// Mask for setting the direction of GPIO5
        gdm6, 14, 14, u8; /// Mask for setting the direction of GPIO6
        gdm7, 15, 15, u8; /// Mask for setting the direction of GPIO7
        gdp8, 16, 16, u8; /// Direction Selection for GPIO8
        gdm8, 20, 20, u8; /// Mask for setting the direction of GPIO8
    }
    0x26, 0x0C, 4, RW, GPIO_DOUT(gpio_dout) { /// GPIO Data Output register
        gop0,  0,  0, u8; /// Output state setting for GPIO0
        gop1,  1,  1, u8; /// Output state setting for GPIO1
        gop2,  2,  2, u8; /// Output state setting for GPIO2
        gop3,  3,  3, u8; /// Output state setting for GPIO3
        gom0,  4,  4, u8; /// Mask for setting the output state of GPIO0
        gom1,  5,  5, u8; /// Mask for setting the output state of GPIO1
        gom2,  6,  6, u8; /// Mask for setting the output state of GPIO2
        gom3,  7,  7, u8; /// Mask for setting the output state of GPIO3
        gop4,  8,  8, u8; /// Output state setting for GPIO4
        gop5,  9,  9, u8; /// Output state setting for GPIO5
        gop6, 10, 10, u8; /// Output state setting for GPIO6
        gop7, 11, 11, u8; /// Output state setting for GPIO7
        gom4, 12, 12, u8; /// Mask for setting the output state of GPIO4
        gom5, 13, 13, u8; /// Mask for setting the output state of GPIO5
        gom6, 14, 14, u8; /// Mask for setting the output state of GPIO6
        gom7, 15, 15, u8; /// Mask for setting the output state of GPIO7
        gop8, 16, 16, u8; /// Output state setting for GPIO8
        gom8, 20, 20, u8; /// Mask for setting the output state of GPIO8
    }
    0x26, 0x10, 4, RW, GPIO_IRQE(gpio_irqe) { /// GPIO Interrupt Enable
        girqe0,  0,  0, u8; /// GPIO IRQ Enable for GPIO0 input
        girqe1,  1,  1, u8; /// GPIO IRQ Enable for GPIO1 input
        girqe2,  2,  2, u8; /// GPIO IRQ Enable for GPIO2 input
        girqe3,  3,  3, u8; /// GPIO IRQ Enable for GPIO3 input
        girqe4,  4,  4, u8; /// GPIO IRQ Enable for GPIO4 input
        girqe5,  5,  5, u8; /// GPIO IRQ Enable for GPIO5 input
        girqe6,  6,  6, u8; /// GPIO IRQ Enable for GPIO6 input
        girqe7,  7,  7, u8; /// GPIO IRQ Enable for GPIO7 input
        girqe8,  8,  8, u8; /// GPIO IRQ Enable for GPIO8 input
    }
    0x26, 0x14, 4, RW, GPIO_ISEN(gpio_isen) { /// GPIO Interrupt Sense Selection
        gisen0,  0,  0, u8; /// GPIO IRQ sense for GPIO0 input
        gisen1,  1,  1, u8; /// GPIO IRQ sense for GPIO1 input
        gisen2,  2,  2, u8; /// GPIO IRQ sense for GPIO2 input
        gisen3,  3,  3, u8; /// GPIO IRQ sense for GPIO3 input
        gisen4,  4,  4, u8; /// GPIO IRQ sense for GPIO4 input
        gisen5,  5,  5, u8; /// GPIO IRQ sense for GPIO5 input
        gisen6,  6,  6, u8; /// GPIO IRQ sense for GPIO6 input
        gisen7,  7,  7, u8; /// GPIO IRQ sense for GPIO7 input
        gisen8,  8,  8, u8; /// GPIO IRQ sense for GPIO8 input
    }
    0x26, 0x18, 4, RW, GPIO_IMODE(gpio_imode) { /// GPIO Interrupt Mode (Level / Edge)
        gimod0,  0,  0, u8; /// GPIO IRQ mode selection for GPIO0 input
        gimod1,  1,  1, u8; /// GPIO IRQ mode selection for GPIO1 input
        gimod2,  2,  2, u8; /// GPIO IRQ mode selection for GPIO2 input
        gimod3,  3,  3, u8; /// GPIO IRQ mode selection for GPIO3 input
        gimod4,  4,  4, u8; /// GPIO IRQ mode selection for GPIO4 input
        gimod5,  5,  5, u8; /// GPIO IRQ mode selection for GPIO5 input
        gimod6,  6,  6, u8; /// GPIO IRQ mode selection for GPIO6 input
        gimod7,  7,  7, u8; /// GPIO IRQ mode selection for GPIO7 input
        gimod8,  8,  8, u8; /// GPIO IRQ mode selection for GPIO8 input
    }
    0x26, 0x1C, 4, RW, GPIO_IBES(gpio_ibes) { /// GPIO Interrupt “Both Edge” Select
        gibes0,  0,  0, u8; /// GPIO IRQ "Both Edges" selection for GPIO0 input
        gibes1,  1,  1, u8; /// GPIO IRQ "Both Edges" selection for GPIO1 input
        gibes2,  2,  2, u8; /// GPIO IRQ "Both Edges" selection for GPIO2 input
        gibes3,  3,  3, u8; /// GPIO IRQ "Both Edges" selection for GPIO3 input
        gibes4,  4,  4, u8; /// GPIO IRQ "Both Edges" selection for GPIO4 input
        gibes5,  5,  5, u8; /// GPIO IRQ "Both Edges" selection for GPIO5 input
        gibes6,  6,  6, u8; /// GPIO IRQ "Both Edges" selection for GPIO6 input
        gibes7,  7,  7, u8; /// GPIO IRQ "Both Edges" selection for GPIO7 input
        gibes8,  8,  8, u8; /// GPIO IRQ "Both Edges" selection for GPIO8 input
    }
    0x26, 0x20, 4, RW, GPIO_ICLR(gpio_iclr) { /// GPIO Interrupt Latch Clear
        giclr0,  0,  0, u8; /// GPIO IRQ latch clear for GPIO0 input
        giclr1,  1,  1, u8; /// GPIO IRQ latch clear for GPIO1 input
        giclr2,  2,  2, u8; /// GPIO IRQ latch clear for GPIO2 input
        giclr3,  3,  3, u8; /// GPIO IRQ latch clear for GPIO3 input
        giclr4,  4,  4, u8; /// GPIO IRQ latch clear for GPIO4 input
        giclr5,  5,  5, u8; /// GPIO IRQ latch clear for GPIO5 input
        giclr6,  6,  6, u8; /// GPIO IRQ latch clear for GPIO6 input
        giclr7,  7,  7, u8; /// GPIO IRQ latch clear for GPIO7 input
        giclr8,  8,  8, u8; /// GPIO IRQ latch clear for GPIO8 input
    }
    0x26, 0x24, 4, RW, GPIO_IDBE(gpio_idbe) { /// GPIO Interrupt De-bounce Enable
        gidbe0,  0,  0, u8; /// GPIO IRQ de-bounce enable for GPIO0
        gidbe1,  1,  1, u8; /// GPIO IRQ de-bounce enable for GPIO1
        gidbe2,  2,  2, u8; /// GPIO IRQ de-bounce enable for GPIO2
        gidbe3,  3,  3, u8; /// GPIO IRQ de-bounce enable for GPIO3
        gidbe4,  4,  4, u8; /// GPIO IRQ de-bounce enable for GPIO4
        gidbe5,  5,  5, u8; /// GPIO IRQ de-bounce enable for GPIO5
        gidbe6,  6,  6, u8; /// GPIO IRQ de-bounce enable for GPIO6
        gidbe7,  7,  7, u8; /// GPIO IRQ de-bounce enable for GPIO7
        gidbe8,  8,  8, u8; /// GPIO IRQ de-bounce enable for GPIO8
    }
    0x26, 0x28, 4, RW, GPIO_RAW(gpio_raw) { /// GPIO raw state
        grawp0,  0,  0, u8; /// GPIO0 port raw state
        grawp1,  1,  1, u8; /// GPIO1 port raw state
        grawp2,  2,  2, u8; /// GPIO2 port raw state
        grawp3,  3,  3, u8; /// GPIO3 port raw state
        grawp4,  4,  4, u8; /// GPIO4 port raw state
        grawp5,  5,  5, u8; /// GPIO5 port raw state
        grawp6,  6,  6, u8; /// GPIO6 port raw state
        grawp7,  7,  7, u8; /// GPIO7 port raw state
        grawp8,  8,  8, u8; /// GPIO8 port raw state
    }
    0x27, 0x02, 2, RW, DRX_TUNE0B(drx_tune0b) { /// Digital Tuning Register 0b
        value, 0, 15, u16; /// DRX_TUNE0B tuning value
    }
    0x27, 0x04, 2, RW, DRX_TUNE1A(drx_tune1a) { /// Digital Tuning Register 1a
        value, 0, 15, u16; /// DRX_TUNE1A tuning value
    }
    0x27, 0x06, 2, RW, DRX_TUNE1B(drx_tune1b) { /// Digital Tuning Register 1b
        value, 0, 15, u16; /// DRX_TUNE1B tuning value
    }
    0x27, 0x08, 4, RW, DRX_TUNE2(drx_tune2) { /// Digital Tuning Register 2
        value, 0, 31, u32; /// DRX_TUNE2 tuning value
    }
    0x27, 0x20, 2, RW, DRX_SFDTOC(drx_sfdtoc) { /// SFD timeout
        count, 0, 15, u16; /// SFD detection timeout count
    }
    0x27, 0x24, 2, RW, DRX_PRETOC(drx_pretoc) { /// Preamble detection timeou
        count, 0, 15, u16; /// Preamble detection timeout count
    }
    0x27, 0x26, 2, RW, DRX_TUNE4H(drx_tune4h) { /// Digital Tuning Register 4h
        value, 0, 15, u16; /// DRX_TUNE4H tuning value
    }
    0x27, 0x28, 2, RO, DRX_CAR_INT(dxr_car_int) { /// Carrier Recovery Integrator Register
        value, 0, 15, u16; /// value
    }
    0x27, 0x2C, 2, RO, RXPACC_NOSAT(rxpacc_nosat) { /// Digital debug register. Unsaturated accumulated preamble symbols.
        value, 0, 15, u16; /// value
    }
    0x28, 0x0B, 1, RW, RF_RXCTRLH(rf_rxctrlh) { /// Analog RX Control Register
        value, 0, 7, u8; /// Analog RX Control Register
    }
    0x28, 0x0C, 3, RW, RF_TXCTRL(rf_txctrl) { /// Analog TX Control Register
        txmtune, 5,  8, u8; /// Transmit mixer tuning register
        txmq,    9, 11, u8; /// Transmit mixer Q-factor tuning register
        value, 0, 23, u32; /// The entire register
    }
    0x28, 0x2C, 4, RO, RF_STATUS(rf_status) { /// RF Status Register
        cplllock,  0, 0, u8; /// Clock PLL lock status
        cplllow,   1, 1, u8; /// Clock PLL low flag
        cpllhigh,  2, 2, u8; /// Clock PLL high flag
        rfplllock, 3, 3, u8; /// RF PLL lock status
    }
    0x28, 0x30, 5, RW, LDOTUNE(ldotune) { /// LDO voltage tuning parameter
        value, 0, 39, u64; /// Internal LDO voltage tuning parameter
    }
    0x2A, 0x0B, 1, RW, TC_PGDELAY(tc_pgdelay) { /// Pulse Generator Delay
        value, 0, 7, u8; /// Transmitter Calibration - Pulse Generator Delay
    }
    0x2B, 0x07, 4, RW, FS_PLLCFG(fs_pllcfg) { /// Frequency synth - PLL configuration
        value, 0, 31, u32; /// Frequency synth - PLL configuration
    }
    0x2B, 0x0B, 1, RW, FS_PLLTUNE(fs_plltune) { /// Frequency synth - PLL Tuning
        value, 0, 7, u8; /// Frequency synthesiser - PLL Tuning
    }
    0x2C, 0x00, 2, RW, AON_WCFG(aon_wcfg) { /// AON Wakeup Configuration Register
        onw_radc,  0,  0, u8; /// On Wake-up Run the (temperature and voltage) Analog-to-Digital Convertors.
        onw_rx,    1,  1, u8; /// On Wake-up turn on the Receiver.
        onw_leui,  3,  3, u8; /// On Wake-up load the EUI from OTP memory into Register file: 0x01 – Extended Unique Identifier.
        onw_ldc,   6,  6, u8; /// On Wake-upload configurations from the AON memory into the host interface register set.
        onw_l64p,  7,  7, u8; /// On Wake-up load the Length64 receiver operating parameter set.
        pres_sleep,8,  8, u8; /// Preserve  Sleep. This bit determines what the DW1000 does with respect to the ARXSLP and ATXSLPsleep controls in Sub-Register 0x36:04 –PMSC_CTRL1after a wake-up event.
        onw_llde, 11, 11, u8; /// On Wake-up load the LDE microcode
        onw_lldo, 12, 12, u8; /// On Wake-up load the LDOTUNE value from OTP
    }
    0x2C, 0x02, 1, RW, AON_CTRL(aon_ctrl) { /// AON Control Register
        restore,  0, 0, u8; /// When this bit is set the DW1000 will copy the user configurations from the AON memory to the host interface register set.
        save,     1, 1, u8; /// When this bit is set the DW1000 will copy the user configurations from the host interface register  set  into  the AON  memory.
        upl_cfg,  2, 2, u8; /// Upload the AON block configurations to the AON.
        dca_read, 3, 3, u8; /// Direct AON memory access read.
        dca_enab, 7, 7, u8; /// Direct AON memory access enable bit.
    }
    0x2C, 0x06, 4, RW, AON_CFG0(aon_cfg0) { /// AON Configuration Register 0
        sleep_en, 0, 0, u8; /// Sleep enable configuration bit
        wake_pin, 1, 1, u8; /// Wake using WAKEUP pin
        wake_spi, 2, 2, u8; /// Wake using SPI access
        wake_cnt, 3, 3, u8; /// Wake when sleep counter elapses
        lpdiv_en, 4, 4, u8; /// Low power divider enable configuration.
        lpclkdiva, 5, 15, u16; /// This field specifies a divider count for dividing the raw DW1000 XTAL oscillator frequency to set an LP clock frequency.
        sleep_tim, 16, 31, u16; /// Sleep time.  This field configures the sleep time count elapse value.
    }
    0x2C, 0x0A, 2, RW, AON_CFG1(aon_cfg1) { /// AON Configuration Register 1
        sleep_cen, 0, 0, u8; /// This bit enables the sleep counter.
        smxx, 1, 1, u8; /// Thisbit needs to be set to 0 for correct operation in the SLEEP state within the DW1000.
        lposc_cal, 2, 2, u8; /// This bit enables the calibration function that measures the period of the IC’s internal low powered oscillator.
    }
    0x2D, 0x04, 2, RW, OTP_ADDR(otp_addr) { /// OTP Address
        value, 0, 10, u16; /// OTP Address
    }
    0x2D, 0x06, 2, RW, OTP_CTRL(otp_ctrl) { /// OTP Control
        otprden,  0,  0, u8; /// Forces OTP into manual read mode
        otpread,  1,  1, u8; /// Commands a read operation
        otpmrwr,  3,  3, u8; /// OTP mode register write
        otpprog,  6,  6, u8; /// Write OTP_WDAT to OTP_ADDR
        otpmr,    7, 10, u8; /// OTP mode register
        ldeload, 15, 15, u8; /// Force load of LDE microcode
    }
    0x2D, 0x0A, 4, RO, OTP_RDAT(otp_rdat) { /// OTP Read Data
        value, 0, 31, u32; /// OTP Read Data
    }
    0x2E, 0x0806, 1, RW, LDE_CFG1(lde_cfg1) { /// LDE Configuration Register 1
        ntm,   0, 4, u8; /// Noise Threshold Multiplier
        pmult, 5, 7, u8; /// Peak Multiplier
    }
    0x2E, 0x1000, 2, RO, LDE_PPINDX(lde_ppindx) { /// LDE Peak Path Index
        value, 0, 15, u16; /// LDE Peak Path Index
    }
    0x2E, 0x1002, 2, RO, LDE_PPAMPL(lde_ppampl) { /// LDE Peak Path Amplitude
        value, 0, 15, u16; /// LDE Peak Path Amplitude
    }
    0x2E, 0x1804, 2, RW, LDE_RXANTD(lde_rxantd) { /// RX Antenna Delay
        value, 0, 15, u16; /// RX Antenna Delay
    }
    0x2E, 0x1806, 2, RW, LDE_CFG2(lde_cfg2) { /// LDE Configuration Register 2
        value, 0, 15, u16; /// The LDE_CFG2 configuration value
    }
    0x2E, 0x2804, 2, RW, LDE_REPC(lde_repc) { /// LDE Replica Coefficient configuration
        value, 0, 15, u16; /// The LDE_REPC configuration value
    }
    0x2F, 0x00, 4, RW, EVC_CTRL(evc_ctrl) { /// Event Counter Control
        evc_en,  0, 0, u8; /// Event Counters Enable
        evc_clr, 1, 1, u8; /// Event Counters Clear
    }
    0x2F, 0x18, 2, RO, EVC_HPW(evc_hpw) { /// Half Period Warning Counter
        value, 0, 11, u16; /// Half Period Warning Event Counter
    }
    0x2F, 0x1A, 2, RO, EVC_TPW(evc_tpw) { /// TX Power-Up Warning Counter
        value, 0, 11, u16; /// TX Power-Up Warning Event Counter
    }
    0x36, 0x00, 4, RW, PMSC_CTRL0(pmsc_ctrl0) { /// PMSC Control Register 0
        sysclks,    0,  1, u8; /// System Clock Selection
        rxclks,     2,  3, u8; /// Receiver Clock Selection
        txclks,     4,  5, u8; /// Transmitter Clock Selection
        face,       6,  6, u8; /// Force Accumulator Clock Enable
        adcce,     10, 10, u8; /// ADC Clock Enable
        amce,      15, 15, u8; /// Accumulator Memory Clock Enable
        gpce,      16, 16, u8; /// GPIO Clock Enable
        gprn,      17, 17, u8; /// GPIO Reset (Not), active low
        gpdce,     18, 18, u8; /// GPIO De-bounce Clock Enable
        gpdrn,     19, 19, u8; /// GPIO De-bounce Reset (Not), active low
        khzclken,  23, 23, u8; /// Kilohertz Clock Enable
        softreset, 28, 31, u8; /// Soft Reset
        raw_value,  0, 31,u32; /// The raw register value
    }
    0x36, 0x04, 4, RW, PMSC_CTRL1(pmsc_ctrl1) { /// PMSC Control Register 1
        arx2init,   1,  1, u8; /// Automatic transition from receive to init
        pktseq,     3, 10, u8; /// Control PMSC control of analog RF subsystem
        atxslp,    11, 11, u8; /// After TX automatically sleep
        arxslp,    12, 12, u8; /// After RX automatically sleep
        snoze,     13, 13, u8; /// Snooze Enable
        snozr,     14, 14, u8; /// Snooze Repeat
        pllsyn,    15, 15, u8; /// Enable clock used for external sync modes
        lderune,   17, 17, u8; /// LDE Run Enable
        khzclkdiv, 26, 31, u8; /// Kilohertz Clock Divisor
    }
    0x36, 0x28, 4, RW, PMSC_LEDC(pmsc_ledc) { /// PMSC LED Control Register
        blink_tim, 0, 7, u8; /// Blink time count value
        blnken, 8, 8, u8; /// Blink Enable
        blnknow, 16, 19, u8; /// Manually triggers an LED blink. There is one trigger bit per LED IO
    }
}

/// Transmit Data Buffer
///
/// Currently only the first 127 bytes of the buffer are supported, which is
/// enough to support standard Standard IEEE 802.15.4 UWB frames.
#[allow(non_camel_case_types)]
pub struct TX_BUFFER;

impl Register for TX_BUFFER {
    const ID: u8 = 0x09;
    const SUB_ID: u16 = 0x00;
    const LEN: usize = 127;
}

impl Writable for TX_BUFFER {
    type Write = tx_buffer::W;

    fn write() -> Self::Write {
        tx_buffer::W([0; 127 + 1])
    }

    fn buffer(w: &mut Self::Write) -> &mut [u8] {
        &mut w.0
    }
}

impl<SPI> DW1000<SPI> {
    /// Transmit Data Buffer
    pub fn tx_buffer(&mut self) -> RegAccessor<TX_BUFFER, SPI> {
        RegAccessor(self, PhantomData)
    }
}

/// Transmit Data Buffer
pub mod tx_buffer {
    /// Used to write to the register
    pub struct W(pub(crate) [u8; 127 + 1]);

    impl W {
        /// Provides write access to the buffer contents
        pub fn data(&mut self) -> &mut [u8] {
            &mut self.0[1..]
        }
    }
}

/// Receive Data Buffer
///
/// Currently only the first 127 bytes of the buffer are supported, which is
/// enough to support standard Standard IEEE 802.15.4 UWB frames.
#[allow(non_camel_case_types)]
pub struct RX_BUFFER;

impl Register for RX_BUFFER {
    const ID: u8 = 0x11;
    const SUB_ID: u16 = 0x00;
    const LEN: usize = 127;
}

impl Readable for RX_BUFFER {
    type Read = rx_buffer::R;

    fn read() -> Self::Read {
        rx_buffer::R([0; 127 + 1])
    }

    fn buffer(w: &mut Self::Read) -> &mut [u8] {
        &mut w.0
    }
}

impl<SPI> DW1000<SPI> {
    /// Receive Data Buffer
    pub fn rx_buffer(&mut self) -> RegAccessor<RX_BUFFER, SPI> {
        RegAccessor(self, PhantomData)
    }
}

/// Receive Data Buffer
pub mod rx_buffer {
    use core::fmt;

    const HEADER_LEN: usize = 1;
    const LEN: usize = 127;

    /// Used to read from the register
    pub struct R(pub(crate) [u8; HEADER_LEN + LEN]);

    impl R {
        /// Provides read access to the buffer contents
        pub fn data(&self) -> &[u8] {
            &self.0[HEADER_LEN..HEADER_LEN + LEN]
        }
    }

    impl fmt::Debug for R {
        fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
            write!(f, "0x")?;
            for i in (0..LEN).rev() {
                write!(f, "{:02x}", self.0[HEADER_LEN + i])?;
            }

            Ok(())
        }
    }
}

/// Internal trait used by `impl_registers!`
trait FromBytes {
    fn from_bytes(bytes: &[u8]) -> Self;
}

/// Internal trait used by `impl_registers!`
trait ToBytes {
    type Bytes;

    fn to_bytes(self) -> Self::Bytes;
}

/// Internal macro used to implement `FromBytes`/`ToBytes`
macro_rules! impl_bytes {
    ($($ty:ty,)*) => {
        $(
            impl FromBytes for $ty {
                fn from_bytes(bytes: &[u8]) -> Self {
                    let mut val = 0;

                    for (i, &b) in bytes.iter().enumerate() {
                        val |= (b as $ty) << (i * 8);
                    }

                    val
                }
            }

            impl ToBytes for $ty {
                type Bytes = [u8; ::core::mem::size_of::<$ty>()];

                fn to_bytes(self) -> Self::Bytes {
                    let mut bytes = [0; ::core::mem::size_of::<$ty>()];

                    for (i, b) in bytes.iter_mut().enumerate() {
                        let shift = 8 * i;
                        let mask  = 0xff << shift;

                        *b = ((self & mask) >> shift) as u8;
                    }

                    bytes
                }
            }
        )*
    }
}

impl_bytes! {
    u8,
    u16,
    u32,
    u64,
}
