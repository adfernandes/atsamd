//! Define a container for a set of SERCOM pads
//!
//! See the [i2c module](super) documentation for more details on declaring and
//! instantiating a [`Pads`] type.

use crate::{gpio::AnyPin, sercom::*, typelevel::Sealed};
use core::marker::PhantomData;

/// Container for a set of SERCOM [`Pad`]s
///
/// See the [module-level](crate::sercom::i2c) documentation for more
/// details on specifying a `Pads` type and creating instances.
pub struct Pads<S, SDA, SCL>
where
    S: Sercom,
    SDA: IsI2cPad<PadNum = Pad0, Sercom = S>,
    SCL: IsI2cPad<PadNum = Pad1, Sercom = S>,
    (SDA, SCL): ShareIoSet,
{
    sercom: PhantomData<S>,
    sda: SDA,
    scl: SCL,
}

impl<S, DI, CI> PadsFromIds<S, DI, CI>
where
    S: Sercom,
    DI: GetPad<S>,
    CI: GetPad<S>,
    Pad<S, DI>: IsI2cPad<Sercom = S, PadNum = Pad0>,
    Pad<S, CI>: IsI2cPad<Sercom = S, PadNum = Pad1>,
    (Pad<S, DI>, Pad<S, CI>): ShareIoSet,
{
    pub fn new(sda: impl AnyPin<Id = DI>, scl: impl AnyPin<Id = CI>) -> Self {
        Self {
            sercom: PhantomData,
            sda: sda.into().into_mode(),
            scl: scl.into().into_mode(),
        }
    }
}

impl<S, SDA, SCL> Pads<S, SDA, SCL>
where
    S: Sercom,
    SDA: IsI2cPad<PadNum = Pad0, Sercom = S>,
    SCL: IsI2cPad<PadNum = Pad1, Sercom = S>,
    (SDA, SCL): ShareIoSet,
{
    /// Consume the [`Pads`] and return each individual
    /// [`Pin`](crate::gpio::Pin)
    #[inline]
    pub fn free(self) -> (SDA, SCL) {
        (self.sda, self.scl)
    }
}

//=============================================================================
// PadsFromIds
//=============================================================================

/// Define a set of [`Pads`] using [`PinId`]s instead of [`Pin`]s
///
/// In some cases, it is more convenient to specify a set of `Pads` using
/// `PinId`s rather than `Pin`s. This alias makes it easier to do so.
///
/// The first parameter is the [`Sercom`], while the
/// remaining two are [`PinId`]s representing the corresponding type
/// parameters of [`Pads`], i.e. `SDA` & `SCL`.
///
/// ```
/// use atsamd_hal::pac::Peripherals;
/// use atsamd_hal::gpio::{PA08, PA09, Pins};
/// use atsamd_hal::sercom::{Sercom0, i2c};
/// use atsamd_hal::typelevel::NoneT;
///
/// pub type Pads = i2c::PadsFromIds<Sercom0, PA08, PA09>;
///
/// pub fn create_pads() -> Pads {
///     let peripherals = Peripherals::take().unwrap();
///     let pins = Pins::new(peripherals.PORT);
///     i2c::Pads::default().sda(pins.pa08).scl(pins.pa09)
/// }
/// ```
///
/// [`Pin`]: crate::gpio::Pin
/// [`PinId`]: crate::gpio::PinId
pub type PadsFromIds<S, SDA, SCL> = Pads<S, Pad<S, SDA>, Pad<S, SCL>>;

//=============================================================================
// PadSet
//=============================================================================

/// Type-level function to recover the [`Pad`] types from a generic set
/// of [`Pads`]
///
/// This trait is used as an interface between the [`Pads`] type and other
/// types in this module. It acts as a [type-level function], returning the
/// corresponding [`Sercom`] and [`OptionalPad`] types. It serves to cut down on
/// the total number of type parameters needed in the [`Config`] struct. The
/// `Config` struct doesn't need access to the [`Pin`]s directly.  Rather, it
/// only needs to apply the [`SomePad`] trait bound when a `Pin` is required.
/// The `PadSet` trait allows each `Config` struct to store an instance of
/// `Pads` without itself being generic over all six type parameters of the
/// `Pads` type.
///
/// This trait is a simplified version of the [`AnyKind`] trait pattern.
///
/// [`Pin`]: crate::gpio::Pin
/// [`Config`]: super::Config
/// [type-level function]: crate::typelevel#type-level-functions
/// [`AnyKind`]: crate::typelevel#anykind-trait-pattern
pub trait PadSet: Sealed {
    type Sercom: Sercom;
    type Sda: IsI2cPad<PadNum = Pad0, Sercom = Self::Sercom>;
    type Scl: IsI2cPad<PadNum = Pad1, Sercom = Self::Sercom>;
}

impl<S, SDA, SCL> Sealed for Pads<S, SDA, SCL>
where
    S: Sercom,
    SDA: IsI2cPad<PadNum = Pad0, Sercom = S>,
    SCL: IsI2cPad<PadNum = Pad1, Sercom = S>,
    (SDA, SCL): ShareIoSet,
{
}

impl<S, SDA, SCL> PadSet for Pads<S, SDA, SCL>
where
    S: Sercom,
    SDA: IsI2cPad<PadNum = Pad0, Sercom = S>,
    SCL: IsI2cPad<PadNum = Pad1, Sercom = S>,
    (SDA, SCL): ShareIoSet,
{
    type Sercom = S;
    type Sda = SDA;
    type Scl = SCL;
}
