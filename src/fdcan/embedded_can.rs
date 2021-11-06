//! `embedded_can` trait impls.

use crate::{Can, Data, ExtendedId, Frame, Id, Instance, StandardId};
use embedded_can_03 as embedded_can;

impl<I> embedded_can::Can for Can<I>
where
    I: Instance,
{
    type Frame = Frame;

    type Error = ();

    fn try_transmit(
        &mut self,
        frame: &Self::Frame,
    ) -> nb::Result<Option<Self::Frame>, Self::Error> {
        match self.transmit(frame) {
            Ok(status) => Ok(status.dequeued_frame().cloned()),
            Err(nb::Error::WouldBlock) => Err(nb::Error::WouldBlock),
            Err(nb::Error::Other(e)) => match e {},
        }
    }

    fn try_receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
        self.receive()
    }
}

impl embedded_can::Frame for Frame {
    fn new(id: impl Into<embedded_can::Id>, data: &[u8]) -> Result<Self, ()> {
        let id = match id.into() {
            embedded_can::Id::Standard(id) => unsafe {
                Id::Standard(StandardId::new_unchecked(id.as_raw()))
            },
            embedded_can::Id::Extended(id) => unsafe {
                Id::Extended(ExtendedId::new_unchecked(id.as_raw()))
            },
        };

        let data = Data::new(data).ok_or(())?;
        Ok(Frame::new_data(id, data))
    }

    fn new_remote(id: impl Into<embedded_can::Id>, dlc: usize) -> Result<Self, ()> {
        let id = match id.into() {
            embedded_can::Id::Standard(id) => unsafe {
                Id::Standard(StandardId::new_unchecked(id.as_raw()))
            },
            embedded_can::Id::Extended(id) => unsafe {
                Id::Extended(ExtendedId::new_unchecked(id.as_raw()))
            },
        };

        if dlc <= 8 {
            Ok(Frame::new_remote(id, dlc as u8))
        } else {
            Err(())
        }
    }

    #[inline]
    fn is_extended(&self) -> bool {
        self.is_extended()
    }

    #[inline]
    fn is_standard(&self) -> bool {
        self.is_standard()
    }

    #[inline]
    fn is_remote_frame(&self) -> bool {
        self.is_remote_frame()
    }

    #[inline]
    fn is_data_frame(&self) -> bool {
        self.is_data_frame()
    }

    #[inline]
    fn id(&self) -> embedded_can::Id {
        match self.id() {
            Id::Standard(id) => unsafe {
                embedded_can::Id::Standard(embedded_can::StandardId::new_unchecked(id.as_raw()))
            },
            Id::Extended(id) => unsafe {
                embedded_can::Id::Extended(embedded_can::ExtendedId::new_unchecked(id.as_raw()))
            },
        }
    }

    #[inline]
    fn dlc(&self) -> usize {
        self.dlc() as usize
    }

    fn data(&self) -> &[u8] {
        if let Some(data) = self.data() {
            data
        } else {
            &[]
        }
    }
}
