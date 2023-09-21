//! This implementation is derived from 0BSD-relicensed work done by 
//! Johannes Draaijer <jcdra1@gmail.com> for the 
//! [`stm32-eth`](https://github.com/stm32-rs/stm32-eth) project


//TODO remove
// this is raw_descriptor
use volatile_register::{RO, RW};

use crate::ethernet::MTU;

pub(crate) const DESC_SIZE: usize = 4;

#[repr(C)]
#[repr(align(4))]
#[derive(Clone, Copy)]
pub struct RawDescriptor {
    pub(crate) desc: [u32; DESC_SIZE],
}

impl Default for RawDescriptor {
    fn default() -> Self {
        Self::new()
    }
}

impl RawDescriptor {
    pub const fn new() -> Self {
        Self {
            desc: [0; DESC_SIZE],
        }
    }

    fn r(&self, n: usize) -> &RO<u32> {
        let ro = &self.desc[n] as *const _ as *const RO<u32>;
        unsafe { &*ro }
    }

    unsafe fn rw(&mut self, n: usize) -> &mut RW<u32> {
        let rw = &mut self.desc[n] as *mut _ as *mut RW<u32>;
        &mut *rw
    }

    pub fn read(&self, n: usize) -> u32 {
        self.r(n).read()
    }

    pub unsafe fn write(&mut self, n: usize, value: u32) {
        self.rw(n).write(value)
    }

    pub unsafe fn modify<F>(&mut self, n: usize, f: F)
    where
        F: FnOnce(u32) -> u32,
    {
        self.rw(n).modify(f)
    }
}

pub struct DescriptorRing<'data, T> {
    descriptors: &'data mut [T],
    buffers: &'data mut [[u8; MTU + 2]],
}

impl<'data, T> DescriptorRing<'data, T> {
    pub fn new(
        descriptors: &'data mut [T],
        buffers: &'data mut [[u8; MTU + 2]],
    ) -> Self {
        assert!(descriptors.len() == buffers.len());

        Self {
            descriptors,
            buffers,
        }
    }

    pub fn len(&self) -> usize {
        self.descriptors.len()
    }

    pub fn descriptor(&self, index: usize) -> &T {
        &self.descriptors[index]
    }

    pub fn get(&self, index: usize) -> (&T, &[u8]) {
        (&self.descriptors[index], &self.buffers[index])
    }

    pub fn get_mut(&mut self, index: usize) -> (&mut T, &mut [u8]) {
        (&mut self.descriptors[index], &mut self.buffers[index])
    }

    pub fn get_mut_and_next(
        &mut self,
        index: usize,
    ) -> (&mut T, &mut [u8], &mut T, &mut [u8]) {
        let next = (index + 1) % self.len();

        macro_rules! mut_and_next {
            ($array:expr) => {{
                let (index_slice, next_slice) = if next == 0 {
                    let (next, index) = $array.split_at_mut(1);
                    (index, next)
                } else {
                    $array.split_at_mut(next)
                };

                (&mut index_slice[index_slice.len() - 1], &mut next_slice[0])
            }};
        }

        let (desc_index, desc_next) = mut_and_next!(self.descriptors);
        let (buf_index, buf_next) = mut_and_next!(self.buffers);

        (desc_index, buf_index, desc_next, buf_next)
    }

    pub fn descriptors_mut(&mut self) -> impl Iterator<Item = &mut T> {
        self.descriptors.iter_mut()
    }

    pub fn descriptors(&self) -> impl Iterator<Item = &T> {
        self.descriptors.iter()
    }

    pub fn last_descriptor_mut(&mut self) -> &mut T {
        &mut self.descriptors[self.descriptors.len() - 1]
    }

    pub fn last_descriptor(&self) -> &T {
        &self.descriptors[self.descriptors.len() - 1]
    }

    pub fn first_buffer(&self) -> &[u8] {
        &self.buffers[0]
    }

    pub fn last_buffer(&self) -> &[u8] {
        &self.buffers[self.buffers.len() - 1]
    }

    pub fn descriptors_and_buffers(
        &mut self,
    ) -> impl Iterator<Item = (&mut T, &mut [u8; MTU + 2])> {
        self.descriptors.iter_mut().zip(self.buffers.iter_mut())
    }

    pub fn descriptors_start_address(&self) -> *const T {
        self.descriptors.as_ptr()
    }
}
