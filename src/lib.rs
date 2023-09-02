use std::hash::Hasher;

pub trait ZeroOneUniformFloatFromBits<O> {
    fn lsb_to_01(self) -> O;
    fn lsb_to_12(self) -> O;
    fn lsb_to_01_signed(self) -> O;
}

pub trait FloatHasher<O> {
    fn bytes_to_int(&mut self, input: &[u8]) -> O;
}

pub struct XXHasher;

impl FloatHasher<u32> for XXHasher {
    fn bytes_to_int(&mut self, input: &[u8]) -> u32 {
        xxhash_rust::xxh32::xxh32(input, 0)
    }
}

impl FloatHasher<u64> for XXHasher {
    fn bytes_to_int(&mut self, input: &[u8]) -> u64 {
        xxhash_rust::xxh3::xxh3_64(input)
    }
}

#[derive(Default)]
pub struct StreamingXXHasher {
    state: xxhash_rust::xxh3::Xxh3,
}

impl<const N: usize> FloatHasher<[u64; N]> for StreamingXXHasher {
    fn bytes_to_int(&mut self, input: &[u8]) -> [u64; N] {
        self.state.reset();
        self.state.update(input);
        let mut input_iter = input.into_iter().cycle().copied();
        let mut uvals = [0; N];
        uvals[0] = self.state.finish();

        for u in uvals[1..].iter_mut() {
            for _ in 0..8 {
                self.state.write_u8(input_iter.next().unwrap());
            }
            *u = self.state.finish()
        }
        uvals
    }
}

pub trait FloatHashInput<I> {}
impl FloatHashInput<u32> for f32 {}
impl FloatHashInput<u64> for f64 {}
impl FloatHashInput<u64> for [f32; 2] {}
impl FloatHashInput<[u64; 2]> for [f32; 3] {}
impl FloatHashInput<[u64; 2]> for [f32; 4] {}
impl FloatHashInput<[u64; 2]> for [f64; 2] {}
impl FloatHashInput<[u64; 3]> for [f64; 3] {}
impl FloatHashInput<[u64; 4]> for [f64; 4] {}

pub trait AsBytes {
    fn as_bytes(&self) -> &[u8];
}

pub trait HashToUniformFloat<I, O, H> {
    fn hash_to_float_01(self, hasher: &mut H) -> O;
    fn hash_to_float_12(self, hasher: &mut H) -> O;
    fn hash_to_float_01_signed(self, hasher: &mut H) -> O;
}

impl<I, O, T, H> HashToUniformFloat<I, O, H> for T
where
    I: ZeroOneUniformFloatFromBits<O>,
    O: FloatHashInput<I>,
    H: FloatHasher<I>,
    Self: Sized + AsBytes,
{
    fn hash_to_float_01(self, hasher: &mut H) -> O {
        hasher.bytes_to_int(self.as_bytes()).lsb_to_01()
    }

    fn hash_to_float_12(self, hasher: &mut H) -> O {
        hasher.bytes_to_int(self.as_bytes()).lsb_to_12()
    }

    fn hash_to_float_01_signed(self, hasher: &mut H) -> O {
        hasher.bytes_to_int(self.as_bytes()).lsb_to_01_signed()
    }
}

impl ZeroOneUniformFloatFromBits<f32> for u32 {
    fn lsb_to_01(self) -> f32 {
        self.lsb_to_12() - 1.0
    }

    fn lsb_to_12(mut self) -> f32 {
        // clear the first two bits
        self = self & 0b0011_1111_1111_1111_1111_1111_1111_1111;
        // set the exponent to 127
        self = self | 0b0011_1111_1000_0000_0000_0000_0000_0000;
        f32::from_bits(self)
    }

    fn lsb_to_01_signed(self) -> f32 {
        let msb = self & 0b1000_0000_0000_0000_0000_0000_0000_0000;
        f32::from_bits(self.lsb_to_01().to_bits() | msb)
    }
}

impl ZeroOneUniformFloatFromBits<f64> for u64 {
    fn lsb_to_01(self) -> f64 {
        let out: f64 = self.lsb_to_12();
        out - 1.0
    }

    fn lsb_to_12(mut self) -> f64 {
        // clear the first two bits of 64 unsigned int
        self = self
            & 0b0011_1111_1111_1111_1111_1111_1111_1111_1111_1111_1111_1111_1111_1111_1111_1111;
        // set the exponent to 1023
        self = self
            | 0b0011_1111_1111_0000_0000_0000_0000_0000_0000_0000_0000_0000_0000_0000_0000_0000;
        f64::from_bits(self)
    }

    fn lsb_to_01_signed(self) -> f64 {
        let msb = self
            & 0b1000_0000_0000_0000_0000_0000_0000_0000_0000_0000_0000_0000_0000_0000_0000_0000;
        let temp_float: f64 = self.lsb_to_01();
        f64::from_bits(temp_float.to_bits() | msb)
    }
}

impl<const N: usize> ZeroOneUniformFloatFromBits<[f64; N]> for [u64; N] {
    fn lsb_to_01(self) -> [f64; N] {
        let mut temp: [f64; N] = [0.0; N];
        for (f, u) in temp.iter_mut().zip(self.iter()) {
            *f = u.lsb_to_01();
        }
        temp
    }

    fn lsb_to_12(self) -> [f64; N] {
        let mut temp: [f64; N] = [0.0; N];
        for (f, u) in temp.iter_mut().zip(self.iter()) {
            *f = u.lsb_to_12();
        }
        temp
    }

    fn lsb_to_01_signed(self) -> [f64; N] {
        let mut temp: [f64; N] = [0.0; N];
        for (f, u) in temp.iter_mut().zip(self.iter()) {
            *f = u.lsb_to_01_signed();
        }
        temp
    }
}

impl<const N: usize, const M: usize> ZeroOneUniformFloatFromBits<[f32; N]> for [u64; M] {
    fn lsb_to_01(self) -> [f32; N] {
        let mut temp: [f32; N] = [0.0; N];
        let chunks_mut = temp.chunks_exact_mut(2);
        for (f, u) in chunks_mut.zip(self.iter()) {
            // split u into two u32s
            let u_bytes = u.to_le_bytes();
            let u_low: u32 = u32::from_le_bytes(u_bytes[0..4].try_into().unwrap());
            let u_high: u32 = u32::from_le_bytes(u_bytes[4..8].try_into().unwrap());
            f[0] = u_low.lsb_to_01();
            f[1] = u_high.lsb_to_01();
        }

        // since N is constant, we can check if it is odd at compile time
        if N % 2 == 1 {
            temp[N - 1] =
                u32::from_le_bytes(self[M - 1].to_le_bytes()[0..4].try_into().unwrap()).lsb_to_01();
        }
        temp
    }

    fn lsb_to_12(self) -> [f32; N] {
        let mut temp: [f32; N] = [0.0; N];
        let chunks_mut = temp.chunks_exact_mut(2);
        for (f, u) in chunks_mut.zip(self.iter()) {
            // split u into two u32s
            let u_bytes = u.to_le_bytes();
            let u_low: u32 = u32::from_le_bytes(u_bytes[0..4].try_into().unwrap());
            let u_high: u32 = u32::from_le_bytes(u_bytes[4..8].try_into().unwrap());
            f[0] = u_low.lsb_to_12();
            f[1] = u_high.lsb_to_12();
        }

        // since N is constant, we can check if it is odd at compile time
        if N % 2 == 1 {
            temp[N - 1] =
                u32::from_le_bytes(self[M - 1].to_le_bytes()[0..4].try_into().unwrap()).lsb_to_12();
        }
        temp
    }

    fn lsb_to_01_signed(self) -> [f32; N] {
        let mut temp: [f32; N] = [0.0; N];
        let chunks_mut = temp.chunks_exact_mut(2);
        for (f, u) in chunks_mut.zip(self.iter()) {
            // split u into two u32s
            let u_bytes = u.to_le_bytes();
            let u_low: u32 = u32::from_le_bytes(u_bytes[0..4].try_into().unwrap());
            let u_high: u32 = u32::from_le_bytes(u_bytes[4..8].try_into().unwrap());
            f[0] = u_low.lsb_to_12();
            f[1] = u_high.lsb_to_12();
        }

        // since N is constant, we can check if it is odd at compile time
        if N % 2 == 1 {
            temp[N - 1] =
                u32::from_le_bytes(self[M - 1].to_le_bytes()[0..4].try_into().unwrap()).lsb_to_12();
        }
        temp
    }
}

impl<T> AsBytes for T
where
    T: bytemuck::Pod,
{
    fn as_bytes(&self) -> &[u8] {
        bytemuck::bytes_of(self)
    }
}
