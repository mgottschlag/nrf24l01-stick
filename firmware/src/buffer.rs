pub struct RingBuffer<Buffer: AsMut<[u8]> + AsRef<[u8]>> {
    buffer: Buffer,
    read: usize,
    length: usize,
}

impl<Buffer: AsMut<[u8]> + AsRef<[u8]>> RingBuffer<Buffer> {
    pub fn new(buffer: Buffer) -> Self {
        Self {
            buffer,
            read: 0,
            length: 0,
        }
    }

    pub fn len(&self) -> usize {
        self.length
    }

    pub fn capacity(&self) -> usize {
        self.buffer.as_ref().len() - self.length
    }

    pub fn append(&mut self, data: &[u8]) -> Result<(), InsufficientSpace> {
        if data.len() > self.capacity() {
            return Err(InsufficientSpace);
        }
        let buffer = self.buffer.as_mut();
        let write = (self.read + self.length) % buffer.len();
        let remaining = buffer.len() - write;
        if data.len() > remaining {
            buffer[write..].copy_from_slice(&data[0..remaining]);
            buffer[0..data.len() - remaining].copy_from_slice(&data[remaining..]);
        } else {
            buffer[write..write + data.len()].copy_from_slice(data);
        }
        self.length += data.len();
        Ok(())
    }

    pub fn next_slice<'a>(&'a self) -> &'a [u8] {
        let buffer = self.buffer.as_ref();
        if self.read + self.length > buffer.len() {
            &buffer[self.read..]
        } else {
            &buffer[self.read..self.read + self.length]
        }
    }

    pub fn consume(&mut self, n: usize) {
        if n >= self.length {
            self.read = 0;
            self.length = 0;
        } else {
            self.length -= n;
            self.read = (self.read + n) % self.buffer.as_mut().len();
        }
    }
}

impl<Buffer: AsMut<[u8]> + AsRef<[u8]>> Default for RingBuffer<Buffer>
where
    Buffer: Default,
{
    fn default() -> Self {
        Self {
            buffer: Buffer::default(),
            read: 0,
            length: 0,
        }
    }
}

pub struct InsufficientSpace;
