#![macro_use]

use core::cell::Cell;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use crate::interrupt::typelevel::Interrupt;
use crate::{chip::interrupt, driverlib};
use embassy_hal_internal::drop::OnDrop;
use embassy_hal_internal::{Peri, PeripheralType};
use embassy_sync::waitqueue::AtomicWaker;
use ti_cc2650_pac::I2C0::vals;

#[derive(Clone)]
#[non_exhaustive]
pub struct MasterConfig {
    pub sda_pin: u32,
    pub scl_pin: u32,
    pub fast_mode: bool,
    // For connecting cc2650 master module with slave module.
    pub loopback: bool,
}

impl Default for MasterConfig {
    fn default() -> Self {
        Self {
            sda_pin: driverlib::IOID_18,
            scl_pin: driverlib::IOID_19,
            fast_mode: false,
            loopback: false,
        }
    }
}

#[derive(Clone)]
#[non_exhaustive]
pub struct SlaveConfig {
    pub sda_pin: u32,
    pub scl_pin: u32,
    pub addr: u8,
}

impl SlaveConfig {
    pub fn new(addr: u8) -> Self {
        Self {
            sda_pin: driverlib::IOID_18,
            scl_pin: driverlib::IOID_19,
            addr,
        }
    }
}

/// Interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        unsafe {
            let master_irq_status = driverlib::I2CMasterIntStatus(driverlib::I2C0_BASE, true);
            driverlib::I2CMasterIntClear(driverlib::I2C0_BASE);
            let slave_irq_status = driverlib::I2CSlaveIntStatus(driverlib::I2C0_BASE, true);
            driverlib::I2CSlaveIntClear(driverlib::I2C0_BASE, slave_irq_status);
            let s = T::state();
            if master_irq_status {
                s.master_waker.wake();
            }
            if slave_irq_status != 0 {
                s.slave_waker.wake();
            }
        }
    }
}

/// I2C peripheral instance
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static + Send {
    /// Interrupt for this peripheral.
    type Interrupt: interrupt::typelevel::Interrupt;
}

pub(crate) struct State {
    pub(crate) master_waker: AtomicWaker,
    pub(crate) slave_waker: AtomicWaker,
}

impl State {
    pub(crate) const fn new() -> Self {
        Self {
            master_waker: AtomicWaker::new(),
            slave_waker: AtomicWaker::new(),
        }
    }
}

pub(crate) trait SealedInstance {
    fn state() -> &'static State;
}

macro_rules! impl_i2c {
    ($type:ident, $pac_type:ident, $irq:ident) => {
        impl crate::i2c::SealedInstance for peripherals::$type {
            fn state() -> &'static crate::i2c::State {
                static STATE: crate::i2c::State = crate::i2c::State::new();
                &STATE
            }
        }
        impl crate::i2c::Instance for peripherals::$type {
            type Interrupt = crate::interrupt::typelevel::$irq;
        }
    };
}

#[derive(PartialEq)]
pub enum MasterResult {
    /// Transaction succeeded
    Ok,
    /// Address was not acknowledged by the receiver.
    AddrAck,
    /// Data was not acknowledged by the receiver.
    DataAck,
    /// Master lost arbitration.
    ArLost,
    InvalidAddr,
}

pub struct I2CMaster<'a, T: Instance> {
    state: &'static State,
    /// STOP owed to the bus by a cancelled transfer, sent once BUSY clears.
    pending_stop: Cell<Option<u32>>,
    _p: PhantomData<(&'a (), T)>,
}

impl<'a, T: Instance> I2CMaster<'a, T> {
    pub(crate) fn new(
        config: MasterConfig,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'a,
    ) -> Self {
        unsafe {
            // Setup pins.
            driverlib::IOCPinTypeI2c(driverlib::IOC_BASE, config.sda_pin, config.scl_pin);

            // Initialize I2C master.
            let freq = driverlib::SysCtrlClockGet();
            driverlib::I2CMasterInitExpClk(driverlib::I2C0_BASE, freq, config.fast_mode);
            driverlib::I2CMasterIntEnable(driverlib::I2C0_BASE);
        };

        if config.loopback {
            // Loopback to connect to the slave.
            ti_cc2650_pac::I2C0.MCR().modify(|w| w.set_LPBK(vals::LPBK::EN));
        }

        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        Self {
            state: T::state(),
            pending_stop: Cell::new(None),
            _p: PhantomData,
        }
    }

    async fn await_master_completion(&self) {
        let _ = poll_fn(|cx| {
            self.state.master_waker.register(cx.waker());
            unsafe {
                if driverlib::I2CMasterBusy(driverlib::I2C0_BASE) {
                    return Poll::Pending;
                }
            }
            Poll::Ready(())
        })
        .await;
    }

    fn handle_transaction_result(&self) -> MasterResult {
        let error_flags = unsafe { driverlib::I2CMasterErr(driverlib::I2C0_BASE) };
        if error_flags == driverlib::I2C_MASTER_ERR_NONE {
            MasterResult::Ok
        } else if error_flags & driverlib::I2C_MASTER_ERR_ARB_LOST != 0 {
            MasterResult::ArLost
        } else if error_flags & driverlib::I2C_MASTER_ERR_ADDR_ACK != 0 {
            MasterResult::AddrAck
        } else {
            MasterResult::DataAck
        }
    }

    fn get_drop_guard(&self, flag: u32) -> OnDrop<impl FnOnce() + '_> {
        OnDrop::new(move || {
            // Can't wait for BUSY in Drop, so defer the STOP to the next operation.
            if unsafe { driverlib::I2CMasterBusy(driverlib::I2C0_BASE) } {
                self.pending_stop.set(Some(flag));
            } else {
                Self::stop_unless_arb_lost(flag);
            }
        })
    }

    /// Must only be called when the controller isn't BUSY.
    fn stop_unless_arb_lost(flag: u32) {
        unsafe {
            // After lost arbitration the bus isn't ours, so no STOP.
            if driverlib::I2CMasterErr(driverlib::I2C0_BASE) & driverlib::I2C_MASTER_ERR_ARB_LOST == 0 {
                driverlib::I2CMasterControl(driverlib::I2C0_BASE, flag);
            }
        }
    }

    fn first_send_cmd(len: usize) -> u32 {
        if len == 1 {
            driverlib::I2C_MASTER_CMD_SINGLE_SEND
        } else {
            driverlib::I2C_MASTER_CMD_BURST_SEND_START
        }
    }

    fn next_send_cmd(i: usize, len: usize) -> u32 {
        if i == len - 1 {
            driverlib::I2C_MASTER_CMD_BURST_SEND_FINISH
        } else {
            driverlib::I2C_MASTER_CMD_BURST_SEND_CONT
        }
    }

    fn first_recv_cmd(len: usize) -> u32 {
        if len == 1 {
            driverlib::I2C_MASTER_CMD_SINGLE_RECEIVE
        } else {
            driverlib::I2C_MASTER_CMD_BURST_RECEIVE_START
        }
    }

    fn next_recv_cmd(i: usize, len: usize) -> u32 {
        if i == len - 1 {
            driverlib::I2C_MASTER_CMD_BURST_RECEIVE_FINISH
        } else {
            driverlib::I2C_MASTER_CMD_BURST_RECEIVE_CONT
        }
    }

    fn send_stale_stop_blocking(&self) {
        if let Some(flag) = self.pending_stop.get() {
            while unsafe { driverlib::I2CMasterBusy(driverlib::I2C0_BASE) } {}
            self.pending_stop.set(None);
            Self::stop_unless_arb_lost(flag);
        }
    }

    async fn send_stale_stop(&self) {
        if let Some(flag) = self.pending_stop.get() {
            self.await_master_completion().await;
            // Cleared only after the await, so cancelling here keeps the STOP pending.
            self.pending_stop.set(None);
            Self::stop_unless_arb_lost(flag);
        }
    }

    pub fn read_byte_blocking(&mut self, slave_addr: u8, buf: &mut u8) -> MasterResult {
        if slave_addr > 0x7F {
            return MasterResult::InvalidAddr;
        }
        self.send_stale_stop_blocking();
        unsafe {
            // Set the receiving slave address.
            driverlib::I2CMasterSlaveAddrSet(driverlib::I2C0_BASE, slave_addr, true);
            // Wait until master controller is not busy.
            while driverlib::I2CMasterBusy(driverlib::I2C0_BASE) {}
            // Write 0-111 to MCTRL.
            driverlib::I2CMasterControl(driverlib::I2C0_BASE, driverlib::I2C_MASTER_CMD_SINGLE_RECEIVE);
            // Wait until master controller is not busy.
            while driverlib::I2CMasterBusy(driverlib::I2C0_BASE) {}

            match self.handle_transaction_result() {
                MasterResult::Ok => *buf = driverlib::I2CMasterDataGet(driverlib::I2C0_BASE) as u8,
                err => return err,
            }
        }
        MasterResult::Ok
    }

    pub fn write_byte_blocking(&mut self, slave_addr: u8, buf: u8) -> MasterResult {
        if slave_addr > 0x7F {
            return MasterResult::InvalidAddr;
        }
        self.send_stale_stop_blocking();
        unsafe {
            // Set the receiving slave address.
            driverlib::I2CMasterSlaveAddrSet(driverlib::I2C0_BASE, slave_addr, false);
            // Transmit one byte.
            driverlib::I2CMasterDataPut(driverlib::I2C0_BASE, buf);
            // Wait until master controller is not busy.
            while driverlib::I2CMasterBusy(driverlib::I2C0_BASE) {}
            // Write 0-111 to MCTRL.
            driverlib::I2CMasterControl(driverlib::I2C0_BASE, driverlib::I2C_MASTER_CMD_SINGLE_SEND);
            // Wait until master controller is not busy.
            while driverlib::I2CMasterBusy(driverlib::I2C0_BASE) {}
            self.handle_transaction_result()
        }
    }

    pub async fn read_byte(&mut self, slave_addr: u8, buf: &mut u8) -> MasterResult {
        if slave_addr > 0x7F {
            return MasterResult::InvalidAddr;
        }
        self.send_stale_stop().await;
        unsafe {
            // Set the receiving slave address.
            driverlib::I2CMasterSlaveAddrSet(driverlib::I2C0_BASE, slave_addr, true);
            // Wait until master controller is not busy.
            self.await_master_completion().await;
            // Write 0-111 to MCTRL.
            driverlib::I2CMasterControl(driverlib::I2C0_BASE, driverlib::I2C_MASTER_CMD_SINGLE_RECEIVE);
            // Wait until master controller is not busy.
            self.await_master_completion().await;

            match self.handle_transaction_result() {
                MasterResult::Ok => *buf = driverlib::I2CMasterDataGet(driverlib::I2C0_BASE) as u8,
                err => return err,
            }
        }
        MasterResult::Ok
    }

    pub async fn write_byte(&mut self, slave_addr: u8, buf: u8) -> MasterResult {
        if slave_addr > 0x7F {
            return MasterResult::InvalidAddr;
        }
        self.send_stale_stop().await;
        unsafe {
            // Set the receiving slave address.
            driverlib::I2CMasterSlaveAddrSet(driverlib::I2C0_BASE, slave_addr, false);
            // Transmit one byte.
            driverlib::I2CMasterDataPut(driverlib::I2C0_BASE, buf);
            // Wait until master controller is not busy.
            self.await_master_completion().await;
            // Write 0-111 to MCTRL.
            driverlib::I2CMasterControl(driverlib::I2C0_BASE, driverlib::I2C_MASTER_CMD_SINGLE_SEND);
            // Wait until master controller is not busy.
            self.await_master_completion().await;
            self.handle_transaction_result()
        }
    }

    /// Tries to read the data one byte at a time into the provided buf.
    /// buf size determines the amount of bytes to be read.
    /// Returns the number of bytes read, and in case of any errors,
    /// the error itself.
    pub async fn read(&mut self, slave_addr: u8, buf: &mut [u8]) -> (usize, MasterResult) {
        if slave_addr > 0x7F {
            return (0, MasterResult::InvalidAddr);
        }
        self.send_stale_stop().await;
        if buf.is_empty() {
            return (0, MasterResult::Ok);
        }

        unsafe {
            // Set the receiving slave address.
            driverlib::I2CMasterSlaveAddrSet(driverlib::I2C0_BASE, slave_addr, true);
            // Wait until master controller is not busy.
            self.await_master_completion().await;
            // Write 0-111 (single byte) or 0-1011 (burst start) to MCTRL.
            driverlib::I2CMasterControl(driverlib::I2C0_BASE, Self::first_recv_cmd(buf.len()));
            let drop = self.get_drop_guard(driverlib::I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP);

            for i in 1usize..buf.len() {
                // Wait until master controller is not busy.
                self.await_master_completion().await;

                // The status refers to buf[i - 1].
                let transaction_result = self.handle_transaction_result();
                if transaction_result != MasterResult::Ok {
                    return (i - 1, transaction_result);
                }
                // Read one byte.
                buf[i - 1] = driverlib::I2CMasterDataGet(driverlib::I2C0_BASE) as u8;

                // Write 0-101 (last byte, NACK) or 0-1001 to MCTRL.
                driverlib::I2CMasterControl(driverlib::I2C0_BASE, Self::next_recv_cmd(i, buf.len()));
            }
            drop.defuse();

            // Wait until master controller is not busy.
            self.await_master_completion().await;

            let transaction_result = self.handle_transaction_result();
            if transaction_result != MasterResult::Ok {
                return (buf.len() - 1, transaction_result);
            }
            // Read the last byte.
            buf[buf.len() - 1] = driverlib::I2CMasterDataGet(driverlib::I2C0_BASE) as u8;

            // Idle
            (buf.len(), MasterResult::Ok)
        }
    }

    pub fn read_blocking(&mut self, slave_addr: u8, buf: &mut [u8]) -> (usize, MasterResult) {
        if slave_addr > 0x7F {
            return (0, MasterResult::InvalidAddr);
        }
        self.send_stale_stop_blocking();
        if buf.is_empty() {
            return (0, MasterResult::Ok);
        }

        unsafe {
            // Set the receiving slave address.
            driverlib::I2CMasterSlaveAddrSet(driverlib::I2C0_BASE, slave_addr, true);
            // Wait until master controller is not busy.
            while driverlib::I2CMasterBusy(driverlib::I2C0_BASE) {}
            // Write 0-111 (single byte) or 0-1011 (burst start) to MCTRL.
            driverlib::I2CMasterControl(driverlib::I2C0_BASE, Self::first_recv_cmd(buf.len()));
            let drop = self.get_drop_guard(driverlib::I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP);

            for i in 1usize..buf.len() {
                // Wait until master controller is not busy.
                while driverlib::I2CMasterBusy(driverlib::I2C0_BASE) {}

                // The status refers to buf[i - 1].
                let transaction_result = self.handle_transaction_result();
                if transaction_result != MasterResult::Ok {
                    return (i - 1, transaction_result);
                }

                // Transfer one byte.
                buf[i - 1] = driverlib::I2CMasterDataGet(driverlib::I2C0_BASE) as u8;

                // Write 0-101 (last byte, NACK) or 0-1001 to MCTRL.
                driverlib::I2CMasterControl(driverlib::I2C0_BASE, Self::next_recv_cmd(i, buf.len()));
            }
            drop.defuse();

            // Wait until master controller is not busy.
            while driverlib::I2CMasterBusy(driverlib::I2C0_BASE) {}

            let transaction_result = self.handle_transaction_result();
            if transaction_result != MasterResult::Ok {
                return (buf.len() - 1, transaction_result);
            }
            // Read the last byte.
            buf[buf.len() - 1] = driverlib::I2CMasterDataGet(driverlib::I2C0_BASE) as u8;

            // Idle
            (buf.len(), MasterResult::Ok)
        }
    }

    /// Tries to write the contents of the buf over I2C, one byte at a time.
    /// Returns the number of bytes sent, and in case of any errors,
    /// the error itself.
    pub async fn write(&mut self, slave_addr: u8, buf: &[u8]) -> (usize, MasterResult) {
        if slave_addr > 0x7F {
            return (0, MasterResult::InvalidAddr);
        }
        self.send_stale_stop().await;
        if buf.is_empty() {
            return (0, MasterResult::Ok);
        }

        unsafe {
            // Set the receiving slave address.
            driverlib::I2CMasterSlaveAddrSet(driverlib::I2C0_BASE, slave_addr, false);
            // Wait until master controller is not busy.
            self.await_master_completion().await;
            // START sends the address followed by MDR, so MDR must hold the first byte.
            driverlib::I2CMasterDataPut(driverlib::I2C0_BASE, buf[0]);
            // Write 0-111 (single byte) or 0-011 (burst start) to MCTRL.
            driverlib::I2CMasterControl(driverlib::I2C0_BASE, Self::first_send_cmd(buf.len()));
            let drop = self.get_drop_guard(driverlib::I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);

            for i in 1usize..buf.len() {
                // Wait until master controller is not busy.
                self.await_master_completion().await;

                // The status refers to buf[i - 1].
                let transaction_result = self.handle_transaction_result();
                if transaction_result != MasterResult::Ok {
                    return (i - 1, transaction_result);
                }

                // Transfer one byte.
                driverlib::I2CMasterDataPut(driverlib::I2C0_BASE, buf[i]);
                // Write 0-101 (last byte) or 0-001 to MCTRL.
                driverlib::I2CMasterControl(driverlib::I2C0_BASE, Self::next_send_cmd(i, buf.len()));
            }
            drop.defuse();

            // Wait until master controller is not busy.
            self.await_master_completion().await;

            let transaction_result = self.handle_transaction_result();
            if transaction_result != MasterResult::Ok {
                return (buf.len() - 1, transaction_result);
            }

            // Idle
            (buf.len(), MasterResult::Ok)
        }
    }

    // Same as write, but blocking.
    pub fn write_blocking(&mut self, slave_addr: u8, buf: &[u8]) -> (usize, MasterResult) {
        if slave_addr > 0x7F {
            return (0, MasterResult::InvalidAddr);
        }
        self.send_stale_stop_blocking();
        if buf.is_empty() {
            return (0, MasterResult::Ok);
        }

        unsafe {
            // Set the receiving slave address.
            driverlib::I2CMasterSlaveAddrSet(driverlib::I2C0_BASE, slave_addr, false);
            // Wait until master controller is not busy.
            while driverlib::I2CMasterBusy(driverlib::I2C0_BASE) {}
            // START sends the address followed by MDR, so MDR must hold the first byte.
            driverlib::I2CMasterDataPut(driverlib::I2C0_BASE, buf[0]);
            // Write 0-111 (single byte) or 0-011 (burst start) to MCTRL.
            driverlib::I2CMasterControl(driverlib::I2C0_BASE, Self::first_send_cmd(buf.len()));
            let drop = self.get_drop_guard(driverlib::I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);

            for i in 1usize..buf.len() {
                // Wait until master controller is not busy.
                while driverlib::I2CMasterBusy(driverlib::I2C0_BASE) {}

                // The status refers to buf[i - 1].
                let transaction_result = self.handle_transaction_result();
                if transaction_result != MasterResult::Ok {
                    return (i - 1, transaction_result);
                }

                // Transfer one byte.
                driverlib::I2CMasterDataPut(driverlib::I2C0_BASE, buf[i]);
                // Write 0-101 (last byte) or 0-001 to MCTRL.
                driverlib::I2CMasterControl(driverlib::I2C0_BASE, Self::next_send_cmd(i, buf.len()));
            }
            drop.defuse();
            // Wait until master controller is not busy.
            while driverlib::I2CMasterBusy(driverlib::I2C0_BASE) {}

            let transaction_result = self.handle_transaction_result();
            if transaction_result != MasterResult::Ok {
                return (buf.len() - 1, transaction_result);
            }

            // Idle
            (buf.len(), MasterResult::Ok)
        }
    }
}

pub struct Runner<'d> {
    state: &'d State,
}

impl<'d> Runner<'d> {
    pub(crate) fn new(state: &'static State) -> Self {
        Self { state }
    }

    pub async fn step(&mut self, read_callback: impl Fn(u8), write_callback: impl Fn() -> u8) {
        let _ = poll_fn(|cx| {
            self.state.slave_waker.register(cx.waker());
            let slave_status = unsafe { driverlib::I2CSlaveStatus(driverlib::I2C0_BASE) };
            if slave_status & driverlib::I2C_SLAVE_ACT_RREQ != 0 {
                let data = unsafe { driverlib::I2CSlaveDataGet(driverlib::I2C0_BASE) };
                read_callback(data as u8);
                return Poll::Ready(());
            } else if slave_status & driverlib::I2C_SLAVE_ACT_TREQ != 0 {
                let data = write_callback();
                unsafe {
                    driverlib::I2CSlaveDataPut(driverlib::I2C0_BASE, data);
                };
                return Poll::Ready(());
            }
            Poll::Pending
        })
        .await;
    }
}

pub struct I2CSlave<'a, T: Instance> {
    runner: Runner<'a>,
    _p: PhantomData<(&'a (), T)>,
}

impl<'a, T: Instance> I2CSlave<'a, T> {
    pub(crate) fn new(
        config: SlaveConfig,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'a,
    ) -> Self {
        assert!(config.addr <= 0x7F, "I2C slave address must be 7-bit");
        unsafe {
            // Setup pins.
            driverlib::IOCPinTypeI2c(driverlib::IOC_BASE, config.sda_pin, config.scl_pin);

            // Initialize I2C slave.
            driverlib::I2CSlaveInit(driverlib::I2C0_BASE, config.addr);
            driverlib::I2CSlaveIntEnable(
                driverlib::I2C0_BASE,
                driverlib::I2C_SLAVE_INT_STOP | driverlib::I2C_SLAVE_INT_START | driverlib::I2C_SLAVE_INT_DATA,
            );
        };

        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        Self {
            runner: Runner::new(T::state()),
            _p: PhantomData,
        }
    }

    pub fn into_runner(self) -> Runner<'a> {
        self.runner
    }
}

pub fn new_master<'a, T: Instance>(
    _i2c: Peri<'a, T>,
    master_config: MasterConfig,
    irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'a,
) -> I2CMaster<'a, T> {
    I2CMaster::new(master_config, irq)
}

pub fn new_slave<'a, T: Instance>(
    _i2c: Peri<'a, T>,
    slave_config: SlaveConfig,
    irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'a,
) -> I2CSlave<'a, T> {
    I2CSlave::new(slave_config, irq)
}

pub fn new_master_slave<'a, T: Instance>(
    _i2c: Peri<'a, T>,
    master_config: MasterConfig,
    slave_config: SlaveConfig,
    irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'a,
) -> (I2CMaster<'a, T>, I2CSlave<'a, T>) {
    assert!(
        master_config.sda_pin == slave_config.sda_pin && master_config.scl_pin == slave_config.scl_pin,
        "I2C master and slave must share SDA/SCL pins"
    );
    (I2CMaster::new(master_config, irq), I2CSlave::new(slave_config, irq))
}

pub(crate) use impl_i2c;
