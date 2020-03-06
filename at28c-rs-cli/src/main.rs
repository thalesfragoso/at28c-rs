use colored::*;
use serial::{core::Error as SerialError, prelude::*};
use std::{
    ffi::OsStr,
    io::{self, prelude::*},
    path::PathBuf,
    process,
    time::Duration,
};
use structopt::StructOpt;
use thiserror::Error;

mod cmds;
use cmds::{Commands, Response};

#[derive(Debug, Error)]
enum CliError {
    #[error("Can not use read and write flags at the same time")]
    ReadAndWrite,
    #[error("Must specify a read or write operation")]
    NoReadOrWrite,
    #[error("Comunication failure: `{0}`")]
    IoError(#[from] io::Error),
    #[error("Serial failure: `{0}`")]
    SerialError(#[from] SerialError),
    #[error("Unexpected Response")]
    DeviceError,
}

#[derive(Copy, Clone, Debug)]
enum Device {
    AT28C256,
    AT28C64,
    Invalid,
}

impl Device {
    pub fn rom_size(&self) -> u32 {
        match self {
            Device::AT28C256 => 32768,
            Device::AT28C64 => 8192,
            Device::Invalid => 0,
        }
    }
}

impl From<&OsStr> for Device {
    fn from(device: &OsStr) -> Self {
        let device_name = device.to_str();
        if let Some(name) = device_name {
            String::from(name).make_ascii_lowercase();
            match name {
                "at28c256" => Device::AT28C256,
                "at28c64" => Device::AT28C64,
                _ => Device::Invalid,
            }
        } else {
            Device::Invalid
        }
    }
}

#[derive(StructOpt, Debug)]
struct Opt {
    /// Write to device
    #[structopt(short, long, required_if("read", "false"))]
    write: bool,

    /// Read from device
    #[structopt(short, long)]
    read: bool,

    /// Serial port where the device is attached
    #[structopt(short, long, parse(from_os_str))]
    port: PathBuf,

    /// Device type, AT28C256 or AT28C64
    #[structopt(short, long, parse(from_os_str))]
    device: Device,

    /// File to write from or read to
    #[structopt(short, long, parse(from_os_str))]
    file: PathBuf,
}

fn main() {
    match try_main() {
        Ok(_) => (),
        Err(e) => {
            eprintln!("{} : {}", "error".red().bold(), e);
            process::exit(1);
        }
    }
}

fn try_main() -> Result<(), CliError> {
    let opt = Opt::from_args();
    if opt.write && opt.read {
        return Err(CliError::ReadAndWrite);
    } else if !opt.write && !opt.read {
        return Err(CliError::NoReadOrWrite);
    }

    let mut port = serial::open(&opt.port)?;
    port.set_timeout(Duration::from_secs(1))?;
    init(&mut port)?;

    Ok(())
}

fn init<T: SerialPort>(port: &mut T) -> Result<(), CliError> {
    let mut buf: [u8; 4] = [Commands::QueryState.into(), 0x00, 0x00, 0x00];
    let count = port.write(&buf)?;
    if count != 4 {
        return Err(CliError::DeviceError);
    }
    port.flush()?;
    let _ = port.read(&mut buf[..1])?;
    match Response::from(buf[0]) {
        Response::Disconnected => {
            buf[0] = Commands::Connect.into();
            let count = port.write(&buf)?;
            if count != 4 {
                return Err(CliError::DeviceError);
            }
            port.flush()?;
            let _ = port.read(&mut buf[..1])?;
        }
        Response::Idle => {}
        _ => {
            return Err(CliError::DeviceError);
        }
    }
    Ok(())
}
