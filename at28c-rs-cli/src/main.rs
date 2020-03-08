use colored::*;
use indicatif::{ProgressBar, ProgressStyle};
use serial::{
    core::{Error as SerialError, FlowControl},
    prelude::*,
};
use std::{
    ffi::OsStr,
    fs::OpenOptions,
    io::{self, prelude::*},
    path::PathBuf,
    process,
    time::{Duration, Instant},
};
use structopt::StructOpt;
use thiserror::Error;

mod cmds;
use cmds::{Commands, Response};

const COMMAND_SIZE: usize = 4;
const PAGE_SIZE: u16 = 64;

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
    #[error("File too big")]
    FileTooBig,
    #[error("For now only .bin files are supported")]
    WrongFileType,
    #[error("Verification pass fail")]
    VerificationError,
    #[error("Invalid Device")]
    InvalidDevice,
}

#[derive(Copy, Clone, Debug, PartialEq)]
enum Device {
    AT28C256,
    AT28C64,
    Invalid,
}

impl Device {
    pub fn rom_size(self) -> u16 {
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

    /// Unlock the device for writing
    #[structopt(short, long)]
    unlock: bool,

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
        Ok(_) => println!("{}", "Done".green().bold()),
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

    if opt.device == Device::Invalid {
        return Err(CliError::InvalidDevice);
    }

    let rom_size = opt.device.rom_size();
    let mut port = serial::open(&opt.port)?;
    port.set_timeout(Duration::from_secs(1))?;
    port.reconfigure(&|settings| {
        settings.set_flow_control(FlowControl::FlowNone);
        Ok(())
    })?;
    init(&mut port)?;

    if opt.unlock {
        unlock(&mut port, opt.device)?;
        println!("{}", "Device unlocked".yellow());
        return Ok(());
    }

    if opt.write {
        write_from_file(&mut port, opt.file, rom_size)
    } else {
        read_to_file(&mut port, opt.file, rom_size)
    }
}

fn init(port: &mut impl SerialPort) -> Result<(), CliError> {
    let mut command: [u8; COMMAND_SIZE] = [Commands::QueryState.into(), 0x00, 0x00, 0x00];
    if port.write(&command)? != COMMAND_SIZE {
        return Err(CliError::DeviceError);
    }
    port.flush()?;
    let _ = port.read(&mut command[..1])?;
    match Response::from(command[0]) {
        Response::Disconnected => {
            command[0] = Commands::Connect.into();
            write_cmd_check_response(port, command, Response::Connected)?;
        }
        Response::Idle => {}
        _ => {
            return Err(CliError::DeviceError);
        }
    }
    Ok(())
}

fn write_from_file(
    port: &mut impl SerialPort,
    path: PathBuf,
    max_size: u16,
) -> Result<(), CliError> {
    if path.extension().ok_or(CliError::WrongFileType)? != "bin" {
        return Err(CliError::WrongFileType);
    }
    let mut bytes = Vec::with_capacity(max_size as usize);
    let mut file = OpenOptions::new().read(true).open(path)?;
    if file.read_to_end(&mut bytes)? > max_size as usize {
        return Err(CliError::FileTooBig);
    }
    let mut addr = 0;
    let pb_write = ProgressBar::new(bytes.len() as u64);
    pb_write.set_message("Wrinting");
    pb_write.set_style(
        ProgressStyle::default_bar()
            .template("{msg:.green.bold} [{bar:40.cyan/blue}] {bytes}/{total_bytes}")
            .progress_chars("#>-"),
    );
    let write_time = Instant::now();
    for page in bytes.chunks(PAGE_SIZE as usize) {
        write_page(port, &page, addr)?;
        addr += PAGE_SIZE;
        pb_write.set_position(u64::from(addr));
    }
    let elapsed = write_time.elapsed();
    pb_write.finish();
    println!(
        "Writing took {}.{}s",
        elapsed.as_secs(),
        elapsed.as_millis()
    );

    addr = 0;
    let pb_read = ProgressBar::new(bytes.len() as u64);
    pb_read.set_message("Verifying");
    pb_read.set_style(
        ProgressStyle::default_bar()
            .template("{msg:.green.bold} [{bar:40.cyan/blue}] {bytes}/{total_bytes}")
            .progress_chars("#>-"),
    );
    let veri_time = Instant::now();
    for page in bytes.chunks(PAGE_SIZE as usize) {
        let mut buf = [0u8; PAGE_SIZE as usize];
        // NOTE(unsafe) buf has PAGE_SIZE length
        unsafe { read_page(port, &mut buf, addr)? };
        if &buf[..page.len()] != page {
            return Err(CliError::VerificationError);
        }
        addr += PAGE_SIZE;
        pb_read.set_position(u64::from(addr));
    }
    let elapsed = veri_time.elapsed();
    pb_read.finish();
    println!("{}", "Verification OK".green());
    println!(
        "Verifying took {}.{}s",
        elapsed.as_secs(),
        elapsed.as_millis()
    );
    Ok(())
}

fn read_to_file(port: &mut impl SerialPort, path: PathBuf, max_size: u16) -> Result<(), CliError> {
    let mut file = OpenOptions::new().write(true).create_new(true).open(path)?;
    let mut addr = 0;

    let pb_read = ProgressBar::new(u64::from(max_size));
    pb_read.set_message("Reading");
    pb_read.set_style(
        ProgressStyle::default_bar()
            .template("{msg:.green.blod} [{bar:40.cyan/blue}] {bytes}/{total_bytes}")
            .progress_chars("#>-"),
    );
    let read_time = Instant::now();
    loop {
        let mut buf = [0u8; PAGE_SIZE as usize];
        // NOTE(unsafe) buf has PAGE_SIZE length
        unsafe { read_page(port, &mut buf, addr)? };
        file.write_all(&buf)?;
        addr += PAGE_SIZE;
        pb_read.set_position(u64::from(addr));
        if addr >= max_size {
            break;
        }
    }
    let elapsed = read_time.elapsed();
    pb_read.finish();
    println!(
        "Reading took {}.{}s",
        elapsed.as_secs(),
        elapsed.as_millis()
    );
    file.flush()?;
    Ok(())
}

fn write_page(port: &mut impl SerialPort, buf: &[u8], addr: u16) -> Result<(), CliError> {
    let mut command: [u8; COMMAND_SIZE] = [Commands::WritePage.into(), 0x00, 0x00, 0x00];
    command[1..3].copy_from_slice(&addr.to_le_bytes()[..]);
    write_cmd_check_response(port, command, Response::SendPage)?;
    if port.write(&buf)? != buf.len() {
        return Err(CliError::DeviceError);
    }
    port.flush()?;
    check_response(port, Response::WriteDone)?;
    Ok(())
}

/// # Safety
/// `buf` length must be equal to `PAGE_SIZE`
unsafe fn read_page(port: &mut impl SerialPort, buf: &mut [u8], addr: u16) -> Result<(), CliError> {
    let mut command: [u8; COMMAND_SIZE] = [Commands::ReadPage.into(), 0x00, 0x00, 0x00];
    command[1..3].copy_from_slice(&addr.to_le_bytes()[..]);
    if port.write(&command)? != COMMAND_SIZE {
        return Err(CliError::DeviceError);
    }
    port.flush()?;

    port.read_exact(buf)?;
    Ok(())
}

fn unlock(port: &mut impl SerialPort, device: Device) -> Result<(), CliError> {
    let command: [u8; COMMAND_SIZE] = match device {
        Device::AT28C256 => [Commands::DisableProctetion256.into(), 0x00, 0x00, 0x00],
        Device::AT28C64 => [Commands::DisableProctetion64.into(), 0x00, 0x00, 0x00],
        Device::Invalid => return Err(CliError::InvalidDevice),
    };
    write_cmd_check_response(port, command, Response::WriteDone)
}

fn write_cmd_check_response(
    port: &mut impl SerialPort,
    cmd: [u8; 4],
    expected: Response,
) -> Result<(), CliError> {
    if port.write(&cmd)? != COMMAND_SIZE {
        return Err(CliError::DeviceError);
    }
    port.flush()?;
    check_response(port, expected)
}

fn check_response(port: &mut impl SerialPort, expected: Response) -> Result<(), CliError> {
    let mut buf = [0];
    let _ = port.read(&mut buf)?;
    if Response::from(buf[0]) != expected {
        Err(CliError::DeviceError)
    } else {
        Ok(())
    }
}
