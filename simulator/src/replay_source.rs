//! Trace-row abstraction for the GUI's `--replay` mode.
//!
//! The GUI's replay mode is driven by a `TraceSource`: an iterator-like producer
//! of `TraceRow` values that the `trace_reader` system consumes each frame to
//! drive the scene.  Today the only implementation is file-backed
//! (`FileTraceSource`, reading the 24-column CSV that both the GUI and headless
//! loggers emit).  A future socket-backed source (for live streaming from a
//! headless run) drops in here — the renderer itself stays source-agnostic.

use std::{
    fs::File,
    io::{self, BufRead, BufReader},
    path::PathBuf,
};

/// One row of recorded sim state.  Column layout mirrors the 24-column CSV
/// header shared by `crates/mars-rover-headless/src/logger.rs` and
/// `simulator/src/logger.rs`.
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct TraceRow {
    pub time_ms: u64,
    pub truth: (f64, f64, f64),
    pub est: Option<(f64, f64, f64)>,
    /// Steering angles in degrees: `[FL, FR, RL, RR]`.  Convention pinned by
    /// the CSV column order — do NOT reorder based on `WheelID` discriminants.
    pub steer: [f64; 4],
    pub power: (i8, i8),
    pub enc: [i32; 4],
    /// GPS fix: `None` on frames without a new fix.  Coordinates in metres.
    pub gps: Option<(f64, f64, u32)>,
    pub sonar_cm: u32,
    pub imu_gyro_z: i16,
    pub mast_deg: i8,
    pub ada_step_us: u64,
}

/// Pluggable source of trace rows.  File-backed is the first implementation;
/// a future socket-backed version will implement the same trait.
pub trait TraceSource: Send {
    /// Return the next row, or `None` on end-of-stream.
    fn next_row(&mut self) -> Option<TraceRow>;

    /// Rewind to the beginning of the stream, if the source supports it.
    /// Default: no-op returning an error so non-seekable sources (e.g. sockets)
    /// signal their limitation.
    fn reset(&mut self) -> io::Result<()> {
        Err(io::Error::new(
            io::ErrorKind::Unsupported,
            "this TraceSource does not support reset",
        ))
    }
}

/// File-backed `TraceSource`.  Reads the CSV lazily one row per `next_row()`
/// call so the renderer can stay in-sync with wall-clock time without buffering
/// the whole trace.
pub struct FileTraceSource {
    path: PathBuf,
    reader: BufReader<File>,
    line_buf: String,
    /// True once the header line has been consumed.  Reset to false by `reset`.
    header_consumed: bool,
}

impl FileTraceSource {
    pub fn open(path: impl Into<PathBuf>) -> io::Result<Self> {
        let path = path.into();
        let reader = BufReader::new(File::open(&path)?);
        Ok(Self {
            path,
            reader,
            line_buf: String::new(),
            header_consumed: false,
        })
    }
}

impl TraceSource for FileTraceSource {
    fn next_row(&mut self) -> Option<TraceRow> {
        if !self.header_consumed {
            self.line_buf.clear();
            if self.reader.read_line(&mut self.line_buf).ok()? == 0 {
                return None;
            }
            self.header_consumed = true;
        }
        loop {
            self.line_buf.clear();
            let n = self.reader.read_line(&mut self.line_buf).ok()?;
            if n == 0 {
                return None;
            }
            if let Some(row) = parse_row(self.line_buf.trim_end()) {
                return Some(row);
            }
            // Skip unparseable lines (blank / malformed) and try the next one.
        }
    }

    fn reset(&mut self) -> io::Result<()> {
        self.reader = BufReader::new(File::open(&self.path)?);
        self.header_consumed = false;
        Ok(())
    }
}

fn parse_opt<T: std::str::FromStr>(s: &str) -> Option<T> {
    let s = s.trim();
    if s.is_empty() { None } else { s.parse().ok() }
}

/// Parse one CSV row.  Returns `None` on blank / too-short lines; callers
/// should skip and continue.  Numeric parse errors fall back to sensible
/// defaults (the replay binary uses the same strategy), so partially-written
/// trailing rows from an aborted log don't stop playback dead.
fn parse_row(line: &str) -> Option<TraceRow> {
    if line.is_empty() {
        return None;
    }
    let c: Vec<&str> = line.splitn(25, ',').collect();
    if c.len() < 23 {
        return None;
    }

    let log_est_x = parse_opt::<f64>(c[4]);
    let log_est_y = parse_opt::<f64>(c[5]);
    let log_est_t = parse_opt::<f64>(c[6]);
    let est = match (log_est_x, log_est_y, log_est_t) {
        (Some(x), Some(y), Some(t)) => Some((x, y, t)),
        _ => None,
    };

    let gps_x = parse_opt::<f64>(c[17]);
    let gps_y = parse_opt::<f64>(c[18]);
    let gps_ts = parse_opt::<u32>(c[19]);
    let gps = match (gps_x, gps_y, gps_ts) {
        (Some(x), Some(y), Some(ts)) => Some((x, y, ts)),
        _ => None,
    };

    Some(TraceRow {
        time_ms: c[0].trim().parse().unwrap_or(0),
        truth: (
            c[1].trim().parse().unwrap_or(0.0),
            c[2].trim().parse().unwrap_or(0.0),
            c[3].trim().parse().unwrap_or(0.0),
        ),
        est,
        steer: [
            c[7].trim().parse().unwrap_or(0.0),
            c[8].trim().parse().unwrap_or(0.0),
            c[9].trim().parse().unwrap_or(0.0),
            c[10].trim().parse().unwrap_or(0.0),
        ],
        power: (
            c[11].trim().parse().unwrap_or(0),
            c[12].trim().parse().unwrap_or(0),
        ),
        enc: [
            c[13].trim().parse().unwrap_or(0),
            c[14].trim().parse().unwrap_or(0),
            c[15].trim().parse().unwrap_or(0),
            c[16].trim().parse().unwrap_or(0),
        ],
        gps,
        sonar_cm: c[20].trim().parse().unwrap_or(u32::MAX),
        imu_gyro_z: c[21].trim().parse().unwrap_or(0),
        mast_deg: c[22].trim().parse().unwrap_or(0),
        ada_step_us: c.get(23).and_then(|s| s.trim().parse().ok()).unwrap_or(0),
    })
}
