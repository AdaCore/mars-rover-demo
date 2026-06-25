//! Sidecar metadata files written alongside a trace CSV.
//!
//! Two sidecars live next to `<path>.csv`:
//!
//! - `<path>.csv.wpts` — one waypoint per line, `x y` (space-separated).  Read
//!   by `mars-rover-replay` so Ada receives the same goals it had during the
//!   recording.
//! - `<path>.csv.meta` — `key: value` lines carrying the parameters needed to
//!   reconstruct the scene for the trace-driven GUI renderer (terrain seed,
//!   GPS cadence, RNG seed, slip-free flag).
//!
//! Both formats are intentionally trivial text so they round-trip in a shell
//! without pulling in serde.  Keeping the writer here (rather than duplicating
//! it in `simulator/src/logger.rs` and `crates/mars-rover-headless/src/logger.rs`)
//! prevents drift between the two drivers.

use std::{
    fs::File,
    io::{self, Write},
    path::Path,
};

/// Parameters required to reconstruct the scene the trace was recorded in.
#[derive(Debug, Clone)]
pub struct TraceMeta {
    pub seed: u64,
    pub gps_interval_ms: u32,
    pub noise_seed: u64,
    pub slip_free: bool,
}

/// Write `<log_path>.wpts` and `<log_path>.meta` next to the given log file.
/// If `waypoints` is empty, the `.wpts` sidecar is skipped (matches the
/// pre-Phase-4 behaviour the replay binary relies on).
pub fn write_sidecars(log_path: &Path, meta: &TraceMeta, waypoints: &[(f64, f64)]) -> io::Result<()> {
    if !waypoints.is_empty() {
        let wpts_path = sidecar_path(log_path, "wpts");
        let mut f = File::create(&wpts_path)?;
        writeln!(f, "# Waypoints recorded alongside {}", log_path.display())?;
        for (x, y) in waypoints {
            writeln!(f, "{} {}", x, y)?;
        }
    }

    let meta_path = sidecar_path(log_path, "meta");
    let mut f = File::create(&meta_path)?;
    writeln!(f, "# Metadata for {}", log_path.display())?;
    writeln!(f, "seed: {}", meta.seed)?;
    writeln!(f, "gps_interval_ms: {}", meta.gps_interval_ms)?;
    writeln!(f, "noise_seed: {}", meta.noise_seed)?;
    writeln!(f, "slip_free: {}", meta.slip_free)?;
    Ok(())
}

/// Read `<log_path>.meta` if present.  Returns `None` when the file is absent
/// or unreadable (the replay loader then falls back to the `--seed` CLI flag).
pub fn read_meta(log_path: &Path) -> Option<TraceMeta> {
    let content = std::fs::read_to_string(sidecar_path(log_path, "meta")).ok()?;
    let mut seed: Option<u64> = None;
    let mut gps_interval_ms: Option<u32> = None;
    let mut noise_seed: Option<u64> = None;
    let mut slip_free: Option<bool> = None;

    for line in content.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let Some((k, v)) = line.split_once(':') else { continue };
        let v = v.trim();
        match k.trim() {
            "seed" => seed = v.parse().ok(),
            "gps_interval_ms" => gps_interval_ms = v.parse().ok(),
            "noise_seed" => noise_seed = v.parse().ok(),
            "slip_free" => slip_free = v.parse().ok(),
            _ => {}
        }
    }

    Some(TraceMeta {
        seed: seed?,
        gps_interval_ms: gps_interval_ms.unwrap_or(0),
        noise_seed: noise_seed.unwrap_or(0),
        slip_free: slip_free.unwrap_or(false),
    })
}

/// Read `<log_path>.wpts` if present.  Accepts both `x y` and `x,y` separators
/// and skips `#` comment lines, matching the loaders in the GUI and headless
/// drivers.
pub fn read_waypoints(log_path: &Path) -> Vec<(f64, f64)> {
    let Ok(content) = std::fs::read_to_string(sidecar_path(log_path, "wpts")) else {
        return Vec::new();
    };
    let mut wps = Vec::new();
    for line in content.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let parts: Vec<&str> = if line.contains(',') {
            line.splitn(2, ',').collect()
        } else {
            line.splitn(2, ' ').collect()
        };
        if parts.len() == 2 {
            if let (Ok(x), Ok(y)) = (
                parts[0].trim().parse::<f64>(),
                parts[1].trim().parse::<f64>(),
            ) {
                wps.push((x, y));
            }
        }
    }
    wps
}

fn sidecar_path(log_path: &Path, ext: &str) -> std::path::PathBuf {
    let mut s = log_path.as_os_str().to_owned();
    s.push(".");
    s.push(ext);
    std::path::PathBuf::from(s)
}
