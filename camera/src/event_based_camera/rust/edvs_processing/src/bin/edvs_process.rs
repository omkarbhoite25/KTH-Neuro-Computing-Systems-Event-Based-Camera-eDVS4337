use edvs_processing::denoise::TemporalFilter;
use edvs_processing::event::Event;
use edvs_processing::hot_pixel::HotPixelFilter;

use std::io::{self, BufRead, Write};

fn print_usage() {
    eprintln!("edvs-process: Filter eDVS events from stdin");
    eprintln!();
    eprintln!("Usage: edvs-process [OPTIONS]");
    eprintln!();
    eprintln!("Reads tab-separated events (x y timestamp polarity) from stdin,");
    eprintln!("applies filters, and writes passing events to stdout.");
    eprintln!();
    eprintln!("Options:");
    eprintln!("  --width N            Sensor width (default: 128)");
    eprintln!("  --height N           Sensor height (default: 128)");
    eprintln!("  --denoise-us N       Temporal denoise threshold in us (default: 5000, 0 = off)");
    eprintln!("  --hot-pixel-rate N   Max events/pixel/window (default: 1000, 0 = off)");
    eprintln!("  --hot-pixel-window N Hot pixel window in us (default: 1000000)");
    eprintln!("  --help               Show this help");
}

fn main() {
    let args: Vec<String> = std::env::args().collect();

    let mut width: u32 = 128;
    let mut height: u32 = 128;
    let mut denoise_us: i64 = 5000;
    let mut hp_rate: u32 = 1000;
    let mut hp_window: i64 = 1_000_000;

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--width" => {
                i += 1;
                width = args[i].parse().expect("invalid --width");
            }
            "--height" => {
                i += 1;
                height = args[i].parse().expect("invalid --height");
            }
            "--denoise-us" => {
                i += 1;
                denoise_us = args[i].parse().expect("invalid --denoise-us");
            }
            "--hot-pixel-rate" => {
                i += 1;
                hp_rate = args[i].parse().expect("invalid --hot-pixel-rate");
            }
            "--hot-pixel-window" => {
                i += 1;
                hp_window = args[i].parse().expect("invalid --hot-pixel-window");
            }
            "--help" | "-h" => {
                print_usage();
                return;
            }
            other => {
                eprintln!("Unknown option: {}", other);
                print_usage();
                std::process::exit(1);
            }
        }
        i += 1;
    }

    let mut temporal_filter = if denoise_us > 0 {
        Some(TemporalFilter::new(width, height, denoise_us))
    } else {
        None
    };

    let mut hot_pixel_filter = if hp_rate > 0 {
        Some(HotPixelFilter::new(width, height, hp_window, hp_rate))
    } else {
        None
    };

    let stdin = io::stdin();
    let stdout = io::stdout();
    let mut out = io::BufWriter::new(stdout.lock());

    let mut total: u64 = 0;
    let mut passed: u64 = 0;

    for line in stdin.lock().lines() {
        let line = match line {
            Ok(l) => l,
            Err(_) => break,
        };

        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }

        let parts: Vec<&str> = line.split('\t').collect();
        if parts.len() < 4 {
            continue;
        }

        let x: u16 = match parts[0].trim().parse() {
            Ok(v) => v,
            Err(_) => continue,
        };
        let y: u16 = match parts[1].trim().parse() {
            Ok(v) => v,
            Err(_) => continue,
        };
        let ts: i64 = match parts[2].trim().parse() {
            Ok(v) => v,
            Err(_) => continue,
        };
        let pol: i8 = match parts[3].trim().parse() {
            Ok(v) => v,
            Err(_) => continue,
        };

        let event = Event::new(x, y, ts, pol);
        total += 1;

        if let Some(ref mut f) = temporal_filter {
            if !f.filter(&event) {
                continue;
            }
        }
        if let Some(ref mut f) = hot_pixel_filter {
            if !f.filter(&event) {
                continue;
            }
        }

        passed += 1;
        let _ = writeln!(out, "{}", event);
    }

    eprintln!(
        "edvs-process: {}/{} events passed filters ({:.1}%)",
        passed,
        total,
        if total > 0 {
            100.0 * passed as f64 / total as f64
        } else {
            0.0
        }
    );
}
