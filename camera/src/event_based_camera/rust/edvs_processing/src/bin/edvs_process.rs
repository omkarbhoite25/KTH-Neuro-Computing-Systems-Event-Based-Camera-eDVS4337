use edvs_processing::denoise::TemporalFilter;
use edvs_processing::event::Event;
use edvs_processing::hot_pixel::HotPixelFilter;

use std::io::{self, BufRead, Write};

struct Config {
    width: u32,
    height: u32,
    denoise_us: i64,
    hp_rate: u32,
    hp_window: i64,
}

impl Config {
    const DEFAULT_WIDTH: u32 = 128;
    const DEFAULT_HEIGHT: u32 = 128;
    const DEFAULT_DENOISE_US: i64 = 5000;
    const DEFAULT_HP_RATE: u32 = 1000;
    const DEFAULT_HP_WINDOW: i64 = 1_000_000;

    fn from_args(args: &[String]) -> Result<Self, String> {
        let mut config = Self {
            width: Self::DEFAULT_WIDTH,
            height: Self::DEFAULT_HEIGHT,
            denoise_us: Self::DEFAULT_DENOISE_US,
            hp_rate: Self::DEFAULT_HP_RATE,
            hp_window: Self::DEFAULT_HP_WINDOW,
        };

        let mut i = 1;
        while i < args.len() {
            match args[i].as_str() {
                "--width" => {
                    i += 1;
                    config.width = Self::parse_arg(args.get(i), "--width")?;
                }
                "--height" => {
                    i += 1;
                    config.height = Self::parse_arg(args.get(i), "--height")?;
                }
                "--denoise-us" => {
                    i += 1;
                    config.denoise_us = Self::parse_arg(args.get(i), "--denoise-us")?;
                }
                "--hot-pixel-rate" => {
                    i += 1;
                    config.hp_rate = Self::parse_arg(args.get(i), "--hot-pixel-rate")?;
                }
                "--hot-pixel-window" => {
                    i += 1;
                    config.hp_window = Self::parse_arg(args.get(i), "--hot-pixel-window")?;
                }
                "--help" | "-h" => return Err(String::new()), // empty string signals help
                other => return Err(format!("Unknown option: {}", other)),
            }
            i += 1;
        }

        config.validate()?;
        Ok(config)
    }

    fn parse_arg<T: std::str::FromStr>(arg: Option<&String>, name: &str) -> Result<T, String> {
        arg.ok_or_else(|| format!("{} requires a value", name))?
            .parse()
            .map_err(|_| format!("invalid value for {}", name))
    }

    fn validate(&self) -> Result<(), String> {
        if self.width == 0 || self.height == 0 {
            return Err("width and height must be positive".into());
        }
        if self.denoise_us < 0 {
            return Err("denoise-us must be non-negative".into());
        }
        if self.hp_window <= 0 && self.hp_rate > 0 {
            return Err("hot-pixel-window must be positive when hot-pixel-rate is set".into());
        }
        Ok(())
    }
}

fn print_usage() {
    eprintln!("edvs-process: Filter eDVS events from stdin");
    eprintln!();
    eprintln!("Usage: edvs-process [OPTIONS]");
    eprintln!();
    eprintln!("Reads tab-separated events (x y timestamp polarity) from stdin,");
    eprintln!("applies filters, and writes passing events to stdout.");
    eprintln!();
    eprintln!("Options:");
    eprintln!("  --width N            Sensor width (default: {})", Config::DEFAULT_WIDTH);
    eprintln!("  --height N           Sensor height (default: {})", Config::DEFAULT_HEIGHT);
    eprintln!("  --denoise-us N       Temporal denoise threshold in us (default: {}, 0 = off)", Config::DEFAULT_DENOISE_US);
    eprintln!("  --hot-pixel-rate N   Max events/pixel/window (default: {}, 0 = off)", Config::DEFAULT_HP_RATE);
    eprintln!("  --hot-pixel-window N Hot pixel window in us (default: {})", Config::DEFAULT_HP_WINDOW);
    eprintln!("  --help               Show this help");
}

fn main() {
    let args: Vec<String> = std::env::args().collect();

    let config = match Config::from_args(&args) {
        Ok(c) => c,
        Err(msg) => {
            if !msg.is_empty() {
                eprintln!("Error: {}", msg);
            }
            print_usage();
            std::process::exit(if msg.is_empty() { 0 } else { 1 });
        }
    };

    let mut temporal_filter = if config.denoise_us > 0 {
        Some(TemporalFilter::new(config.width, config.height, config.denoise_us))
    } else {
        None
    };

    let mut hot_pixel_filter = if config.hp_rate > 0 {
        Some(HotPixelFilter::new(config.width, config.height, config.hp_window, config.hp_rate))
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
            Ok(v) if v == -1 || v == 1 => v,
            _ => continue,  // only valid polarities are -1 and +1
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
