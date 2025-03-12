use colored::Colorize;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::env;
use std::io::{BufRead, BufReader};
use std::process::{self, Command, Stdio};
use std::time::{SystemTime, UNIX_EPOCH};
use tokio::task;

#[derive(Serialize, Deserialize, Debug, Clone)]
struct ImageData {
    image_data: String,
    scale: i32,
    flip: bool,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct GraphDataSettings {
    clear_data: bool,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct GraphDataPoint {
    x: f64,
    y: f64,
    settings: GraphDataSettings,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct RobotPosition {
    x: f64,
    y: f64,
    heading: f64,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct StringData {
    value: String,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
struct RobotData {
    // sent_timestamp: f64,
    // images: HashMap<String, ImageData>,
    graph_data: HashMap<String, Vec<GraphDataPoint>>,
    // string_data: HashMap<String, StringData>,
    // robot_position: RobotPosition,
}

impl Default for RobotData {
    fn default() -> Self {
        RobotData {
            // sent_timestamp: 0.0,
            // images: HashMap::new(),
            graph_data: HashMap::new(),
            // string_data:HashMap::new(),
            // robot_position: RobotPosition {
            // x: 0.0,
            // y: 0.0,
            // heading: 0.0,
            // },
        }
    }
}

async fn send_data(client: &reqwest::Client, robot_data: RobotData) -> bool {
    let json_bytes = task::spawn_blocking(move || serde_json::to_vec(&robot_data).unwrap())
        .await
        .unwrap();

    match client
        .post("http://localhost:3000/batch")
        .header("Content-Type", "application/json")
        .body(json_bytes)
        .send()
        .await
    {
        Ok(_response) => true,
        Err(e) => {
            println!("Error sending request: {}", e);
            false
        }
    }
}

pub fn current_timestamp_nanos() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("Time went backwards")
        .as_nanos()
}

#[tokio::main]
async fn main() -> std::io::Result<()> {
    let client = reqwest::Client::new();
    let mut robot_data = RobotData::default();

    if !send_data(&client, robot_data).await {
        eprintln!(
            "{}",
            "FAILED TO SEND WAGGLE DATA. MAKE SURE THE WAGGLE SERVER IS RUNNING".red()
        );
        process::exit(1);
    }
    robot_data = RobotData::default();

    println!("Starting waggle interceptor!");
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: {} <command> [arguments...]", args[0]);
        std::process::exit(1);
    }

    let command = &args[1];
    let command_args = &args[2..];

    let mut child = Command::new("unbuffer")
        .arg(command)
        .args(command_args)
        .stdout(Stdio::piped())
        .spawn()?;

    let stdout = child.stdout.take().expect("Failed to capture stdout");
    let reader = BufReader::new(stdout);

    let mut last_sent_timestamp = current_timestamp_nanos();
    let mut last_clear_timestamp = current_timestamp_nanos();

    let target_fps = 15;

    let clear_every_nanos = 30 * 1_000_000_000u128;
    let nanos_per_frame = 1_000_000_000u128 / target_fps as u128;

    println!("Nanos per frame: {}", nanos_per_frame);

    println!("Starting read");
    for line in reader.lines() {
        let line = line?.to_lowercase();
        println!("{}", line);
        let split: Vec<&str> = line.split_ascii_whitespace().collect();

        if split.len() < 4 || split[0] != "waggle" {
            continue;
        }

        if split[1] == "graph" {
            let graph_name = split[2];
            if let Ok(value) = split[3].parse::<f64>() {
                let graph_data = GraphDataPoint {
                    x: current_timestamp_nanos() as f64,
                    y: value,
                    settings: GraphDataSettings { clear_data: false },
                };

                robot_data
                    .graph_data
                    .entry(graph_name.to_string())
                    .or_insert_with(|| Vec::new())
                    .push(graph_data);

                if (current_timestamp_nanos() - last_sent_timestamp) >= nanos_per_frame {
                    last_sent_timestamp = current_timestamp_nanos();

                    send_data(&client, robot_data).await;

                    robot_data = RobotData::default();
                }

                if (current_timestamp_nanos() - last_clear_timestamp) >= clear_every_nanos {
                    last_clear_timestamp = current_timestamp_nanos();
                    clearscreen::clear().expect("failed to clear screen");
                }
            }
        }
    }

    child.wait()?;

    Ok(())
}
