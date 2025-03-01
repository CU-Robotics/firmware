use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::env;
use std::io::{BufRead, BufReader};
use std::process::{Command, Stdio};
use std::time::{SystemTime, UNIX_EPOCH};

#[derive(Serialize, Deserialize, Debug)]
struct ImageData {
    image_data: String,
    scale: i32,
    flip: bool,
}

#[derive(Serialize, Deserialize, Debug)]
struct GraphDataSettings {
    clear_data: bool,
}

#[derive(Serialize, Deserialize, Debug)]
struct GraphDataPoint {
    x: f64,
    y: f64,
    settings: GraphDataSettings,
}

#[derive(Serialize, Deserialize, Debug)]
struct RobotPosition {
    x: f64,
    y: f64,
    heading: f64,
}

#[derive(Serialize, Deserialize, Debug)]
struct StringData {
    value: String,
}

#[derive(Serialize, Deserialize, Debug)]
struct RobotData {
    sent_timestamp: f64,
    images: HashMap<String, ImageData>,
    graph_data: HashMap<String, Vec<GraphDataPoint>>,
    string_data: HashMap<String, StringData>,
    robot_position: RobotPosition,
}

impl Default for RobotData {
    fn default() -> Self {
        RobotData {
            sent_timestamp: 0.0,
            images: HashMap::new(),
            graph_data: HashMap::new(),
            string_data: HashMap::new(),
            robot_position: RobotPosition {
                x: 0.0,
                y: 0.0,
                heading: 0.0,
            },
        }
    }
}

async fn send_data(robot_data: RobotData) {
    let client = reqwest::Client::new();

    let json_str = serde_json::to_string_pretty(&robot_data).unwrap();

    match client
        .post("http://localhost:3000/batch")
        .header("Content-Type", "application/json")
        .body(json_str.clone())
        .send()
        .await
    {
        Ok(_response) => {}
        Err(e) => {
            println!("Error sending request: {}", e);
        }
    };
}

pub fn current_timestamp_nanos() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("Time went backwards")
        .as_nanos() as u128
}

#[tokio::main]
async fn main() -> std::io::Result<()> {
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

    let mut robot_data = RobotData::default();

    let mut last_sent_timestamp = current_timestamp_nanos();
    let target_fps = 15;

    for line in reader.lines() {
        let line = line?.to_lowercase();
        println!("intercepted {}", line);
        let split: Vec<&str> = line.split_ascii_whitespace().collect();
        if split.len() < 4 {
            continue;
        }
        if split[0] != "waggle" {
            continue;
        }
        if split[1] == "graph" {
            let graph_name = split[2];
            // println!("parsing {}", split[3]);
            let value: Result<f64, _> = split[3].parse();
            if let Ok(value) = value {
                // println!("Graph {}: {}", graph_name, value);
                let graph_data: GraphDataPoint = GraphDataPoint {
                    x: (current_timestamp_nanos() as f64),
                    y: value,
                    settings: GraphDataSettings { clear_data: false },
                };
                robot_data
                    .graph_data
                    .entry(graph_name.to_string())
                    .or_insert_with(Vec::new)
                    .push(graph_data);

                if (current_timestamp_nanos() - last_sent_timestamp) / (10e9 as u128)
                    > 1 / target_fps
                {
                    last_sent_timestamp = current_timestamp_nanos();
                    send_data(robot_data).await;
                    robot_data = RobotData::default();
                }
            } else {
                println!("Invalid graph value: {}", split[3]);
            }
            continue;
        }
    }

    child.wait()?;
    send_data(robot_data).await;

    Ok(())
}
