use image::current_timestamp_nanos;
use rand::Rng;
use rand::distributions::Alphanumeric;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::env;
use std::io::{BufRead, BufReader};
use std::process::{Command, Stdio};
use std::time::Duration;
use std::time::{SystemTime, UNIX_EPOCH};
use tokio::time::sleep;

mod image;

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

fn random_string(len: usize) -> String {
    rand::thread_rng()
        .sample_iter(&Alphanumeric)
        .take(len)
        .map(char::from)
        .collect()
}

fn random_graph_data_point() -> GraphDataPoint {
    let mut rng = rand::thread_rng();
    let now = SystemTime::now();
    let duration = now.duration_since(UNIX_EPOCH);
    let millis = duration.unwrap().as_millis();
    GraphDataPoint {
        x: millis as f64,
        y: rng.r#gen::<f64>(),
        settings: GraphDataSettings { clear_data: false },
    }
}

fn random_graph_data_points() -> Vec<GraphDataPoint> {
    let mut rng = rand::thread_rng();
    let count = rng.r#gen_range(1..=5);
    (0..count).map(|_| random_graph_data_point()).collect()
}

fn random_robot_position() -> RobotPosition {
    let mut rng = rand::thread_rng();
    let now = SystemTime::now();
    let duration = now.duration_since(UNIX_EPOCH);
    let millis = duration.unwrap().as_millis();
    RobotPosition {
        x: millis as f64,
        y: rng.r#gen::<f64>(),
        heading: rng.r#gen::<f64>(),
    }
}

fn create_json_string() -> RobotData {
    let mut rng = rand::thread_rng();

    let mut graph_data: HashMap<String, Vec<GraphDataPoint>> = HashMap::new();
    for i in 0..2 {
        let key = format!("graph_key_{}", i);
        graph_data.insert(key, random_graph_data_points());
    }

    let mut string_data: HashMap<String, StringData> = HashMap::new();
    for i in 0..2 {
        let key = format!("string_key_{}", i);
        string_data.insert(
            key,
            StringData {
                value: random_string(10),
            },
        );
    }

    let mut images: HashMap<String, ImageData> = HashMap::new();
    let image = image::generate_timestamp_image(200, 200);
    for i in 0..20 {
        let key = format!("image_key_{}", i);
        images.insert(
            key,
            ImageData {
                image_data: image.clone(),
                scale: rng.r#gen_range(1..=5),
                flip: rng.r#gen::<bool>(),
            },
        );
    }

    let robot_data = RobotData {
        sent_timestamp: rng.r#gen::<f64>(),
        images,
        graph_data,
        string_data,
        robot_position: random_robot_position(),
    };

    robot_data
}

async fn send_random_loop() {
    loop {
        let start = SystemTime::now();
        let robot_data = create_json_string();
        println!("Sending");

        send_data(robot_data).await;

        let elapsed = start.elapsed().unwrap();
        let remaining = Duration::from_secs(1) / 30 - elapsed;
        if remaining.as_millis() > 0 {
            sleep(remaining).await;
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
        Ok(response) => {
            println!("Sent successfully: Status {}", response.status());
        }
        Err(e) => {
            println!("Error sending request: {}", e);
        }
    };
}

#[tokio::main]
async fn main() -> std::io::Result<()> {
    // main_loop().await;
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
    let mut data_counter = 0;

    for line in reader.lines() {
        let line = line?.to_lowercase();
        println!("reeeee2 {}", line);
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
                data_counter += 1;
                if data_counter > 500 {
                    send_data(robot_data).await;
                    robot_data = RobotData::default();
                    data_counter = 0;
                }
            } else {
                // println!("Invalid graph value: {}", split[4]);
            }
            continue;
        }
    }

    child.wait()?;
    send_data(robot_data).await;

    Ok(())
}
