use base64::Engine;
use base64::engine::general_purpose::STANDARD as BASE64;
use image::{ImageBuffer, Rgb};
use std::io::Cursor;
use std::time::{SystemTime, UNIX_EPOCH};

fn draw_ball(
    img: &mut ImageBuffer<Rgb<u8>, Vec<u8>>,
    x_min: u32,
    x_max: u32,
    y_min: u32,
    y_max: u32,
    ball_x: f64,
    ball_y: f64,
    ball_radius: f64,
) {
    let r2 = ball_radius * ball_radius;
    for y in y_min..y_max {
        let dy = y as f64 + 0.5 - ball_y;
        let dy2 = dy * dy;
        // If the entire row is outside the ball vertically, skip it.
        if dy2 > r2 {
            continue;
        }
        // For a given y, the horizontal distance from the center of the ball
        // that is still within the circle is:
        let dx = (r2 - dy2).sqrt();
        // Compute the starting and ending x coordinates (accounting for pixel centers).
        let x0 = ((ball_x - dx) - 0.5).ceil() as i32;
        let x1 = ((ball_x + dx) - 0.5).floor() as i32;
        // Clamp the computed x-range to the drawing region.
        let x0 = x0.max(x_min as i32) as u32;
        let x1 = x1.min((x_max - 1) as i32) as u32;
        // Draw a horizontal line for the current scanline.
        for x in x0..=x1 {
            img.put_pixel(x, y, Rgb([255, 0, 0]));
        }
    }
}

/// Generates an image with a bouncing ball.
/// The ball's position is computed from the current timestamp.
/// The image is filled with a white background, and the ball is drawn in red.
pub fn generate_timestamp_image(width: u32, height: u32) -> String {
    let timestamp_ms = current_timestamp_nanos() as f64;
    let seconds = timestamp_ms / 1000.0;

    // Ball parameters: radius is 10% of the smallest dimension.
    let ball_radius = (width.min(height) as f64) * 0.1;
    let freq_x = 1.0;
    let freq_y = 1.3;

    // Compute ball position using sine/cosine for smooth bouncing
    let ball_x =
        ball_radius + ((width as f64 - 2.0 * ball_radius) * (0.5 + 0.5 * (seconds * freq_x).sin()));
    let ball_y = ball_radius
        + ((height as f64 - 2.0 * ball_radius) * (0.5 + 0.5 * (seconds * freq_y).cos()));

    // Create an image filled with a white background.
    let mut img: ImageBuffer<Rgb<u8>, Vec<u8>> =
        ImageBuffer::from_pixel(width, height, Rgb([255, 255, 255]));
    // Determine the bounding box for the ball
    let x_min = (ball_x - ball_radius).max(0.0) as u32;
    let y_min = (ball_y - ball_radius).max(0.0) as u32;
    let x_max = (ball_x + ball_radius).min(width as f64) as u32;
    let y_max = (ball_y + ball_radius).min(height as f64) as u32;

    // Draw the ball: iterate only over the bounding box.
    // for y in y_min..y_max {
    //     for x in x_min..x_max {
    //         let dx = x as f64 + 0.5 - ball_x;
    //         let dy = y as f64 + 0.5 - ball_y;
    //         if dx * dx + dy * dy <= ball_radius * ball_radius {
    //             img.put_pixel(x, y, Rgb([255, 0, 0])); // red ball
    //         }
    //     }
    // }
    draw_ball(
        &mut img,
        x_min,
        x_max,
        y_min,
        y_max,
        ball_x,
        ball_y,
        ball_radius,
    );

    let mut bytes: Vec<u8> = Vec::new();
    img.write_to(&mut Cursor::new(&mut bytes), image::ImageFormat::Png)
        .expect("Failed to write image to buffer");

    // Use the modern base64 encoding approach
    BASE64.encode(bytes)
}

/// Helper function to get the current timestamp in milliseconds.
pub fn current_timestamp_nanos() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("Time went backwards")
        .as_nanos() as u64
}
