use macroquad::prelude::*;
use nalgebra::Vector2;
use nbody_simulation::physics::body::Body;

#[macroquad::main("N-Body Simulation")]
async fn main() {
    let dt = 1.0 / 60.0;
    // let g = 6.6743e-11;
    let g = 1e5;
    let eps = 1e-8;
    let mut bodies = Vec::<Body>::new();

    loop {
        clear_background(BLACK);

        // PHYSICS
        let mut forces = vec![Vector2::new(0.0, 0.0); bodies.len()];

        for i in 0..bodies.len() {
            for j in (i + 1)..bodies.len() {
                let r = bodies[j].pos() - bodies[i].pos();
                let r_squared = r.magnitude_squared();

                let force_magnitude = g * bodies[i].mass() * bodies[j].mass() / (r_squared + eps);
                let force = r.normalize() * force_magnitude;

                forces[i] += force;
                forces[j] -= force;
            }
        }
        for (body, force) in bodies.iter_mut().zip(forces.iter()) {
            body.apply_force(*force);
        }
        for body in &mut bodies {
            body.update(dt as f64);
        }

        // RENDERING
        for body in &mut bodies {
            let x = body.x() as f32;
            let y = body.y() as f32;
            let radius = 5.0;
            draw_circle(x, y, radius, WHITE);
        }

        draw_text("N-Body Simulation", 20.0, 30.0, 30.0, WHITE);
        draw_text(
            &format!(
                "Bodies: {} | FPS: {:.0}",
                bodies.len(),
                1.0 / get_frame_time()
            ),
            20.0,
            60.0,
            20.0,
            WHITE,
        );
        draw_text(
            "Click to add bodies",
            20.0,
            screen_height() - 30.0,
            20.0,
            GRAY,
        );

        if is_mouse_button_pressed(MouseButton::Left) {
            let (mx, my) = mouse_position();
            bodies.push(Body::new(mx as f64, my as f64, 0.0, 0.0, 1.0));
        }

        next_frame().await;
    }
}
