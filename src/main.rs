use macroquad::prelude::*;
use nalgebra::Vector2;
use nbody_simulation::physics::body::Body;
use nbody_simulation::barnes_hut::{Quadtree, BarnesHutForceCalculator};
use rayon::prelude::*;

#[macroquad::main("N-Body Simulation")]
async fn main() {
    let dt = 1.0 / 60.0;
    let g = 100.0;
    let eps = 5e-3;
    let mut bodies = Vec::<Body>::new();
    
    let mut use_barnes_hut = true;
    let mut show_quadtree = false;
    let force_calculator = BarnesHutForceCalculator::new(2.0, g, eps);
    
    fn generate_normal_distribution(mean: f64, std_dev: f64) -> f64 {
        let u1: f64 = rand::gen_range(0.0, 1.0);
        let u2: f64 = rand::gen_range(0.0, 1.0);
        let z0 = (-2.0_f64 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos();
        mean + std_dev * z0
    }
    
    fn initialize_galaxy(bodies: &mut Vec<Body>) {
        bodies.clear();
        
        let center_x = screen_width() as f64 / 2.0;
        let center_y = screen_height() as f64 / 2.0;
        let sigma = 120.0;
        
        bodies.push(Body::new(center_x, center_y, 0.0, 0.0, 1000.0));
        
        for _ in 0..10000 {
            let r = rand::gen_range(20.0, 200.0);
            let angle = rand::gen_range(0.0, 2.0 * std::f64::consts::PI);
            
            let x = center_x + r * angle.cos();
            let y = center_y + r * angle.sin();
            
            let orbital_speed = (1000.0 * 100.0 / r).sqrt() * 1.0;
            let vx = -orbital_speed * angle.sin();
            let vy = orbital_speed * angle.cos();
            
            bodies.push(Body::new(x, y, vx, vy, 0.1));
        }
    }

    loop {
        clear_background(BLACK);

        let mut forces = vec![Vector2::new(0.0, 0.0); bodies.len()];
        
        if use_barnes_hut && bodies.len() > 5 {
            let mut quadtree = Quadtree::new((0.0, screen_width() as f64, 0.0, screen_height() as f64), 1);
            quadtree.build_from_bodies(&bodies);
            forces = force_calculator.calculate_forces(&bodies, &quadtree);
            if show_quadtree {
                quadtree.draw();
            }
        } else {
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
        }
        for (body, force) in bodies.iter_mut().zip(forces.iter()) {
            body.apply_force(*force);
        }
        for body in &mut bodies {
            body.update(dt as f64);
        }

        // RENDERING
        for (i, body) in bodies.iter().enumerate() {
            let x = body.x() as f32;
            let y = body.y() as f32;
            let radius = if i == 0 { 8.0 } else { 1.0 };
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
            screen_height() - 60.0,
            20.0,
            GRAY,
        );
        draw_text(
            &format!(
                "[B] Barnes-Hut: {} | [Q] Show Tree: {} | [G] Generate Galaxy",
                if use_barnes_hut { "ON" } else { "OFF" },
                if show_quadtree { "ON" } else { "OFF" }
            ),
            20.0,
            screen_height() - 30.0,
            20.0,
            GRAY,
        );


        if is_mouse_button_pressed(MouseButton::Left) {
            let (mx, my) = mouse_position();
            bodies.push(Body::new(mx as f64, my as f64, 0.0, 0.0, 1.0));
        }

        if is_key_pressed(KeyCode::B) {
            use_barnes_hut = !use_barnes_hut;
        }

        if is_key_pressed(KeyCode::Q) {
            show_quadtree = !show_quadtree;
        }

        if is_key_pressed(KeyCode::G) {
            initialize_galaxy(&mut bodies);
        }

        next_frame().await;
    }
}
