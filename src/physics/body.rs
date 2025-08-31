use nalgebra::{Point2, Vector2};

pub struct Body {
    pos: Point2<f64>,
    prev_pos: Point2<f64>,
    vel: Vector2<f64>,
    acc: Vector2<f64>,
    mass: f64,
    first_step: bool,
}

impl Body {
    pub fn new(x: f64, y: f64, vx: f64, vy: f64, mass: f64) -> Self {
        Self {
            pos: Point2::new(x, y),
            prev_pos: Point2::new(x, y),
            vel: Vector2::new(vx, vy),
            acc: Vector2::new(0.0, 0.0),
            mass,
            first_step: true,
        }
    }

    pub fn update(&mut self, dt: f64) {
        if self.first_step {
            let new_pos = self.pos + self.vel * dt + self.acc * dt * dt * 0.5;
            self.prev_pos = self.pos;
            self.pos = new_pos.into();
            self.first_step = false;
        } else {
            let new_pos = 2.0 * self.pos - self.prev_pos + self.acc * dt * dt;
            self.prev_pos = self.pos;
            self.pos = new_pos.into();
        }
        self.acc = Vector2::new(0.0, 0.0);
    }
    pub fn apply_force(&mut self, force: Vector2<f64>) {
        self.acc += force / self.mass;
    }
    pub fn pos(&self) -> Point2<f64> {
        self.pos
    }
    pub fn x(&self) -> f64 {
        self.pos.x
    }
    pub fn y(&self) -> f64 {
        self.pos.y
    }
    pub fn mass(&self) -> f64 {
        self.mass
    }
}
