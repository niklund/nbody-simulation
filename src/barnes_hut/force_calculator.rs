use nalgebra::{Point2, Vector2};
use crate::physics::body::Body;
use super::quadtree::{Quadtree, QuadNode};
use rayon::prelude::*;

pub struct BarnesHutForceCalculator {
    pub theta: f64,
    pub g: f64,
    pub eps: f64,
}

impl BarnesHutForceCalculator {
    pub fn new(theta: f64, g: f64, eps: f64) -> Self {
        Self { theta, g, eps }
    }

    pub fn calculate_forces(&self, bodies: &[Body], quadtree: &Quadtree) -> Vec<Vector2<f64>> {
        (0..bodies.len())
            .into_par_iter()
            .map(|i| self.calculate_force_on_body(i, bodies, quadtree.root(), quadtree.bounds()))
            .collect()
    }

    fn calculate_force_on_body(
        &self,
        body_index: usize,
        bodies: &[Body],
        node: &QuadNode,
        node_bounds: (f64, f64, f64, f64)
    ) -> Vector2<f64> {
        match node {
            QuadNode::Leaf { body_indices } => {
                let mut force = Vector2::new(0.0, 0.0);
                for &other_index in body_indices {
                    if other_index != body_index {
                        force += self.calculate_direct_force(body_index, other_index, bodies);
                    }
                }
                force
            }
            QuadNode::Internal { children, center_of_mass, total_mass } => {
                if *total_mass == 0.0 {
                    return Vector2::new(0.0, 0.0);
                }

                let body_pos = bodies[body_index].pos();
                let (min_x, max_x, min_y, max_y) = node_bounds;
                let node_width = (max_x - min_x).max(max_y - min_y);

                if self.should_use_approximation(body_pos, *center_of_mass, node_width) {
                    self.calculate_force_from_center_of_mass(body_index, bodies, *center_of_mass, *total_mass)
                } else {
                    let mut force = Vector2::new(0.0, 0.0);
                    for (i, child) in children.iter().enumerate() {
                        let child_bounds = Self::get_child_bounds(node_bounds, i);
                        force += self.calculate_force_on_body(body_index, bodies, child, child_bounds);
                    }
                    force
                }
            }
        }
    }

    fn should_use_approximation(
        &self,
        body_pos: Point2<f64>,
        node_center: Point2<f64>,
        node_width: f64
    ) -> bool {
        let distance = (body_pos - node_center).magnitude();
        distance > node_width / self.theta && distance > 1e-10
    }

    fn calculate_direct_force(&self, body1_index: usize, body2_index: usize, bodies: &[Body]) -> Vector2<f64> {
        let body1 = &bodies[body1_index];
        let body2 = &bodies[body2_index];
        
        let r = body2.pos() - body1.pos();
        let r_squared = r.magnitude_squared();
        
        if r_squared < self.eps * self.eps {
            return Vector2::new(0.0, 0.0);
        }
        
        let force_magnitude = self.g * body1.mass() * body2.mass() / (r_squared + self.eps);
        r.normalize() * force_magnitude
    }

    fn calculate_force_from_center_of_mass(
        &self,
        body_index: usize,
        bodies: &[Body],
        center_of_mass: Point2<f64>,
        total_mass: f64
    ) -> Vector2<f64> {
        let body = &bodies[body_index];
        let r = center_of_mass - body.pos();
        let r_squared = r.magnitude_squared();
        
        if r_squared < self.eps * self.eps {
            return Vector2::new(0.0, 0.0);
        }
        
        let force_magnitude = self.g * body.mass() * total_mass / (r_squared + self.eps);
        r.normalize() * force_magnitude
    }

    fn get_child_bounds(parent_bounds: (f64, f64, f64, f64), quadrant: usize) -> (f64, f64, f64, f64) {
        let (min_x, max_x, min_y, max_y) = parent_bounds;
        let mid_x = (min_x + max_x) / 2.0;
        let mid_y = (min_y + max_y) / 2.0;
        
        match quadrant {
            0 => (min_x, mid_x, mid_y, max_y),
            1 => (mid_x, max_x, mid_y, max_y),
            2 => (min_x, mid_x, min_y, mid_y),
            3 => (mid_x, max_x, min_y, mid_y),
            _ => panic!("Invalid quadrant: {}", quadrant),
        }
    }
}