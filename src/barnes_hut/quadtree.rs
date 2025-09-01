use crate::physics::body::Body;
use nalgebra::{Point2, Vector2};

#[derive(Debug, Clone)]
pub enum QuadNode {
    Leaf {
        body_indices: Vec<usize>,
    },
    Internal {
        children: [Box<QuadNode>; 4],
        center_of_mass: Point2<f64>,
        total_mass: f64,
    },
}

impl QuadNode {
    pub fn new_leaf() -> Self {
        Self::Leaf {
            body_indices: Vec::new(),
        }
    }
    pub fn is_leaf(&self) -> bool {
        matches!(self, QuadNode::Leaf { .. })
    }
}

pub struct Quadtree {
    root: QuadNode,
    bounds: (f64, f64, f64, f64),
    max_bodies_per_leaf: usize,
}

impl Quadtree {
    pub fn new(bounds: (f64, f64, f64, f64), max_bodies_per_leaf: usize) -> Self {
        Self {
            root: QuadNode::new_leaf(),
            bounds,
            max_bodies_per_leaf,
        }
    }
    
    pub fn insert_body(&mut self, body_index: usize, bodies: &[Body]) {
        let bounds = self.bounds;
        let max_bodies = self.max_bodies_per_leaf;
        Self::insert_body_recursive(&mut self.root, body_index, bounds, bodies, max_bodies);
    }
    
    fn insert_body_recursive(
        node: &mut QuadNode,
        body_index: usize,
        node_bounds: (f64, f64, f64, f64),
        bodies: &[Body],
        max_bodies_per_leaf: usize
    ) {
        match node {
            QuadNode::Leaf { body_indices } => {
                body_indices.push(body_index);
                if body_indices.len() > max_bodies_per_leaf && body_indices.len() > 1 {
                    Self::subdivide_node(node, node_bounds, bodies, max_bodies_per_leaf);
                }
            }
            QuadNode::Internal { children, .. } => {
                let quadrant = Self::get_quadrant(bodies[body_index].pos(), node_bounds);
                let child_bounds = Self::get_child_bounds(node_bounds, quadrant);
                Self::insert_body_recursive(&mut children[quadrant], body_index, child_bounds, bodies, max_bodies_per_leaf);
            }
        }
    }

    fn subdivide_node(node: &mut QuadNode, node_bounds: (f64, f64, f64, f64), bodies: &[Body], max_bodies_per_leaf: usize) {
        let (min_x, max_x, min_y, max_y) = node_bounds;
        let width = max_x - min_x;
        let height = max_y - min_y;
        
        // Prevent infinite subdivision
        if width < 1.0 || height < 1.0 {
            return;
        }
        
        if let QuadNode::Leaf { body_indices } = node {
            // Don't subdivide if all bodies are in the same location
            if body_indices.len() <= 1 {
                return;
            }
            
            let old_indices = body_indices.clone();
            *node = QuadNode::Internal {
                children: [
                    Box::new(QuadNode::new_leaf()),
                    Box::new(QuadNode::new_leaf()),
                    Box::new(QuadNode::new_leaf()),
                    Box::new(QuadNode::new_leaf()),
                ],
                center_of_mass: Point2::new(0.0, 0.0),
                total_mass: 0.0,
            };

            if let QuadNode::Internal { children, .. } = node {
                for body_index in old_indices {
                    let quadrant = Self::get_quadrant(bodies[body_index].pos(), node_bounds);
                    let child_bounds = Self::get_child_bounds(node_bounds, quadrant);
                    Self::insert_body_recursive(&mut children[quadrant], body_index, child_bounds, bodies, max_bodies_per_leaf);
                }
            }
        }
    }

    fn get_quadrant(pos: Point2<f64>, bounds: (f64, f64, f64, f64)) -> usize {
        let (min_x, max_x, min_y, max_y) = bounds;
        let mid_x = (min_x + max_x) / 2.0;
        let mid_y = (min_y + max_y) / 2.0;

        match (pos.x >= mid_x, pos.y >= mid_y) {
            (false, false) => 2,
            (true, false) => 3,
            (false, true) => 0,
            (true, true) => 1,
        }
    }

    fn get_child_bounds(
        parent_bounds: (f64, f64, f64, f64),
        quadrant: usize,
    ) -> (f64, f64, f64, f64) {
        let (min_x, max_x, min_y, max_y) = parent_bounds;
        let mid_x = (min_x + max_x) / 2.0;
        let mid_y = (min_y + max_y) / 2.0;

        match quadrant {
            0 => (min_x, mid_x, mid_y, max_y),
            1 => (mid_x, max_x, mid_y, max_y),
            2 => (min_x, mid_x, min_y, mid_y),
            3 => (mid_x, max_x, min_y, mid_y),
            _ => panic!("Invalid quadrant"),
        }
    }

    pub fn build_from_bodies(&mut self, bodies: &[Body]) {
        self.root = QuadNode::new_leaf();
        for i in 0..bodies.len() {
            self.insert_body(i, bodies);
        }
        Self::calculate_center_of_mass(&mut self.root, bodies);
    }

    fn calculate_center_of_mass(node: &mut QuadNode, bodies: &[Body]) {
        match node {
            QuadNode::Leaf { body_indices: _ } => {
                
            }
            QuadNode::Internal { children, center_of_mass, total_mass } => {
                let mut total = 0.0;
                let mut weighted_pos = Vector2::new(0.0, 0.0);
                
                for child in children.iter_mut() {
                    Self::calculate_center_of_mass(child, bodies);
                    match child.as_ref() {
                        QuadNode::Internal { center_of_mass: child_com, total_mass: child_mass, .. } => {
                            if *child_mass > 0.0 {
                                total += child_mass;
                                weighted_pos += child_com.coords * *child_mass;
                            }
                        }
                        QuadNode::Leaf { body_indices } => {
                            for &index in body_indices {
                                let body = &bodies[index];
                                total += body.mass();
                                weighted_pos += body.pos().coords * body.mass();
                            }
                        }
                    }
                }
                
                if total > 0.0 {
                    *center_of_mass = Point2::from(weighted_pos / total);
                    *total_mass = total;
                }
            }
        }
    }

    pub fn draw_tree(&self, node: &QuadNode, bounds: (f64, f64, f64, f64)) {
        use macroquad::prelude::*;
        
        let (min_x, max_x, min_y, max_y) = bounds;
        
        draw_rectangle_lines(
            min_x as f32, 
            min_y as f32, 
            (max_x - min_x) as f32, 
            (max_y - min_y) as f32, 
            1.0, 
            GREEN
        );
        
        if let QuadNode::Internal { children, .. } = node {
            for (i, child) in children.iter().enumerate() {
                let child_bounds = Self::get_child_bounds(bounds, i);
                Self::draw_tree_recursive(child, child_bounds);
            }
        }
    }

    fn draw_tree_recursive(node: &QuadNode, bounds: (f64, f64, f64, f64)) {
        use macroquad::prelude::*;
        
        let (min_x, max_x, min_y, max_y) = bounds;
        
        draw_rectangle_lines(
            min_x as f32, 
            min_y as f32, 
            (max_x - min_x) as f32, 
            (max_y - min_y) as f32, 
            1.0, 
            GREEN
        );
        
        if let QuadNode::Internal { children, .. } = node {
            for (i, child) in children.iter().enumerate() {
                let child_bounds = Self::get_child_bounds(bounds, i);
                Self::draw_tree_recursive(child, child_bounds);
            }
        }
    }

    pub fn draw(&self) {
        self.draw_tree(&self.root, self.bounds);
    }

    pub fn root(&self) -> &QuadNode {
        &self.root
    }

    pub fn bounds(&self) -> (f64, f64, f64, f64) {
        self.bounds
    }
}