use wasm_bindgen::prelude::*;
use web_sys::{CanvasRenderingContext2d, HtmlCanvasElement, js_sys, console};

#[wasm_bindgen]
pub struct Agent {
    x: f64,
    y: f64,
    theta: f64,
    fov_angle: f64,
    fov_range: f64,
}

const SPEED: f64 = 0.8;
const TURN_SPEED: f64 = 0.025;

const RADIUS: f64 = 5.0;

const CELL_SIZE: f64 = 7.5; 
const ROOM_WIDTH_MIN: usize = 2;
const ROOM_WIDTH_MAX: usize = 4;
const ROOM_HEIGHT_MIN: usize = 2;
const ROOM_HEIGHT_MAX: usize = 4;
const NUM_ROOMS: usize = 15;
const HALL_WIDTH: usize = 3; // should be odd


const NUM_RAYS: usize = 180;

fn score_pose_against_map(agent: &Agent, slamAgent: &Agent, slam_map: &SlamMap, map: &Map) -> f64 {
    let mut score = 0.0;
    let fov = agent.fov_angle;
    let range = agent.fov_range;
    
    for i in 0..NUM_RAYS {
        let angle = agent.theta - fov / 2.0 + (i as f64) * (fov / NUM_RAYS as f64);

        for j in 0..((range) / CELL_SIZE + 1.0)  as usize{
            // what the ground truth is
            let ray_x = agent.x + j as f64 * CELL_SIZE * angle.cos();
            let ray_y = agent.y + j as f64 * CELL_SIZE * angle.sin();

            let tile_x = (ray_x / CELL_SIZE) as usize;
            let tile_y = (ray_y / CELL_SIZE) as usize;

            // what the slam agent sees
            let tile_slam_x = ((slamAgent.x + j as f64 * CELL_SIZE * angle.cos())  / CELL_SIZE) as usize;
            let tile_slam_y = ((slamAgent.y + j as f64 * CELL_SIZE * angle.sin()) / CELL_SIZE) as usize;

            if tile_slam_x >= slam_map.width || tile_slam_y >= slam_map.height 
                || tile_x >= map.width || tile_y >= map.height {
                continue;
            }

            let slam_tile = slam_map.cells[tile_slam_y][tile_slam_x];
            let observed = map.tiles[tile_y][tile_x];

            match (observed, slam_tile) {
                (Tile::Wall, SCell::Occupied) => score += 3.0,
                (Tile::Floor, SCell::Free) => score += 1.0,
                (_, SCell::Unknown) => score -= 0.2,
                _ => score -= 2.0,
            }            

            if slam_tile == SCell::Occupied {
                break;
            }
        }
    }
    score
}

// Angle function helpers
fn angular_distance(a: f64, b: f64) -> f64 {
    let diff = (a - b).abs() % (2.0 * std::f64::consts::PI);
    diff.min(2.0 * std::f64::consts::PI - diff)
}

fn wrap_angle(a: f64) -> f64 {
    let two_pi = 2.0 * std::f64::consts::PI;
    (a % two_pi + two_pi) % two_pi
}

fn shortest_angle_delta(from: f64, to: f64) -> f64 {
    let mut delta = (to - from) % (2.0 * std::f64::consts::PI);
    if delta > std::f64::consts::PI {
        delta -= 2.0 * std::f64::consts::PI;
    } else if delta < -std::f64::consts::PI {
        delta += 2.0 * std::f64::consts::PI;
    }
    delta
}


#[wasm_bindgen]
impl Agent {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Agent {
            x: 0.0, // x position
            y: 0.0, // y position
            theta: 0.0, // theta (orientation in radians)
            fov_angle: std::f64::consts::FRAC_PI_2, // fov 90 degrees
            fov_range: 4.0 * CELL_SIZE, // fov range
        }
    }

    pub fn copy_pos(&mut self, other: &Agent) {
        self.x = other.x;
        self.y = other.y;
        self.theta = other.theta;
    }

    pub fn add_noise(&mut self) {
        let noise_x = js_sys::Math::random() * 1.0 - 0.5; 
        let noise_y = js_sys::Math::random() * 1.0 - 0.5; 

        self.x += noise_x;
        self.y += noise_y;
    }

    pub fn localize(&mut self, other: &Agent, slam_map: &SlamMap, map: &Map) {
        let mut best_score = score_pose_against_map(other, &self, slam_map, map);
        let mut best_pose = (self.x, self.y);

        for dx in (-10..=10) {
            for dy in (-10..=10) {
                let try_x = self.x + dx as f64/25.0;
                let try_y = self.y + dy as f64/25.0;
                let original_x = self.x;
                let original_y = self.y;

                self.x = try_x;
                self.y = try_y;

                let score = score_pose_against_map(other, &self, slam_map, map);

                self.x = original_x;
                self.y = original_y;
                if score > best_score + 5.0 {
                    best_score = score;
                    best_pose = (try_x, try_y);
                }
            }
        }

        let blend = 0.2;
        self.x = (1.0 - blend) * self.x + blend * best_pose.0;
        self.y = (1.0 - blend) * self.y + blend * best_pose.1;
    }

    pub fn copy_theta(&mut self, other: &Agent) {
        self.theta = other.theta;
    }

    pub fn update(&mut self, 
        keys: &js_sys::Set, 
        ctx: &CanvasRenderingContext2d, 
        map: &Map, 
        add_noise: bool,
        check_walkable: bool) -> bool {
        let mut new_x = self.x;
        let mut new_y = self.y;

        let mut should_update = false;

        if keys.has(&JsValue::from_str("a")) || keys.has(&JsValue::from_str("A")) {
            self.theta -= TURN_SPEED;
            if self.theta < -std::f64::consts::PI {
                self.theta += 2.0 * std::f64::consts::PI;
            }
        }
        if keys.has(&JsValue::from_str("d")) || keys.has(&JsValue::from_str("D")) {
            self.theta += TURN_SPEED;
            if self.theta > std::f64::consts::PI {
                self.theta -= 2.0 * std::f64::consts::PI;
            }
        }
        if keys.has(&JsValue::from_str("w")) || keys.has(&JsValue::from_str("W")) {
            new_x += SPEED * self.theta.cos();
            new_y += SPEED * self.theta.sin();
            should_update = true;
        }
        if keys.has(&JsValue::from_str("s")) || keys.has(&JsValue::from_str("S")) {
            new_x -= SPEED * self.theta.cos();
            new_y -= SPEED * self.theta.sin();
            should_update = true;
        }

        if add_noise {
            let noise_x = js_sys::Math::random() * (SPEED/20.0) - (SPEED/20.0); 
            let noise_y = js_sys::Math::random() * (SPEED/20.0) - (SPEED/20.0); 

            new_x += noise_x;
            new_y += noise_y;
        }

        if check_walkable {
            if map.is_walkable(new_x - RADIUS, new_y - RADIUS) && map.is_walkable(new_x + RADIUS, new_y + RADIUS) {
                self.x = new_x;
                self.y = new_y;
            } else {
                let directions = [
                    0.0,                               // east
                    std::f64::consts::FRAC_PI_2,       // north
                    std::f64::consts::PI,              // west
                    3.0 * std::f64::consts::FRAC_PI_2, // south
                ];

                let mut best_theta = None;
                let mut best_distance = std::f64::MAX;

                for &dir in &directions {
                    let test_x = self.x + SPEED * dir.cos();
                    let test_y = self.y + SPEED * dir.sin();

                    if map.is_walkable(test_x - RADIUS, test_y - RADIUS) &&
                    map.is_walkable(test_x + RADIUS, test_y + RADIUS) {
                        let dist = angular_distance(self.theta, dir);
                        if dist < best_distance {
                            best_distance = dist;
                            best_theta = Some(dir);
                        }
                    }
                }

                if let Some(target_theta) = best_theta {
                    let delta = shortest_angle_delta(self.theta, target_theta);
                    let step = delta.clamp(-TURN_SPEED, TURN_SPEED);
                    self.theta = wrap_angle(self.theta + step);
                }

                return false;
            }
        } else {
            self.x = new_x;
            self.y = new_y;
        }
        

        let canvas = ctx.canvas().unwrap();
        let width = canvas.width() as f64;
        let height = canvas.height() as f64;

        self.x = self.x.clamp(RADIUS, width as f64);
        self.y = self.y.clamp(RADIUS, height as f64);
        return should_update;
    }

    pub fn spawn(&mut self, map: &Map) {
        if map.rooms.is_empty() {
            self.x = 0.0;
            self.y = 0.0;
            self.theta = 0.0;
        }
        let room = &map.rooms[0];
        let x = room.x as f64 * CELL_SIZE + room.width as f64 + RADIUS;
        let y = room.y as f64 * CELL_SIZE + room.height as f64 + RADIUS;

        console::log_1(&format!("Spawning agent at room center: ({}, {})", x, y).into());

        self.x = x;
        self.y = y;
        self.theta = 0.0;
    }

    pub fn draw(&self, ctx: &CanvasRenderingContext2d) {
        let canvas = ctx.canvas().unwrap();
        let width = canvas.width() as f64;
        let height = canvas.height() as f64;

        ctx.begin_path();
        ctx.arc(self.x, self.y, RADIUS, 0.0, std::f64::consts::PI * 2.0).unwrap();
        ctx.set_fill_style(&"#00f".into());
        ctx.fill();

        ctx.begin_path();
        ctx.move_to(self.x, self.y);
        ctx.arc(
            self.x,
            self.y,
            self.fov_range,
            self.theta - self.fov_angle / 2.0,
            self.theta + self.fov_angle / 2.0,
        );
        ctx.close_path();
        ctx.set_fill_style(&"rgba(255, 255, 0, 0.2)".into());
        ctx.fill();
    }
}


#[derive(Clone, Copy, PartialEq, Eq)]
enum Tile {
    Floor,
    Wall,
}

#[wasm_bindgen]
pub struct Room {
    x: usize,
    y: usize,
    width: usize,
    height: usize,
}

impl Room {
    pub fn new() -> Room {
        Room {
            x: 0,
            y: 0,
            width: 0,
            height: 0,
        }
    }

    pub fn center(&self) -> (usize, usize) {
        let center_x = self.x + self.width / 2;
        let center_y = self.y + self.height / 2;
        (center_x, center_y)
    }
}

#[wasm_bindgen]
pub struct Map {
    width: usize,
    height: usize,
    tiles: Vec<Vec<Tile>>,
    rooms: Vec<Room>,
}

#[wasm_bindgen]
impl Map {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Map {
        Map { 
            width: 0 as usize,
            height: 0 as usize,
            tiles: vec![], 
            rooms: vec![],
        }
    }

    pub fn init(&mut self, ctx: &CanvasRenderingContext2d) {
        let canvas = ctx.canvas().unwrap();
        let width = canvas.width() as f64 / CELL_SIZE;
        let height = canvas.height() as f64 / CELL_SIZE;

        let mut tiles = vec![vec![Tile::Wall; width as usize]; height as usize];

        // Carve out random rooms
        for _ in 0..NUM_ROOMS {
            let room_w = (js_sys::Math::random() * ROOM_WIDTH_MAX as f64 + ROOM_WIDTH_MIN as f64); // 3 to 8 wide
            let room_h = (js_sys::Math::random() * ROOM_HEIGHT_MAX as f64 + ROOM_HEIGHT_MIN as f64); // 3 to 8 tall
            let x = (js_sys::Math::random() * ((width - room_w) as f64)) as usize;
            let y = (js_sys::Math::random() * ((height - room_h) as f64)) as usize;

            for ry in y..y + room_h as usize{
                for rx in x..x + room_w as usize{
                    tiles[ry][rx] = Tile::Floor;
                }
            }
            self.rooms.push(Room {
                x,
                y,
                width: room_w as usize,
                height: room_h as usize,
            });
        }

        let half_w = HALL_WIDTH / 2;

        for i in 1..self.rooms.len() {
            let (x1, y1) = self.rooms[i - 1].center();
            let (x2, y2) = self.rooms[i].center();

            // horizontal corridor
            let x_start = std::cmp::min(x1, x2);
            let x_end = std::cmp::max(x1, x2);

            for x in x_start..=x_end {
                for dy in -(half_w as isize)..=(half_w as isize) {
                    let y = (y1 as isize + dy).clamp(0, (tiles.len() - 1) as isize) as usize;
                    tiles[y][x] = Tile::Floor;
                }
            }

            // vertical corridor
            let y_start = std::cmp::min(y1, y2);
            let y_end = std::cmp::max(y1, y2);

            for y in y_start..=y_end {
                for dx in -(half_w as isize)..=(half_w as isize) {
                    let x = (x2 as isize + dx).clamp(0, (tiles[0].len() - 1) as isize) as usize;
                    tiles[y][x] = Tile::Floor;
                }
            }
        }
        
        self.width = width as usize;
        self.height = height as usize;
        self.tiles = tiles;
    }

    pub fn is_walkable(&self, x: f64, y: f64) -> bool {
        let tile_x = (x / CELL_SIZE) as usize;
        let tile_y = (y / CELL_SIZE) as usize;

        if tile_x >= self.width || tile_y >= self.height {
            return false;
        }

        self.tiles[tile_y][tile_x] == Tile::Floor
    }

    pub fn draw(&self, ctx: &CanvasRenderingContext2d) {
        let canvas = ctx.canvas().unwrap();
        let width = canvas.width();
        let height = canvas.height();
        ctx.clear_rect(0.0, 0.0, width.into(), height.into());

        for y in 0..self.height {
            for x in 0..self.width {
                match self.tiles[y][x] {
                    Tile::Wall => ctx.set_fill_style(&"#333".into()),
                    Tile::Floor => ctx.set_fill_style(&"#999".into()),
                }
                ctx.fill_rect(
                    (x as f64) * CELL_SIZE,
                    (y as f64) * CELL_SIZE,
                    CELL_SIZE,
                    CELL_SIZE,
                );
            }
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum SCell {
    Unknown,
    Free,
    Occupied,
}

#[wasm_bindgen]
pub struct SlamMap {
    cells: Vec<Vec<SCell>>,
    width: usize,
    height: usize,
}

#[wasm_bindgen]
impl SlamMap {
    #[wasm_bindgen(constructor)]
    pub fn new() -> SlamMap {
        SlamMap {
            cells: vec![],
            width: 0,
            height: 0,
        }
    }

    pub fn init(&mut self, ctx: &CanvasRenderingContext2d) {
        let canvas = ctx.canvas().unwrap();
        let width = canvas.width() as f64 / CELL_SIZE;
        let height = canvas.height() as f64 / CELL_SIZE;

        self.width = width as usize;
        self.height = height as usize;
        self.cells = vec![vec![SCell::Unknown; self.width]; self.height];
    }

    pub fn update_with_scan(&mut self, agent: &Agent, slamAgent: &Agent, map: &Map) {
        // we raytrace off of the real agent, but update the slam map with the slam agent's position
        let fov = agent.fov_angle;
        let range = agent.fov_range;
        
        for i in 0..NUM_RAYS {
            let angle = agent.theta - fov / 2.0 + (i as f64) * (fov / NUM_RAYS as f64);

            for j in 0..(range / CELL_SIZE + 1.0)  as usize{
                // what the ground truth is
                let ray_x = agent.x + j as f64 * CELL_SIZE * angle.cos();
                let ray_y = agent.y + j as f64 * CELL_SIZE * angle.sin();

                let tile_x = (ray_x / CELL_SIZE) as usize;
                let tile_y = (ray_y / CELL_SIZE) as usize;

                // what the slam agent sees
                let tile_slam_x = ((slamAgent.x + j as f64 * CELL_SIZE * angle.cos()) / CELL_SIZE) as usize;
                let tile_slam_y = ((slamAgent.y + j as f64 * CELL_SIZE * angle.sin()) / CELL_SIZE) as usize;

                if tile_slam_x < self.width && tile_slam_y < self.height {
                    if map.is_walkable(ray_x, ray_y) {
                        self.cells[tile_slam_y][tile_slam_x] = SCell::Free;
                    } else {
                        self.cells[tile_slam_y][tile_slam_x] = SCell::Occupied;
                        break;
                    }
                }
            }
        }
    }

    pub fn draw(&self, ctx: &CanvasRenderingContext2d) {
        let canvas = ctx.canvas().unwrap();
        let width = canvas.width();
        let height = canvas.height();
        ctx.clear_rect(0.0, 0.0, width.into(), height.into());

        for y in 0..self.height {
            for x in 0..self.width {
                match self.cells[y][x] {
                    SCell::Unknown => ctx.set_fill_style(&"#111".into()),
                    SCell::Free => ctx.set_fill_style(&"#666".into()),
                    SCell::Occupied => ctx.set_fill_style(&"#ccc".into()),
                }
                ctx.fill_rect(
                    (x as f64) * CELL_SIZE,
                    (y as f64) * CELL_SIZE,
                    CELL_SIZE,
                    CELL_SIZE,
                );
            }
        }
    }
}