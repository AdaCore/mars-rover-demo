//! TRON-style persistent light ribbon trail.
//!
//! Press `L` to toggle.  While enabled, the rover leaves a glowing cyan-blue
//! ribbon that hugs the terrain.  The trail persists when the ribbon is
//! toggled off; pressing `L` again starts a fresh trail.
//!
//! If waypoints are loaded, the ribbon activates automatically when the rover
//! first captures waypoint 0 (within 0.1 m) and deactivates when the last
//! waypoint is captured.

use std::collections::VecDeque;

use bevy::{
    prelude::*,
    render::{
        mesh::Indices,
        render_asset::RenderAssetUsages,
        render_resource::PrimitiveTopology,
        view::NoFrustumCulling,
    },
};

use crate::resource::{RobotRes, TerrainRes, WaypointsRes};

// ── Public event ──────────────────────────────────────────────────────────────

/// Sent by the keyboard handler when the rover is reset (R key).
/// `ribbon::RibbonPlugin` listens for this to clear the trail.
#[derive(Event)]
pub struct RoverResetEvent;

// ── Constants ─────────────────────────────────────────────────────────────────

/// Thickness of the ribbon in metres (XZ depth of the slab).
/// Small but non-zero to prevent degenerate-mesh rendering artefacts.
const RIBBON_THICKNESS: f32 = 0.015;

/// Height above the sampled terrain surface for the bottom edge.
const TERRAIN_OFFSET: f32 = 0.02;

/// Minimum distance the rover must travel before a new point is sampled.
const MIN_SAMPLE_DIST: f32 = 0.05;

/// Distance (metres) within which a waypoint is considered captured.
const WAYPOINT_CAPTURE_DIST: f64 = 0.1;

/// Maximum number of sampled points kept in the trail.
const MAX_POINTS: usize = 5000;

// ── Resource ──────────────────────────────────────────────────────────────────

#[derive(Resource)]
pub struct RibbonTrail {
    pub enabled: bool,
    points: VecDeque<Vec3>,
    dirty: bool,
    /// Ribbon wall height in metres; set from robot config at startup.
    ribbon_height: f32,
    mesh_handle: Option<Handle<Mesh>>,
    entity: Option<Entity>,
    /// Set when waypoint 0 has been captured; gates auto-activate logic.
    first_wp_captured: bool,
    /// Set when the last waypoint has been captured; gates auto-deactivate logic.
    last_wp_captured: bool,
}

impl Default for RibbonTrail {
    fn default() -> Self {
        Self {
            enabled: false,
            points: VecDeque::with_capacity(MAX_POINTS),
            dirty: false,
            ribbon_height: 0.063, // fallback; overwritten by setup_ribbon
            mesh_handle: None,
            entity: None,
            first_wp_captured: false,
            last_wp_captured: false,
        }
    }
}

// ── Plugin ────────────────────────────────────────────────────────────────────

pub struct RibbonPlugin;

impl Plugin for RibbonPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<RoverResetEvent>()
            .init_resource::<RibbonTrail>()
            .add_systems(Startup, setup_ribbon)
            .add_systems(
                Update,
                (
                    handle_ribbon_keyboard,
                    check_waypoint_capture,
                    sample_rover_position,
                    rebuild_ribbon_mesh,
                    handle_rover_reset,
                )
                    .chain(),
            );
    }
}

// ── Systems ───────────────────────────────────────────────────────────────────

fn setup_ribbon(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut ribbon: ResMut<RibbonTrail>,
    robot: Res<RobotRes>,
) {
    ribbon.ribbon_height =
        (robot.config().ground_clearance + robot.config().body_height) as f32;

    // Empty mesh — kept in MAIN_WORLD so we can mutate it each frame.
    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::MAIN_WORLD | RenderAssetUsages::RENDER_WORLD,
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, Vec::<[f32; 3]>::new());
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, Vec::<[f32; 3]>::new());
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, Vec::<[f32; 2]>::new());
    mesh.insert_indices(Indices::U32(vec![]));

    let mesh_handle = meshes.add(mesh);

    // TRON cyan-blue: high linear-RGB values so bloom produces a strong glow.
    let material = materials.add(StandardMaterial {
        base_color: Color::rgba(0.0, 0.0, 0.0, 1.0),
        emissive: Color::rgb_linear(0.0, 80_000.0, 200_000.0),
        double_sided: true,
        cull_mode: None,
        ..default()
    });

    let entity = commands
        .spawn((
            PbrBundle {
                mesh: mesh_handle.clone(),
                material,
                visibility: Visibility::Hidden,
                ..default()
            },
            // The ribbon mesh is mutated in-place each frame, so Bevy never
            // recomputes its AABB.  Skip frustum culling to prevent the ribbon
            // from being incorrectly culled after a pan/zoom.
            NoFrustumCulling,
        ))
        .id();

    ribbon.mesh_handle = Some(mesh_handle);
    ribbon.entity = Some(entity);
}

fn handle_ribbon_keyboard(
    keys: Res<ButtonInput<KeyCode>>,
    mut ribbon: ResMut<RibbonTrail>,
) {
    if keys.just_pressed(KeyCode::KeyL) {
        ribbon.enabled = !ribbon.enabled;
        if ribbon.enabled {
            // Starting a fresh trail — clear any persisted points and reset
            // waypoint capture state so auto-activate can fire again.
            ribbon.points.clear();
            ribbon.dirty = true;
            ribbon.first_wp_captured = false;
            ribbon.last_wp_captured = false;
        }
        // When disabling, leave points and visibility intact so the trail persists.
    }
}

fn check_waypoint_capture(
    robot: Res<RobotRes>,
    waypoints: Res<WaypointsRes>,
    mut ribbon: ResMut<RibbonTrail>,
) {
    if waypoints.0.is_empty() {
        return;
    }

    let pos = robot.position();
    let rx = pos.x();
    let ry = pos.y();

    const CAP_SQ: f64 = WAYPOINT_CAPTURE_DIST * WAYPOINT_CAPTURE_DIST;

    // Auto-activate when the rover reaches the first waypoint.
    if !ribbon.first_wp_captured {
        let (wx, wy) = waypoints.0[0];
        let dx = rx - wx;
        let dy = ry - wy;
        if dx * dx + dy * dy <= CAP_SQ {
            ribbon.first_wp_captured = true;
            ribbon.enabled = true;
            ribbon.points.clear();
            ribbon.dirty = true;
        }
        // Don't check the last waypoint until the first has been captured.
        return;
    }

    // Auto-deactivate when the rover reaches the last waypoint.
    if !ribbon.last_wp_captured {
        let (wx, wy) = *waypoints.0.last().unwrap();
        let dx = rx - wx;
        let dy = ry - wy;
        if dx * dx + dy * dy <= CAP_SQ {
            ribbon.last_wp_captured = true;
            ribbon.enabled = false;
            // Leave points intact — trail persists after deactivation.
        }
    }
}

fn sample_rover_position(
    robot: Res<RobotRes>,
    terrain: Res<TerrainRes>,
    mut ribbon: ResMut<RibbonTrail>,
) {
    if !ribbon.enabled {
        return;
    }

    let pos = robot.position();
    let x = pos.x() as f32;
    let z = -pos.y() as f32;
    let y = terrain.height_at(pos.x(), pos.y()) as f32 + TERRAIN_OFFSET;
    let candidate = Vec3::new(x, y, z);

    let too_close = ribbon
        .points
        .back()
        .map(|last| last.distance(candidate) < MIN_SAMPLE_DIST)
        .unwrap_or(false);

    if too_close {
        return;
    }

    if ribbon.points.len() >= MAX_POINTS {
        ribbon.points.pop_front();
    }
    ribbon.points.push_back(candidate);
    ribbon.dirty = true;
}

fn rebuild_ribbon_mesh(
    mut ribbon: ResMut<RibbonTrail>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut visibility_query: Query<&mut Visibility>,
) {
    if !ribbon.dirty {
        return;
    }
    ribbon.dirty = false;

    let Some(entity) = ribbon.entity else { return };
    let Some(handle) = ribbon.mesh_handle.clone() else { return };

    if ribbon.points.len() < 2 {
        if let Ok(mut vis) = visibility_query.get_mut(entity) {
            *vis = Visibility::Hidden;
        }
        if let Some(mesh) = meshes.get_mut(&handle) {
            mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, Vec::<[f32; 3]>::new());
            mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, Vec::<[f32; 3]>::new());
            mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, Vec::<[f32; 2]>::new());
            mesh.insert_indices(Indices::U32(vec![]));
        }
        return;
    }

    let height = ribbon.ribbon_height;
    let (positions, normals, uvs, indices) = build_ribbon_geometry(&ribbon.points, height);

    if let Some(mesh) = meshes.get_mut(&handle) {
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
        mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
        mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
        mesh.insert_indices(Indices::U32(indices));
    }

    if let Ok(mut vis) = visibility_query.get_mut(entity) {
        *vis = Visibility::Visible;
    }
}

fn handle_rover_reset(
    mut events: EventReader<RoverResetEvent>,
    mut ribbon: ResMut<RibbonTrail>,
) {
    for _ in events.read() {
        ribbon.points.clear();
        ribbon.dirty = true;
        ribbon.enabled = false;
        ribbon.first_wp_captured = false;
        ribbon.last_wp_captured = false;
    }
}

// ── Geometry builder ──────────────────────────────────────────────────────────

/// Build a thin vertical slab ribbon.
///
/// Each cross-section has 4 vertices arranged as a rectangle in the plane
/// perpendicular to the path:
///
/// ```text
///   TL(4i+1) ──── TR(4i+3)   ← top edge
///      |               |
///   BL(4i)   ──── BR(4i+2)   ← bottom edge (terrain level)
/// ```
///
/// Three faces are emitted per segment: left side, right side, and top cap.
/// With `double_sided: true` these are visible from all angles.
fn build_ribbon_geometry(
    points: &VecDeque<Vec3>,
    ribbon_height: f32,
) -> (Vec<[f32; 3]>, Vec<[f32; 3]>, Vec<[f32; 2]>, Vec<u32>) {
    let n = points.len();
    let half_t = RIBBON_THICKNESS / 2.0;

    // 4 verts per cross-section; 3 quads (6 tris, 18 indices) per segment.
    let mut positions = Vec::with_capacity(4 * n);
    let mut normals   = Vec::with_capacity(4 * n);
    let mut uvs       = Vec::with_capacity(4 * n);
    let mut indices   = Vec::with_capacity((n - 1) * 18);

    for i in 0..n {
        // Central-difference tangent projected onto XZ.
        let raw = match i {
            0              => points[1] - points[0],
            _ if i == n-1  => points[n-1] - points[n-2],
            _              => points[i+1] - points[i-1],
        };
        let xz = Vec3::new(raw.x, 0.0, raw.z);
        let tangent = if xz.length_squared() > 1e-10 {
            xz.normalize()
        } else if i > 0 {
            let prev = points[i] - points[i-1];
            let pxz = Vec3::new(prev.x, 0.0, prev.z);
            if pxz.length_squared() > 1e-10 { pxz.normalize() } else { Vec3::X }
        } else {
            Vec3::X
        };

        // Perpendicular in XZ (90° CCW).
        let perp = Vec3::new(-tangent.z, 0.0, tangent.x);

        let bl = points[i]             - perp * half_t;
        let br = points[i]             + perp * half_t;
        let tl = points[i] + Vec3::Y * ribbon_height - perp * half_t;
        let tr = points[i] + Vec3::Y * ribbon_height + perp * half_t;

        let v = i as f32 / (n - 1) as f32;

        // BL  (index 4i)
        positions.push(bl.to_array());
        normals.push((-perp).to_array());
        uvs.push([0.0, v]);
        // TL  (index 4i+1)
        positions.push(tl.to_array());
        normals.push((-perp).to_array());
        uvs.push([0.0, v]);
        // BR  (index 4i+2)
        positions.push(br.to_array());
        normals.push(perp.to_array());
        uvs.push([1.0, v]);
        // TR  (index 4i+3)
        positions.push(tr.to_array());
        normals.push(perp.to_array());
        uvs.push([1.0, v]);
    }

    for i in 0..(n as u32 - 1) {
        let bl0 = 4 * i;
        let tl0 = 4 * i + 1;
        let br0 = 4 * i + 2;
        let tr0 = 4 * i + 3;
        let bl1 = 4 * (i + 1);
        let tl1 = 4 * (i + 1) + 1;
        let br1 = 4 * (i + 1) + 2;
        let tr1 = 4 * (i + 1) + 3;

        // Left face (−perp normal, double_sided so visible from both sides).
        indices.extend_from_slice(&[bl0, bl1, tl0, tl0, bl1, tl1]);
        // Right face (+perp normal).
        indices.extend_from_slice(&[br0, tr0, br1, tr0, tr1, br1]);
        // Top cap (Y-up normal, gives top-down visibility with bloom).
        indices.extend_from_slice(&[tl0, tr0, tl1, tr0, tr1, tl1]);
    }

    (positions, normals, uvs, indices)
}
