//! 3D visualization.

use std::collections::BTreeMap;

use bevy::{
    core_pipeline::{bloom::BloomSettings, tonemapping::Tonemapping},
    input::mouse::{MouseMotion, MouseScrollUnit, MouseWheel},
    pbr::NotShadowCaster,
    prelude::*,
};
use rand::{
    distr::{Distribution, Uniform},
    Rng, SeedableRng,
};
use rand_chacha::ChaCha8Rng;

use mars_rover_core::{
    domain::{
        Environment, HasCollision, Obstacle, Position, Robot, RobotConfig, Terrain, WheelID,
    },
    sim::SimInput,
    world::RNG_SEED,
};

use crate::{
    controller,
    replay_mode::ReplayActive,
    resource::{
        EnvironmentRes, RobotRes, ShowWaypointPath, SimInputsRes, SlipFreeMode, TerrainRes,
        WaypointsRes,
    },
};

pub struct Visualizer;

impl Plugin for Visualizer {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, set_up)
            .add_systems(
                Update,
                (
                    update_text,
                    handle_keyboard_input,
                    draw_ekf_marker,
                    draw_waypoint_path,
                    (
                        handle_relocation_of_rover,
                        handle_relocation_of_rocks,
                        handle_camera_orbit,
                        focus_camera,
                        update_rover,
                        update_rocks,
                    )
                        .chain(),
                ),
            )
            .init_resource::<Scene>()
            .init_resource::<SlipFreeMode>()
            .init_resource::<ShowWaypointPath>();
    }
}

#[derive(Resource, Default)]
pub struct Scene {
    rover: Rover,
    phantom_rover: Rover,
    phantom_rover_position: Option<Vec3>,
    rocks: Vec<Rock>,
    phantom_rock: Option<PhantomRock>,
    camera: SceneCamera,
    show_text: bool,
}

#[derive(Default)]
pub struct Rover {
    body: Option<Entity>,
    wheels: BTreeMap<WheelID, Option<Entity>>,
    distance_sensor: Option<Entity>,
    distance_sensor_light: Option<Entity>,
}

struct Rock {
    entity: Entity,
}

struct PhantomRock {
    entity: Entity,
    position: Vec3,
    obstacle: Obstacle,
    idx: usize,
}

struct SceneCamera {
    is_focus: Vec3,
    should_focus: Vec3,
    azimuth: f32,
    elevation: f32,
    radius: f32,
    orbiting: bool,
}

impl Default for SceneCamera {
    fn default() -> Self {
        Self {
            is_focus: Vec3::ZERO,
            should_focus: Vec3::ZERO,
            azimuth: INITIAL_CAMERA_AZIMUTH,
            elevation: INITIAL_CAMERA_ELEVATION,
            radius: INITIAL_CAMERA_RADIUS,
            orbiting: false,
        }
    }
}

#[derive(Component)]
struct Surface;

#[derive(Component)]
struct StatusText;

/// Build the subdivided terrain mesh.  Vertex Y is set from the terrain height;
/// Bevy uses Y-up so the ground plane is XZ.
fn build_terrain_mesh(terrain: &Terrain) -> Mesh {
    let gw = terrain.grid_width();
    let gh = terrain.grid_height();
    let cs = terrain.cell_size() as f32;
    let x_min = terrain.x_min() as f32;
    let y_min_d = terrain.y_min() as f32; // domain Y

    let n = gw * gh;
    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(n);
    let mut normals: Vec<[f32; 3]> = Vec::with_capacity(n);
    let mut uvs: Vec<[f32; 2]> = Vec::with_capacity(n);
    let mut colors: Vec<[f32; 4]> = Vec::with_capacity(n);

    // Collect heights first so we can normalise the colour gradient.
    let heights: Vec<f32> = (0..gh)
        .flat_map(|iy| {
            (0..gw).map(move |ix| {
                let wx = x_min + ix as f32 * cs;
                let wy = y_min_d + iy as f32 * cs;
                terrain.height_at(wx as f64, wy as f64) as f32
            })
        })
        .collect();
    let h_min = heights.iter().cloned().fold(f32::INFINITY, f32::min);
    let h_max = heights.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
    let h_range = (h_max - h_min).max(1e-6);

    for iy in 0..gh {
        for ix in 0..gw {
            let wx = x_min + ix as f32 * cs;
            let wy = y_min_d + iy as f32 * cs;
            let h = heights[iy * gw + ix];

            // Bevy: X = world_x, Y = height, Z = −world_y
            positions.push([wx, h, -wy]);

            // Normal from the terrain gradient:
            //   tangent along domain_x: (1, ∂h/∂x, 0)  in Bevy XYZ
            //   tangent along domain_y: (0, ∂h/∂y, −1)
            //   normal = cross(tx, ty) = (−∂h/∂x, 1, ∂h/∂y), then normalised
            let (gx, gy) = terrain.gradient_at(wx as f64, wy as f64);
            let nx = -(gx as f32);
            let ny = 1.0_f32;
            let nz = gy as f32;
            let len = (nx * nx + ny * ny + nz * nz).sqrt();
            normals.push([nx / len, ny / len, nz / len]);

            uvs.push([
                ix as f32 / (gw - 1) as f32,
                iy as f32 / (gh - 1) as f32,
            ]);

            // Mars-like colour: dark reddish-brown at low, light ochre at high.
            let t = (h - h_min) / h_range;
            colors.push([0.30 + 0.40 * t, 0.15 + 0.20 * t, 0.08 + 0.10 * t, 1.0]);
        }
    }

    let mut indices: Vec<u32> = Vec::with_capacity((gw - 1) * (gh - 1) * 6);
    for iy in 0..(gh - 1) {
        for ix in 0..(gw - 1) {
            let i00 = (iy * gw + ix) as u32;
            let i10 = (iy * gw + ix + 1) as u32;
            let i01 = ((iy + 1) * gw + ix) as u32;
            let i11 = ((iy + 1) * gw + ix + 1) as u32;
            indices.extend_from_slice(&[i00, i10, i01, i10, i11, i01]);
        }
    }

    let mut mesh = Mesh::new(
        bevy::render::render_resource::PrimitiveTopology::TriangleList,
        bevy::render::render_asset::RenderAssetUsages::RENDER_WORLD,
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, colors);
    mesh.insert_indices(bevy::render::mesh::Indices::U32(indices));
    mesh
}

fn set_up(
    mut scene: ResMut<Scene>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
    robot: Res<RobotRes>,
    environment: Res<EnvironmentRes>,
    terrain: Res<TerrainRes>,
) {
    create_surface(
        &environment,
        &terrain,
        &mut scene,
        &mut commands,
        &mut meshes,
        &mut materials,
        &asset_server,
    );

    create_sky(&mut commands, &mut meshes, &mut materials, &environment);
    create_rover(
        robot.config(),
        &mut scene.rover,
        &mut commands,
        &mut meshes,
        &mut materials,
        &asset_server,
        false,
    );
    create_light(&mut commands);
    create_camera(&mut scene, &mut commands);
    create_text(&mut commands);
}

fn create_surface(
    environment: &Environment,
    terrain: &Terrain,
    scene: &mut ResMut<Scene>,
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    asset_server: &Res<AssetServer>,
) {
    // Hidden plane used only for mouse-picking (ray-plane intersection).
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Plane3d::default().mesh()),
            material: materials.add(Color::WHITE),
            visibility: Visibility::Hidden,
            ..default()
        },
        Surface,
    ));

    // Terrain mesh — replaces the flat tile grid.
    commands.spawn(PbrBundle {
        mesh: meshes.add(build_terrain_mesh(terrain)),
        material: materials.add(StandardMaterial {
            perceptual_roughness: 1.0,
            ..default()
        }),
        ..default()
    });

    let mut rng = ChaCha8Rng::seed_from_u64(RNG_SEED);

    for obstacle in environment.obstacles() {
        scene.rocks.push(create_rock(
            obstacle,
            commands,
            meshes,
            materials,
            asset_server,
            0.3 + rng.random_range(-0.1..=0.1),
            false,
        ));
    }
}

fn create_rock(
    obstacle: &Obstacle,
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    asset_server: &Res<AssetServer>,
    height: f32,
    is_transparent: bool,
) -> Rock {
    let alpha = if is_transparent { 0.5 } else { 1.0 };
    let alpha_mode = if is_transparent {
        AlphaMode::Blend
    } else {
        AlphaMode::Opaque
    };

    Rock {
        entity: commands
            .spawn(PbrBundle {
                mesh: meshes.add(Cuboid::new(
                    obstacle.x_length() as f32,
                    height,
                    obstacle.y_length() as f32,
                )),
                material: materials.add(StandardMaterial {
                    base_color: Color::rgba(1.0, 1.0, 1.0, alpha),
                    base_color_texture: Some(asset_server.load("textures/mars_rock.png")),
                    perceptual_roughness: 1.0,
                    thickness: 1.0,
                    attenuation_distance: 0.0,
                    alpha_mode,
                    ..default()
                }),
                transform: Transform::from_xyz(
                    obstacle.position().x() as f32,
                    0.0,
                    -obstacle.position().y() as f32,
                ),
                ..default()
            })
            .id(),
    }
}


fn create_sky(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    environment: &Environment,
) {
    let initial_camera_position = Position::new(
        INITIAL_CAMERA_POSITION[0] as f64,
        -INITIAL_CAMERA_POSITION[2] as f64,
    );
    let r = Position::new(environment.x_max(), environment.y_max())
        .distance(initial_camera_position) as f32
        + 1.0;
    let t = std::f32::consts::PI * 0.35;
    let center_x = initial_camera_position.x() as f32;
    let center_z = -initial_camera_position.y() as f32;

    // Sun
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Sphere::new(0.08)),
            material: materials.add(StandardMaterial {
                emissive: Color::rgb_linear(200000.0, 180000.0, 160000.0),
                ..default()
            }),
            transform: Transform::from_xyz(center_x + r * t.cos(), 0.6, center_z + (-r) * t.sin()),
            ..default()
        },
        NotShadowCaster,
    ));

    // Stars
    let mut rng = ChaCha8Rng::seed_from_u64(RNG_SEED);
    let y_rng = Uniform::try_from(-0.7..=0.7).unwrap();
    let intensity_rng = Uniform::try_from(0.0..=1.0).unwrap();
    let pi2_rng = Uniform::try_from(0.0..=2.0 * std::f32::consts::PI).unwrap();

    let number_of_stars = 800;

    let stars = (0..number_of_stars)
        .map(|_| {
            let t = pi2_rng.sample(&mut rng);
            (
                center_x + r * t.cos(),
                y_rng.sample(&mut rng),
                center_z + r * t.sin(),
                intensity_rng.sample(&mut rng),
            )
        })
        .collect::<Vec<_>>();

    for (x, y, z, i) in stars {
        commands.spawn((
            PbrBundle {
                mesh: meshes.add(Sphere::new(0.001)),
                material: materials.add(StandardMaterial {
                    emissive: Color::rgb_linear(i * 15000.0, i * 15000.0, i * 15000.0),
                    ..default()
                }),
                transform: Transform::from_xyz(x, y, z),
                ..default()
            },
            NotShadowCaster,
        ));
    }
}

fn create_rover(
    robot_config: &RobotConfig,
    rover: &mut Rover,
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    asset_server: &Res<AssetServer>,
    is_transparent: bool,
) {
    let alpha = if is_transparent { 0.5 } else { 1.0 };
    let alpha_mode = if is_transparent {
        AlphaMode::Blend
    } else {
        AlphaMode::Opaque
    };

    rover.body = Some(commands.spawn(SpatialBundle::default()).id());

    let body_material = StandardMaterial {
        base_color: Color::rgba(1.0, 1.0, 1.0, alpha),
        base_color_texture: Some(asset_server.load("textures/rover_body.png")),
        metallic: 1.0,
        perceptual_roughness: 0.4,
        alpha_mode,
        ..default()
    };

    let body = commands
        .spawn(PbrBundle {
            mesh: meshes.add(Cuboid::new(
                (robot_config.distance_to_body_front + robot_config.distance_to_body_rear) as f32,
                robot_config.body_height as f32,
                (robot_config.distance_to_body_left + robot_config.distance_to_body_right) as f32,
            )),
            material: materials.add(body_material.clone()),
            ..default()
        })
        .id();

    commands.entity(rover.body.unwrap()).push_children(&[body]);

    for wheel_id in WheelID::iter() {
        let wheel = commands.spawn(SpatialBundle::default()).id();
        let tire = commands
            .spawn(PbrBundle {
                mesh: meshes.add(Cylinder::new(
                    robot_config.wheel_radius as f32,
                    robot_config.wheel_width as f32,
                )),
                material: materials.add(StandardMaterial {
                    base_color: Color::rgba(1.0, 1.0, 1.0, alpha),
                    base_color_texture: Some(asset_server.load("textures/rover_tire.png")),
                    metallic: 1.0,
                    perceptual_roughness: 0.8,
                    alpha_mode,
                    ..default()
                }),
                ..default()
            })
            .id();

        let rim_mesh = Cylinder::new(
            robot_config.wheel_radius as f32 * 0.9,
            robot_config.wheel_width as f32 * 0.02,
        );
        let rim_material = StandardMaterial {
            base_color: Color::rgba(1.0, 1.0, 1.0, alpha),
            base_color_texture: Some(asset_server.load("textures/rover_rim.png")),
            metallic: 1.0,
            perceptual_roughness: 0.4,
            alpha_mode,
            ..default()
        };
        let left_rim = commands
            .spawn(PbrBundle {
                mesh: meshes.add(rim_mesh),
                material: materials.add(rim_material.clone()),
                transform: Transform::from_xyz(0.0, 0.5 * robot_config.wheel_width as f32, 0.0),
                ..default()
            })
            .id();
        let right_rim = commands
            .spawn(PbrBundle {
                mesh: meshes.add(rim_mesh),
                material: materials.add(rim_material.clone()),
                transform: Transform::from_xyz(0.0, -0.5 * robot_config.wheel_width as f32, 0.0),
                ..default()
            })
            .id();

        commands
            .entity(wheel)
            .push_children(&[left_rim, right_rim, tire]);

        rover.wheels.insert(*wheel_id, Some(wheel));
    }

    rover.distance_sensor = Some(commands.spawn(SpatialBundle::default()).id());

    let distance_sensor = commands
        .spawn(PbrBundle {
            mesh: meshes.add(Cuboid::new(
                robot_config.distance_sensor_height as f32,
                robot_config.distance_sensor_depth as f32,
                robot_config.distance_sensor_width as f32,
            )),
            material: materials.add(body_material),
            ..default()
        })
        .id();

    let distance_sensor_light_area = commands
        .spawn(PbrBundle {
            mesh: meshes.add(Cylinder::new(
                0.3 * robot_config.distance_sensor_height as f32,
                0.1 * robot_config.distance_sensor_depth as f32,
            )),
            material: materials.add(StandardMaterial {
                base_color: Color::rgba(1.0, 1.0, 1.0, alpha),
                emissive: Color::rgb_linear(1000.0, 1000.0, 1000.0),
                alpha_mode,
                ..default()
            }),
            transform: Transform::from_xyz(
                0.0,
                0.5 * robot_config.distance_sensor_depth as f32,
                0.0,
            ),
            ..default()
        })
        .id();

    commands
        .entity(rover.distance_sensor.unwrap())
        .push_children(&[distance_sensor, distance_sensor_light_area]);

    if !is_transparent {
        rover.distance_sensor_light = Some(
            commands
                .spawn(SpotLightBundle {
                    spot_light: SpotLight {
                        color: Color::WHITE,
                        intensity: 10_000.0,
                        shadows_enabled: true,
                        outer_angle: std::f32::consts::PI / 12.0,
                        ..default()
                    },
                    ..default()
                })
                .id(),
        );
    }
}

fn remove_rover(rover: &mut Rover, commands: &mut Commands) {
    if let Some(entity) = rover.body {
        commands.entity(entity).despawn_recursive();
        rover.body = None;
    }

    for wheel_id in WheelID::iter() {
        if let Some(entity) = rover.wheels[wheel_id] {
            commands.entity(entity).despawn_recursive();
            rover.wheels.insert(*wheel_id, None);
        }
    }

    if let Some(entity) = rover.distance_sensor {
        commands.entity(entity).despawn_recursive();
        rover.distance_sensor = None;
    }

    if let Some(entity) = rover.distance_sensor_light {
        commands.entity(entity).despawn();
        rover.distance_sensor_light = None;
    }
}

fn create_light(commands: &mut Commands) {
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 1000.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_rotation(Quat::from_euler(
            EulerRot::ZYX,
            0.0,
            0.85 * std::f32::consts::PI,
            -std::f32::consts::FRAC_PI_8,
        )),
        ..default()
    });
}

// Arcball camera parameters. Initial values match the old fixed offset [0, 1, 2].
const INITIAL_CAMERA_POSITION: [f32; 3] = [0.0, 1.0, 2.0]; // kept for create_sky sky-dome sizing
const INITIAL_CAMERA_AZIMUTH: f32 = 0.0;
const INITIAL_CAMERA_ELEVATION: f32 = 0.4636; // arcsin(1/sqrt(5)) ≈ 26.6°
const INITIAL_CAMERA_RADIUS: f32 = 4.472; // 2*sqrt(5), scaled for doubled world
const CAMERA_ELEVATION_MIN: f32 = 0.0873; // ~5°
const CAMERA_ELEVATION_MAX: f32 = std::f32::consts::FRAC_PI_2 - 0.01; // ~89.4°, avoids looking_at singularity
const CAMERA_RADIUS_MIN: f32 = 0.5;
const CAMERA_RADIUS_MAX: f32 = 16.0;
const CAMERA_ORBIT_SENSITIVITY: f32 = 0.005; // radians per pixel
const CAMERA_ZOOM_LINE: f32 = 0.3; // radius units per scroll line
const CAMERA_ZOOM_PIXEL: f32 = 0.003; // radius units per trackpad pixel

fn spherical_to_cartesian(focus: Vec3, azimuth: f32, elevation: f32, radius: f32) -> Vec3 {
    focus
        + Vec3::new(
            radius * elevation.cos() * azimuth.sin(),
            radius * elevation.sin(),
            radius * elevation.cos() * azimuth.cos(),
        )
}

fn create_camera(scene: &mut ResMut<Scene>, commands: &mut Commands) {
    let position = spherical_to_cartesian(
        scene.camera.is_focus,
        scene.camera.azimuth,
        scene.camera.elevation,
        scene.camera.radius,
    );
    commands.spawn((
        Camera3dBundle {
            camera: Camera {
                hdr: true,
                ..default()
            },
            tonemapping: Tonemapping::TonyMcMapface,
            transform: Transform::from_translation(position)
                .looking_at(scene.camera.is_focus, Vec3::Y),
            ..default()
        },
        BloomSettings::NATURAL,
    ));
}

fn create_text(commands: &mut Commands) {
    let text_style = TextStyle {
        font_size: 20.0,
        ..default()
    };
    commands.spawn((
        TextBundle::from_sections(vec![TextSection::new("", text_style.clone())]).with_style(
            Style {
                position_type: PositionType::Absolute,
                bottom: Val::Px(12.0),
                left: Val::Px(12.0),
                ..default()
            },
        ),
        StatusText,
    ));
}

fn update_rocks(
    scene: Res<Scene>,
    mut transforms: Query<&mut Transform>,
    environment: Res<EnvironmentRes>,
) {
    for (obstacle, rock) in std::iter::zip(environment.obstacles(), scene.rocks.iter()) {
        if !obstacle.is_movable() {
            continue;
        }

        if let Ok(mut rock_transform) = transforms.get_mut(rock.entity) {
            *rock_transform = Transform {
                translation: to_bevy_position(obstacle.position()),
                ..default()
            };
        }
    }

    if let Some(phantom_rock) = &scene.phantom_rock {
        if let Ok(mut phantom_rock_transform) = transforms.get_mut(phantom_rock.entity) {
            *phantom_rock_transform = Transform {
                translation: phantom_rock.position,
                ..default()
            };
        }
    }
}

fn update_rover(
    scene: Res<Scene>,
    mut transforms: Query<&mut Transform>,
    robot: Res<RobotRes>,
    terrain: Res<TerrainRes>,
) {
    transform_rover(&scene.rover, &robot, &terrain, &mut transforms);

    if let Some(position) = scene.phantom_rover_position {
        let mut robot = robot.clone();
        robot.set_position(to_domain_position(position));
        transform_rover(&scene.phantom_rover, &robot, &terrain, &mut transforms);
    }
}

fn transform_rover(
    rover: &Rover,
    robot: &Robot,
    terrain: &Terrain,
    transforms: &mut Query<&mut Transform>,
) {
    let cfg = robot.config();
    let center = robot.position();

    // Terrain height under the rover's centre, used for body and sensor placement.
    let center_h = terrain.height_at(center.x(), center.y()) as f32;

    // Body tilt: rotate so the rover's up direction follows the terrain normal.
    //   Terrain normal in Bevy coords: N = normalise(−∂h/∂x, 1, ∂h/∂y)
    let (gx, gy) = terrain.gradient_at(center.x(), center.y());
    let terrain_normal = Vec3::new(-(gx as f32), 1.0, gy as f32).normalize();
    let tilt = Quat::from_rotation_arc(Vec3::Y, terrain_normal);

    if let Some(entity) = rover.body {
        if let Ok(mut t) = transforms.get_mut(entity) {
            *t = Transform {
                translation: Vec3::new(
                    center.x() as f32,
                    center_h + cfg.ground_clearance as f32,
                    -center.y() as f32,
                ),
                rotation: tilt * Quat::from_rotation_y(robot.heading().into()),
                ..default()
            };
        }
    }

    for wheel_id in WheelID::iter() {
        let wpos = robot.wheel_position(*wheel_id);
        let wheel_h = terrain.height_at(wpos.x(), wpos.y()) as f32;
        if let Some(entity) = rover.wheels[wheel_id] {
            if let Ok(mut t) = transforms.get_mut(entity) {
                *t = Transform {
                    translation: Vec3::new(
                        wpos.x() as f32,
                        wheel_h + cfg.wheel_radius as f32,
                        -wpos.y() as f32,
                    ),
                    rotation: Quat::from_rotation_y(
                        (robot.heading()
                            + robot.wheel_steering_angle(*wheel_id).unwrap_or_default())
                        .into(),
                    ) * Quat::from_rotation_z(-Into::<f32>::into(
                        robot.wheel_rotating_angle(*wheel_id).unwrap_or_default(),
                    )) * Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2),
                    ..default()
                };
            }
        }
    }

    let sensor_pos = robot.distance_sensor_position();
    let sensor_y = center_h
        + (cfg.ground_clearance + cfg.body_height + 0.5 * cfg.distance_sensor_height) as f32;

    if let Some(entity) = rover.distance_sensor {
        if let Ok(mut t) = transforms.get_mut(entity) {
            *t = Transform {
                translation: Vec3::new(sensor_pos.x() as f32, sensor_y, -sensor_pos.y() as f32),
                rotation: Quat::from_rotation_y(robot.distance_sensor_heading().into())
                    * Quat::from_rotation_z(-std::f32::consts::FRAC_PI_2),
                ..default()
            };
        }
    }
    if let Some(entity) = rover.distance_sensor_light {
        if let Ok(mut t) = transforms.get_mut(entity) {
            *t = Transform {
                translation: Vec3::new(sensor_pos.x() as f32, sensor_y, -sensor_pos.y() as f32),
                rotation: Quat::from_rotation_y(
                    f32::from(robot.distance_sensor_heading()) - std::f32::consts::FRAC_PI_2,
                ),
                ..default()
            };
        }
    }
}

#[allow(clippy::type_complexity)]
fn focus_camera(
    time: Res<Time>,
    mut scene: ResMut<Scene>,
    mut transforms: ParamSet<(Query<&mut Transform, With<Camera3d>>, Query<&Transform>)>,
) {
    const SPEED: f32 = 1.0;

    if let Some(rover_entity) = scene.rover.body {
        if let Ok(rover_transform) = transforms.p1().get(rover_entity) {
            scene.camera.should_focus = rover_transform.translation;
        }
    }

    // Smooth out the focus movement
    let mut focus_motion = scene.camera.should_focus - scene.camera.is_focus;
    if focus_motion.length() > 0.2 {
        focus_motion *= SPEED * time.delta_seconds();
        scene.camera.is_focus += focus_motion;
    }

    let position = spherical_to_cartesian(
        scene.camera.is_focus,
        scene.camera.azimuth,
        scene.camera.elevation,
        scene.camera.radius,
    );

    for mut transform in transforms.p0().iter_mut() {
        *transform = Transform::from_translation(position)
            .looking_at(scene.camera.is_focus, Vec3::Y);
    }
}

#[allow(clippy::too_many_arguments)]
fn handle_relocation_of_rover(
    cameras: Query<(&Camera, &GlobalTransform)>,
    surfaces: Query<&GlobalTransform, With<Surface>>,
    windows: Query<&Window>,
    mut scene: ResMut<Scene>,
    mut gizmos: Gizmos,
    transforms: Query<&Transform>,
    environment: Res<EnvironmentRes>,
    buttons: Res<ButtonInput<MouseButton>>,
    robot: Res<RobotRes>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
    sim_inputs: Res<SimInputsRes>,
    replay_active: Res<ReplayActive>,
) {
    // In replay mode the rover's position is driven from the trace; dragging
    // would push a `SimInput::MoveRover` into a queue nothing drains.
    if replay_active.0 {
        return;
    }

    let Some(pointer_position) = get_pointer_position_on_surface(&cameras, &surfaces, &windows)
    else {
        return;
    };
    let surface = surfaces.single();

    if let Some(rover_entity) = scene.rover.body {
        if let Ok(rover_transform) = transforms.get(rover_entity) {
            let mut rover_position = rover_transform.translation;
            rover_position.y = 0.0;
            let rover_distance = pointer_position.distance(rover_position);

            if scene.phantom_rover_position.is_none()
                && scene.phantom_rock.is_none()
                && !scene.camera.orbiting
                && rover_distance < 0.1
            {
                gizmos.circle(
                    rover_position + surface.up() * 0.01,
                    Direction3d::new_unchecked(surface.up()),
                    0.1,
                    Color::rgba(1.0, 1.0, 1.0, 0.2),
                );

                // Create phantom rover if rover is dragged
                if buttons.pressed(MouseButton::Left) {
                    create_rover(
                        robot.config(),
                        &mut scene.phantom_rover,
                        &mut commands,
                        &mut meshes,
                        &mut materials,
                        &asset_server,
                        true,
                    );
                    scene.phantom_rover_position = Some(pointer_position);
                }
            }
        }
    }

    let mut phantom_robot = robot.clone();
    phantom_robot.set_position(to_domain_position(pointer_position));
    let mut invalid_position = false;

    // Update position while phantom rover is dragged
    if scene.phantom_rover_position.is_some() {
        scene.phantom_rover_position = Some(pointer_position);

        invalid_position =
            environment.has_collision(&phantom_robot) || !environment.contains(&phantom_robot);

        if invalid_position {
            // Draw a circle just above the ground plane at that position.
            gizmos.circle(
                pointer_position + surface.up() * 0.01,
                Direction3d::new_unchecked(surface.up()),
                0.1,
                Color::rgba(1.0, 0.0, 0.0, 0.5),
            );
        }
    }

    // Update rover position and remove phantom rover if phantom rover is dropped
    if scene.phantom_rover_position.is_some() && !buttons.pressed(MouseButton::Left) {
        scene.phantom_rover_position = None;
        if !invalid_position {
            let p = phantom_robot.position();
            let theta = f64::from(robot.heading());
            sim_inputs.0.push(SimInput::MoveRover { to: p });
            sim_inputs.0.push(SimInput::RequestEkfReset {
                pose: (p.x(), p.y(), theta),
            });
        }
        remove_rover(&mut scene.phantom_rover, &mut commands);
    }
}

#[allow(clippy::too_many_arguments)]
fn handle_relocation_of_rocks(
    cameras: Query<(&Camera, &GlobalTransform)>,
    surfaces: Query<&GlobalTransform, With<Surface>>,
    windows: Query<&Window>,
    mut scene: ResMut<Scene>,
    mut gizmos: Gizmos,
    transforms: Query<&Transform>,
    robot: Res<RobotRes>,
    environment: Res<EnvironmentRes>,
    buttons: Res<ButtonInput<MouseButton>>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
    sim_inputs: Res<SimInputsRes>,
    replay_active: Res<ReplayActive>,
) {
    if replay_active.0 {
        return;
    }

    let Some(pointer_position) = get_pointer_position_on_surface(&cameras, &surfaces, &windows)
    else {
        return;
    };
    let surface = surfaces.single();

    for (idx, obstacle) in environment.obstacles().iter().enumerate() {
        if !obstacle.is_movable() {
            continue;
        }

        if obstacle.contains(to_domain_position(pointer_position)) {
            let rock = scene.rocks.get(idx).unwrap();
            if let Ok(rock_transform) = transforms.get(rock.entity) {
                let rock_position = rock_transform.translation;
                let rock_distance = pointer_position.distance(rock_position);
                if scene.phantom_rock.is_none()
                    && scene.phantom_rover_position.is_none()
                    && !scene.camera.orbiting
                    && rock_distance < 0.1
                {
                    gizmos.circle(
                        rock_position + surface.up() * 0.01,
                        Direction3d::new_unchecked(surface.up()),
                        0.1,
                        Color::rgba(1.0, 1.0, 1.0, 0.2),
                    );

                    // Create phantom rock if rock is dragged
                    if buttons.pressed(MouseButton::Left) {
                        scene.phantom_rock = Some(PhantomRock {
                            entity: create_rock(
                                obstacle,
                                &mut commands,
                                &mut meshes,
                                &mut materials,
                                &asset_server,
                                0.3,
                                true,
                            )
                            .entity,
                            position: pointer_position,
                            obstacle: obstacle.clone(),
                            idx,
                        });
                    }
                }
            }
        }
    }

    let mut invalid_position = false;

    // Update position while phantom rock is dragged
    if let Some(phantom_rock) = &mut scene.phantom_rock {
        phantom_rock.position = pointer_position;
        phantom_rock
            .obstacle
            .set_position(to_domain_position(pointer_position));

        invalid_position = environment.has_collision(&phantom_rock.obstacle)
            || robot.has_collision(&phantom_rock.obstacle)
            || !environment.contains(&phantom_rock.obstacle);

        if invalid_position {
            // Draw a circle just above the ground plane at that position.
            gizmos.circle(
                pointer_position + surface.up() * 0.01,
                Direction3d::new_unchecked(surface.up()),
                0.1,
                Color::rgba(1.0, 0.0, 0.0, 0.5),
            );
        }
    }

    // Update rock position and remove phantom rock if phantom rock is dropped
    if let Some(phantom_rock) = &scene.phantom_rock {
        if !buttons.pressed(MouseButton::Left) {
            if !invalid_position {
                sim_inputs.0.push(SimInput::MoveObstacle {
                    idx: phantom_rock.idx,
                    to: phantom_rock.obstacle.position(),
                });
            }
            commands.entity(phantom_rock.entity).despawn();
            scene.phantom_rock = None;
        }
    }
}

fn handle_camera_orbit(
    mut scene: ResMut<Scene>,
    buttons: Res<ButtonInput<MouseButton>>,
    mut mouse_motion: EventReader<MouseMotion>,
    mut mouse_wheel: EventReader<MouseWheel>,
) {
    // Start orbiting on the first frame of a left press, only when no drag is in progress
    if buttons.just_pressed(MouseButton::Left)
        && scene.phantom_rover_position.is_none()
        && scene.phantom_rock.is_none()
    {
        scene.camera.orbiting = true;
    }

    if !buttons.pressed(MouseButton::Left) {
        scene.camera.orbiting = false;
    }

    // Consume mouse motion events; apply to spherical coords only when orbiting
    for event in mouse_motion.read() {
        if scene.camera.orbiting {
            scene.camera.azimuth -= event.delta.x * CAMERA_ORBIT_SENSITIVITY;
            // delta.y is positive downward in screen space; subtract to match
            // natural "drag up = look more from above" convention
            scene.camera.elevation -= event.delta.y * CAMERA_ORBIT_SENSITIVITY;
            scene.camera.elevation = scene.camera.elevation
                .clamp(CAMERA_ELEVATION_MIN, CAMERA_ELEVATION_MAX);
        }
    }

    // Zoom via scroll wheel, active regardless of orbiting state
    for event in mouse_wheel.read() {
        let delta = match event.unit {
            MouseScrollUnit::Line => event.y * CAMERA_ZOOM_LINE,
            MouseScrollUnit::Pixel => event.y * CAMERA_ZOOM_PIXEL,
        };
        // Positive scroll (up) = zoom in = smaller radius
        scene.camera.radius = (scene.camera.radius - delta)
            .clamp(CAMERA_RADIUS_MIN, CAMERA_RADIUS_MAX);
    }
}

fn get_pointer_position_on_surface(
    cameras: &Query<(&Camera, &GlobalTransform)>,
    surfaces: &Query<&GlobalTransform, With<Surface>>,
    windows: &Query<&Window>,
) -> Option<Vec3> {
    let (camera, camera_transform) = cameras.single();
    let surface = surfaces.single();
    let cursor_position = windows.get_single().ok()?.cursor_position()?;
    let ray = camera.viewport_to_world(camera_transform, cursor_position)?;
    let distance = ray.intersect_plane(surface.translation(), Plane3d::new(surface.up()))?;
    Some(ray.get_point(distance))
}

fn update_text(
    mut text: Query<&mut Text, With<StatusText>>,
    scene: Res<Scene>,
    robot: Res<RobotRes>,
    slip_free: Res<SlipFreeMode>,
    show_path: Res<ShowWaypointPath>,
) {
    let mut text = text.single_mut();
    let path_str = if show_path.0 { "  [P] PATH" } else { "" };
    if scene.show_text {
        let sen_dis = robot
            .distance_sensor_distance()
            .map_or("---".to_string(), |f| format!("{f:4.2}"));
        let sen_ang = robot.distance_sensor_angle().to_deg();
        let sen_ang = if sen_ang > 180.0 {
            -(360.0 - sen_ang)
        } else {
            sen_ang
        };
        let vel_left = robot
            .wheel_velocity(WheelID::FrontLeft)
            .map_or("---".to_string(), |f| {
                format!("{:4.2}", Into::<f64>::into(f))
            });
        let vel_right = robot
            .wheel_velocity(WheelID::FrontRight)
            .map_or("---".to_string(), |f| {
                format!("{:4.2}", Into::<f64>::into(f))
            });
        let slip_str = if slip_free.0 { "  [F] SLIP-FREE" } else { "" };
        text.sections[0].value = format!(
            "SEN: {sen_dis} m {sen_ang:3.0} deg   VEL: {vel_left} m/s {vel_right} m/s{slip_str}{path_str}"
        );
    } else {
        // Show mode indicators even when detailed status is off.
        text.sections[0].value = path_str.trim_start().to_string();
    }
}

fn handle_keyboard_input(
    keys: Res<ButtonInput<KeyCode>>,
    mut scene: ResMut<Scene>,
    robot: Res<RobotRes>,
    slip_free: Res<SlipFreeMode>,
    mut reset_events: EventWriter<crate::ribbon::RoverResetEvent>,
    mut show_path: ResMut<ShowWaypointPath>,
    sim_inputs: Res<SimInputsRes>,
    replay_active: Res<ReplayActive>,
) {
    // R is overloaded:
    //   live   → push SimInput::Reset so the sim re-centres the rover
    //   replay → handled in `replay_mode::handle_replay_keys` (rewinds the
    //            trace cursor); still clear the ribbon trail here so the old
    //            path doesn't linger behind the rewound rover.
    if keys.just_pressed(KeyCode::KeyR) {
        if !replay_active.0 {
            let theta = f64::from(robot.heading());
            sim_inputs.0.push(SimInput::Reset {
                pose: (0.0, 0.0, theta),
            });
        }
        reset_events.send(crate::ribbon::RoverResetEvent);
    }

    if keys.just_pressed(KeyCode::KeyT) {
        scene.show_text = !scene.show_text;
    }

    // F toggles slip-free mode, which is a sim-only flag; in replay the
    // trajectory is fixed, so the toggle has no meaning.
    if keys.just_pressed(KeyCode::KeyF) && !replay_active.0 {
        sim_inputs.0.push(SimInput::SetSlipFree(!slip_free.0));
    }

    if keys.just_pressed(KeyCode::KeyP) {
        show_path.0 = !show_path.0;
    }
}

/// Draw the EKF estimated position and heading each frame.
///
/// A yellow circle marks the estimated position; a short yellow line segment
/// indicates the estimated heading θ.  The marker appears only after Ada's
/// filter has initialised (i.e., after the first GPS fix is received).
///
/// Coordinate convention: world (x, y) → Bevy (x, terrain_h, −y), same as
/// the rover body transform in `transform_rover`.
fn draw_ekf_marker(mut gizmos: Gizmos, terrain: Res<TerrainRes>) {
    let Some((est_x, est_y, theta)) = controller::ekf_estimated_state() else {
        return;
    };

    let h = terrain.height_at(est_x as f64, est_y as f64) as f32 + 0.02;
    let center = Vec3::new(est_x, h, -est_y);

    // Position circle
    gizmos.circle(
        center,
        Direction3d::new_unchecked(Vec3::Y),
        0.12,
        Color::rgba(1.0, 0.9, 0.0, 0.85),
    );

    // Heading line: theta=0 points along +x in world frame → +x in Bevy.
    // Bevy z is −y_world, so the heading direction is (cos θ, 0, −sin θ).
    let heading_end = center + Vec3::new(theta.cos() * 0.15, 0.0, -theta.sin() * 0.15);
    gizmos.line(center, heading_end, Color::rgba(1.0, 0.9, 0.0, 0.85));
}

/// Draw the waypoint path as a cyan line strip with dots at each waypoint.
///
/// Only rendered when `ShowWaypointPath` is on (toggle with `P`).  Each
/// segment is lifted slightly above the terrain surface so it remains
/// visible regardless of slope.  The path is redrawn every frame (Bevy
/// gizmos are ephemeral) so toggling takes effect immediately.
fn draw_waypoint_path(
    mut gizmos: Gizmos,
    waypoints: Res<WaypointsRes>,
    show_path: Res<ShowWaypointPath>,
    terrain: Res<TerrainRes>,
) {
    if !show_path.0 || waypoints.0.is_empty() {
        return;
    }

    const PATH_LIFT: f32 = 0.05;
    const PATH_COLOR: Color = Color::rgba(0.0, 0.85, 1.0, 0.9);
    const DOT_RADIUS: f32 = 0.08;

    let positions: Vec<Vec3> = waypoints
        .0
        .iter()
        .map(|&(wx, wy)| {
            let h = terrain.height_at(wx, wy) as f32 + PATH_LIFT;
            Vec3::new(wx as f32, h, -(wy as f32))
        })
        .collect();

    for pair in positions.windows(2) {
        gizmos.line(pair[0], pair[1], PATH_COLOR);
    }
    for &pos in &positions {
        gizmos.circle(pos, Direction3d::new_unchecked(Vec3::Y), DOT_RADIUS, PATH_COLOR);
    }
}

fn to_domain_position(position: Vec3) -> Position {
    Position::new(position.x as f64, -position.z as f64)
}

fn to_bevy_position(position: Position) -> Vec3 {
    Vec3::new(position.x() as f32, 0.0, -position.y() as f32)
}
