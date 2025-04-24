//! 3D visualization.

use std::collections::BTreeMap;

use bevy::{
    core_pipeline::{bloom::BloomSettings, tonemapping::Tonemapping},
    pbr::NotShadowCaster,
    prelude::*,
};
use rand::{
    distr::{Distribution, Uniform},
    Rng, SeedableRng,
};
use rand_chacha::ChaCha8Rng;

use crate::{
    domain::{
        Angle, Environment, HasCollision, Movability, Obstacle, Position, Robot, RobotConfig,
        WheelID,
    },
    resource::{EnvironmentRes, RobotRes},
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
                    handle_relocation_of_rover,
                    handle_relocation_of_rocks,
                    (focus_camera, update_rover, update_rocks).chain(),
                ),
            )
            .insert_resource(create_robot())
            .insert_resource(create_environment())
            .init_resource::<Scene>();
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

#[derive(Default)]
struct SceneCamera {
    is_focus: Vec3,
    should_focus: Vec3,
    is_position: Vec3,
    should_position: Vec3,
}

#[derive(Component)]
struct Surface;

const RNG_SEED: u64 = 19878367467712;

fn create_robot() -> RobotRes {
    Robot::new(
        Angle::new(0.0),
        RobotConfig::new(
            0.066,
            0.049,
            0.0385,
            0.0385,
            0.08,
            0.075,
            0.0565,
            0.0565,
            0.028,
            0.0225,
            0.017,
            Position::new(0.0375, -0.0125),
            0.018,
            0.045,
            0.027,
            0.018,
            0.007,
        ),
    )
    .into()
}

fn create_environment() -> EnvironmentRes {
    const X_MAX: i32 = 24;
    const Y_MAX: i32 = 14;
    const LENGTH: f64 = 0.1;
    const HALF_LENGTH: f64 = 0.05;

    let mut rng = ChaCha8Rng::seed_from_u64(RNG_SEED);
    let position_offset = Uniform::try_from(-HALF_LENGTH..=HALF_LENGTH).unwrap();
    let length_offset = Uniform::try_from(0.0..=LENGTH).unwrap();

    let mut obstacles = vec![];

    for x in -X_MAX + 1..=X_MAX - 1 {
        for y in [
            (-Y_MAX - 1) as f64 / 10.0,
            -Y_MAX as f64 / 10.0,
            Y_MAX as f64 / 10.0,
            (Y_MAX + 1) as f64 / 10.0,
        ] {
            obstacles.push(Obstacle::new(
                Position::new(
                    x as f64 / 10.0 + position_offset.sample(&mut rng),
                    y + position_offset.sample(&mut rng),
                ),
                LENGTH + length_offset.sample(&mut rng),
            ));
        }
    }

    for x in [
        (-X_MAX - 1) as f64 / 10.0,
        -X_MAX as f64 / 10.0,
        X_MAX as f64 / 10.0,
        (X_MAX + 1) as f64 / 10.0,
    ] {
        for y in -Y_MAX + 1..=Y_MAX - 1 {
            obstacles.push(Obstacle::new(
                Position::new(
                    x + position_offset.sample(&mut rng),
                    y as f64 / 10.0 + position_offset.sample(&mut rng),
                ),
                LENGTH + length_offset.sample(&mut rng),
            ));
        }
    }

    for (x, y) in [(-0.8, -0.25), (0.8, 0.25)] {
        obstacles
            .push(Obstacle::new(Position::new(x, y), 0.1).with_movability(Movability::Movable));
    }

    Environment::new(obstacles).into()
}

fn set_up(
    mut scene: ResMut<Scene>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
    robot: Res<RobotRes>,
    environment: Res<EnvironmentRes>,
) {
    create_surface(
        &environment,
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
    scene: &mut ResMut<Scene>,
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    asset_server: &Res<AssetServer>,
) {
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Plane3d::default().mesh()),
            material: materials.add(Color::WHITE),
            visibility: Visibility::Hidden,
            ..default()
        },
        Surface,
    ));

    const TILE_SIZE: f32 = 1.0;
    let x_min = (environment.x_min() - f64::EPSILON) as i32;
    let x_max = (environment.x_max() + f64::EPSILON) as i32;
    let y_min = (environment.y_min() - f64::EPSILON).floor() as i32;
    let y_max = (environment.y_max() + f64::EPSILON).ceil() as i32;

    for x in x_min..=x_max {
        for y in y_min..=y_max {
            commands.spawn(PbrBundle {
                mesh: meshes.add(Rectangle::new(TILE_SIZE, TILE_SIZE)),
                material: materials.add(surface_material(asset_server)),
                transform: Transform::from_rotation(Quat::from_rotation_x(
                    -std::f32::consts::FRAC_PI_2,
                )) * Transform::from_xyz(x as f32, y as f32 - TILE_SIZE / 2.0, 0.0),
                ..default()
            });
        }
    }

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

fn surface_material(asset_server: &Res<AssetServer>) -> StandardMaterial {
    StandardMaterial {
        base_color_texture: Some(asset_server.load("textures/mars_surface.png")),
        perceptual_roughness: 1.0,
        thickness: 1.0,
        attenuation_distance: 0.0,
        ..default()
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

const INITIAL_CAMERA_POSITION: [f32; 3] = [0.0, 1.0, 2.0];

fn create_camera(scene: &mut ResMut<Scene>, commands: &mut Commands) {
    scene.camera.is_focus = Vec3::ZERO;
    scene.camera.should_focus = scene.camera.is_focus;
    scene.camera.is_position = Vec3::from(INITIAL_CAMERA_POSITION);
    scene.camera.should_position = scene.camera.is_position;

    commands.spawn((
        Camera3dBundle {
            camera: Camera {
                hdr: true,
                ..default()
            },
            tonemapping: Tonemapping::TonyMcMapface,
            transform: Transform::from_translation(scene.camera.is_position)
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
    commands.spawn(
        TextBundle::from_sections(vec![TextSection::new("", text_style.clone())]).with_style(
            Style {
                position_type: PositionType::Absolute,
                bottom: Val::Px(12.0),
                left: Val::Px(12.0),
                ..default()
            },
        ),
    );
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

fn update_rover(scene: Res<Scene>, mut transforms: Query<&mut Transform>, robot: Res<RobotRes>) {
    transform_rover(&scene.rover, &robot, &mut transforms);

    if let Some(position) = scene.phantom_rover_position {
        let mut robot = robot.clone();
        robot.set_position(to_domain_position(position));
        transform_rover(&scene.phantom_rover, &robot, &mut transforms);
    }
}

fn transform_rover(rover: &Rover, robot: &Robot, transforms: &mut Query<&mut Transform>) {
    let pos = robot.position();
    if let Some(entity) = rover.body {
        if let Ok(mut rover_transform) = transforms.get_mut(entity) {
            *rover_transform = Transform {
                translation: Vec3::new(
                    pos.x() as f32,
                    robot.config().ground_clearance as f32,
                    -pos.y() as f32,
                ),
                rotation: Quat::from_rotation_y(robot.heading().into()),
                ..default()
            };
        }
    }

    for wheel_id in WheelID::iter() {
        let pos = robot.wheel_position(*wheel_id);
        if let Some(entity) = rover.wheels[wheel_id] {
            if let Ok(mut wheel_transform) = transforms.get_mut(entity) {
                *wheel_transform = Transform {
                    translation: Vec3::new(
                        pos.x() as f32,
                        robot.config().wheel_radius as f32,
                        -pos.y() as f32,
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

    let pos = robot.distance_sensor_position();
    if let Some(entity) = rover.distance_sensor {
        if let Ok(mut distance_sensor_transform) = transforms.get_mut(entity) {
            *distance_sensor_transform = Transform {
                translation: Vec3::new(
                    pos.x() as f32,
                    (robot.config().ground_clearance
                        + robot.config().body_height
                        + 0.5 * robot.config().distance_sensor_height) as f32,
                    -pos.y() as f32,
                ),
                rotation: Quat::from_rotation_y(robot.distance_sensor_heading().into())
                    * Quat::from_rotation_z(-std::f32::consts::FRAC_PI_2),
                ..default()
            };
        }
    }
    if let Some(entity) = rover.distance_sensor_light {
        if let Ok(mut distance_sensor_light_transform) = transforms.get_mut(entity) {
            *distance_sensor_light_transform = Transform {
                translation: Vec3::new(
                    pos.x() as f32,
                    (robot.config().ground_clearance
                        + robot.config().body_height
                        + 0.5 * robot.config().distance_sensor_height) as f32,
                    -pos.y() as f32,
                ),
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

    // Smooth out the camera movement
    let mut camera_motion = scene.camera.should_focus - scene.camera.is_focus;
    if camera_motion.length() > 0.2 {
        camera_motion *= SPEED * time.delta_seconds();
        scene.camera.is_focus += camera_motion;
    }

    scene.camera.should_position = scene.camera.should_focus
        + 1.5 * (Vec3::from(INITIAL_CAMERA_POSITION) - scene.camera.should_focus).normalize();
    let mut camera_motion = scene.camera.should_position - scene.camera.is_position;
    if camera_motion.length() > 0.1 {
        camera_motion *= SPEED * time.delta_seconds();
        scene.camera.is_position += camera_motion;
    }

    for mut transform in transforms.p0().iter_mut() {
        *transform = Transform::from_translation(scene.camera.is_position)
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
    mut robot: ResMut<RobotRes>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
) {
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
            robot.set_position(phantom_robot.position());
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
    mut environment: ResMut<EnvironmentRes>,
    buttons: Res<ButtonInput<MouseButton>>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
) {
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
                environment
                    .set_obstacle_position(phantom_rock.idx, phantom_rock.obstacle.position());
            }
            commands.entity(phantom_rock.entity).despawn();
            scene.phantom_rock = None;
        }
    }
}

fn get_pointer_position_on_surface(
    cameras: &Query<(&Camera, &GlobalTransform)>,
    surfaces: &Query<&GlobalTransform, With<Surface>>,
    windows: &Query<&Window>,
) -> Option<Vec3> {
    let (camera, camera_transform) = cameras.single();
    let surface = surfaces.single();
    let cursor_position = windows.single().cursor_position()?;
    let ray = camera.viewport_to_world(camera_transform, cursor_position)?;
    let distance = ray.intersect_plane(surface.translation(), Plane3d::new(surface.up()))?;
    Some(ray.get_point(distance))
}

fn update_text(mut text: Query<&mut Text>, scene: Res<Scene>, robot: Res<RobotRes>) {
    let mut text = text.single_mut();
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
        text.sections[0].value =
            format!("SEN: {sen_dis} m {sen_ang:3.0} deg   VEL: {vel_left} m/s {vel_right} m/s");
    } else {
        text.sections[0].value = String::new();
    }
}

fn handle_keyboard_input(
    keys: Res<ButtonInput<KeyCode>>,
    mut scene: ResMut<Scene>,
    mut robot: ResMut<RobotRes>,
) {
    if keys.just_pressed(KeyCode::KeyR) {
        robot.set_position(Position::default());
    }

    if keys.just_pressed(KeyCode::KeyT) {
        scene.show_text = !scene.show_text;
    }
}

fn to_domain_position(position: Vec3) -> Position {
    Position::new(position.x as f64, -position.z as f64)
}

fn to_bevy_position(position: Position) -> Vec3 {
    Vec3::new(position.x() as f32, 0.0, -position.y() as f32)
}
