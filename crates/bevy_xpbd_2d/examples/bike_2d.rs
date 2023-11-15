use bevy::prelude::*;
use bevy::sprite::MaterialMesh2dBundle;
use bevy::transform::TransformSystem;
use bevy_xpbd_2d::{math::*, prelude::*};
use examples_common_2d::XpbdExamplePlugin;
use itertools::interleave;

//use bevy_editor_pls;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, XpbdExamplePlugin))
        // p=pause, Return=step
        //.add_plugins(bevy_editor_pls::EditorPlugin::default())
        .insert_resource(ClearColor(Color::rgb(0.05, 0.05, 0.1)))
        .insert_resource(SubstepCount(50))
        // higher gravity makes the wheelies faster
        .insert_resource(Gravity(Vector::NEG_Y * 160.0))
        .add_systems(Startup, setup)
        .add_systems(Update, motor_run)
        .add_systems(
            PostUpdate, camera_follow
                .after(bevy_xpbd_2d::PhysicsSet::Sync)
                .before(TransformSystem::TransformPropagate)
        )
        .run();
}

/// Used to straighten the bike to horizontal together with the Motor.
#[derive(Component)]
struct Wheel;

/// Player applies torque to this.
#[derive(Component)]
struct Motor;

#[derive(Component)]
struct BikeCamera;

#[derive(Component)]
struct BikeCameraTarget;

/// The part of the body where motor torque should be applied as a fake reaction force (in the opposite direction compared to the motor).
#[derive(Component)]
struct MotorTorqueBody;

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    commands.spawn((
        Camera2dBundle::default(),
        BikeCamera,
    ));

    let square_sprite = Sprite {
        color: Color::rgb(0.2, 0.7, 0.9),
        custom_size: Some(Vec2::splat(50.0)),
        ..default()
    };

    
    let motor_sprite = Sprite {
        color: Color::rgb(0.9, 0.7, 0.9),
        custom_size: Some(Vec2::splat(50.0)),
        ..default()
    };
    
    let wheel_sprite = Sprite {
        color: Color::rgb(0.2, 0.2, 0.9),
        custom_size: Some(Vec2::splat(50.0)),
        ..default()
    };

    let floor_sprite = Sprite {
        color: Color::rgb(0.2, 0.7, 0.2),
        custom_size: Some(Vec2::splat(50.0)),
        ..default()
    };
    
    let motor = commands
        .spawn((
            SpriteBundle {
                sprite: motor_sprite.clone(),
                transform: Transform::from_xyz(-100.0, -100.0, 0.0),
                ..default()
            },
            RigidBody::Dynamic,
            Collider::ball(30.0),
            // super low mass, so that landing makes the wheel take ground's speed rather than jolt the entire mass and bounce off
            Mass(0.0000001),
            // stick to the ground!
            Friction {
                static_coefficient: 1000.0,
                dynamic_coefficient: 1000.0,
                ..Default::default()
            },
            Restitution {
                coefficient: 0.0005,
                combine_rule: CoefficientCombine::Min,
            },
            Motor,
        ))
        .id();

    let wheel = commands
        .spawn((
            SpriteBundle {
                sprite: wheel_sprite.clone(),
                transform: Transform::from_xyz(100.0, -100.0, 0.0),
                ..default()
            },
            RigidBody::Dynamic,
            Collider::ball(30.0),
            Mass(0.000001),
            // less bouncy in general
            Restitution {
                coefficient: 0.005,
                combine_rule: CoefficientCombine::Min,
            },

            Friction {
                static_coefficient: 1000.0,
                dynamic_coefficient: 1000.0,
                ..Default::default()
            },
            
            Wheel,
        ))
        .id();
        
    let body = commands
        .spawn((
            SpriteBundle {
                sprite: square_sprite,
                transform: Transform::from_xyz(50.0, 0.0, 0.0),
                ..default()
            },
            RigidBody::Dynamic,
            Collider::cuboid(50.0, 50.0),
            Mass(0.0001),
            // feels weird it the motorcycle doesn't tumble after falling over
            Friction {
                static_coefficient: 40.0,
                dynamic_coefficient: 40.0,
                ..Default::default()
            },
            MotorTorqueBody,
            BikeCameraTarget,
        ))
        .id();
        
    commands.spawn((
        RevoluteJoint::new(motor, body)
            .with_local_anchor_2(Vector::Y * -100.0 + Vector::X * -150.0)
            // way glitchy otherwise. A guess: if the collision has a smooth transition between "no contact" and "full force", then it doesn't "overreact" when the time step missed the exact time of something happening. Similar effect might have been had by adding penetration to the wheel (deformation), but doing that is underdocumented.
            // also: maybe this reduces bounciness..
            .with_compliance(0.000001)
            // ...together with this. When landing from a jump, the compliance accepts the jolt and damping dissipates it.
            // It feels a bit rubbery, though, and limits angling for wheelies for some reason. It might be fixable by introducing another joint type and replacing joint_damping system with one that treats radial velocity different than axial.
            .with_linear_velocity_damping(5.0)
            // maybe this will reduce bounciness on touching the ground: the contact with the ground will not try to change the momentum the entire mass of the bike but only the wheel - less of a jolt.
            .with_angular_velocity_damping(0.0),
        Motor,
    ));

    commands.spawn(
        RevoluteJoint::new(wheel, body)
            .with_local_anchor_2(Vector::Y * -100.0 + Vector::X * 50.0)
            .with_compliance(0.000001)
            .with_linear_velocity_damping(5.0),
    );
    
    const FLOOR_WIDTH: u64 = 100000;
    
    /*
    commands.spawn((
        SpriteBundle {
            sprite: floor_sprite.clone(),
            transform: Transform::from_xyz(0.0, -50.0 * 6.0, 0.0)
                .with_scale(Vec3::new(FLOOR_WIDTH as f32 / 50.0, 1.0, 1.0)),
            ..default()
        },
        RigidBody::Static,
        Collider::cuboid(50.0, 50.0),
    ));*/
    
    // A tilted section of the floor to test jumps
    /*
    commands.spawn((
        SpriteBundle {
            sprite: floor_sprite.clone(),
            transform: Transform::from_xyz(400.0, -50.0 * 6.0, 0.0)
                .with_rotation(Quat::from_rotation_z(0.1))
                .with_scale(Vec3::new(600 as f32 / 50.0, 1.0, 1.0)),
            ..default()
        },
        RigidBody::Static,
        Collider::cuboid(50.0, 50.0),
    ));*/
    
    const CLOUD_SPACING: u64 = 300;
    
    let cloud_sprite = Sprite {
        color: Color::rgb(0.7, 0.7, 0.7),
        custom_size: Some(Vec2::splat(50.0)),
        ..default()
    };
    
    fn hash(v: u64) -> u32 {
        // Something something Knuth.
        // https://stackoverflow.com/a/665545
        (v.wrapping_mul(2654435761) % (1 << 32)) as u32
    }
    
    for i in 0..(FLOOR_WIDTH / CLOUD_SPACING) {
        let offset = (i * CLOUD_SPACING) as i64 - FLOOR_WIDTH as i64 / 2;
        commands.spawn(SpriteBundle {
            sprite: cloud_sprite.clone(),
            // y=-1 -> background
            transform: Transform::from_xyz(offset as f32, 100.0, -1.0)
                // randomize the rotation or else it's even more unclear what the squares are
                .with_rotation(Quat::from_rotation_z(hash(i) as f32)),
            ..default()
        });
    }
    
    // bumpy floor
    {
        let tops = (0..300).map(|i| [
            // offset
            i as f32 * 600.0 + 
            // normalize to 0..1
            (hash(hash(i) as u64) as f32 / (1u64 << 32) as f32)
            // stretch to 300..3300
                * 3000.0 + 300.0,
            // height
            hash(i) as f32 / (1u64 << 32) as f32 * 150.0,
            0.0,
        ]);
        
        let bottoms = tops.clone().map(|[x, y, z]| [x, y + 100.0, z]);
        
        let vertices = interleave(tops, bottoms).collect::<Vec<_>>();
        
        let normals = (0..600).map(|_| [0.0, 0.0, 1.0]).collect::<Vec<_>>();
        
        let mesh = Mesh::new(bevy::render::render_resource::PrimitiveTopology::TriangleStrip)
            .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, vertices.clone())
            .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
            
        let bumps_mesh = meshes
            .add(mesh.clone())
            .into();
        let bumps_material = materials.add(ColorMaterial::from(Color::rgb(0.2, 0.7, 0.2)));
        
        commands
            .spawn((
                MaterialMesh2dBundle {
                    mesh: bumps_mesh,
                    material: bumps_material,
                    transform: Transform::from_xyz(-600.0, -350.0, 0.0),
                    ..default()
                },
                RigidBody::Static,
                Collider::trimesh(
                    vertices.into_iter().map(|[x,y,_]| Vec2::new(x, y)).collect(),
                    (0..598).map(|i| [i, i+1, i+2]).collect(),
                )
            ));
    }
}


fn clamp(v: f32) -> f32 {
    if v < 0.0 {
        0.0
    } else if v > 1.0 {
        1.0
    } else {
        v
    }
}

/// Returns absolute percentage
fn current_torque(angular: f32) -> f32 {
    // Idle spinning angular velocity forward
    let max_angular = 30.0;
    let max_angular_reverse = max_angular / 5.0;
    if angular > 0.0 {
        clamp(1.0 - angular.abs() / max_angular)
    } else {
        clamp(1.0 - angular.abs() / max_angular_reverse)
    }
}

fn motor_run(
    time: Res<Time>,
    keyboard_input: Res<Input<KeyCode>>,
    mut motors: Query<
        (&mut ExternalTorque, Ref<AngularVelocity>, Ref<LinearVelocity>, Ref<Transform>),
        With<Motor>,
    >,
    mut application_points: Query<
        &mut ExternalTorque,
        (With<MotorTorqueBody>, Without<Motor>),
    >,
    wheels: Query<
        Ref<Transform>,
        (With<Wheel>, Without<Motor>),
    >,
) {
    // Precision is adjusted so that the example works with
    // both the `f32` and `f64` features. Otherwise you don't need this.
    //let delta_time = time.delta_seconds_f64().adjust_precision();
    
    // 100% torque
    let max_torque = -50000000.0;

    // quadratic complexity, but we have one of each so whatever. The code is less bug-prone this way
    for (mut torque, angular, linear, motor_transform) in &mut motors {
        let magnitude = max_torque * current_torque(-angular.0);
        for mut antitorque in &mut application_points {
            if keyboard_input.any_pressed([KeyCode::W, KeyCode::Up]) {
                *torque = ExternalTorque::new(magnitude)
                    .with_persistence(false);
                // the linear damping on joints interferes with torque. Let's make it stronger
                *antitorque = ExternalTorque::new(-max_torque * 3.0)
                    .with_persistence(false);
            } else if keyboard_input.any_pressed([KeyCode::S, KeyCode::Down]) {
                *torque = ExternalTorque::new(-magnitude)
                    .with_persistence(false);
                // the linear damping on joints interferes with torque. Let's make it stronger
                *antitorque = ExternalTorque::new(max_torque * 3.0)
                    .with_persistence(false);
            }
            // Not physical: straightening the bike back to horizontal. Hopefully gives a better feel.
            else {
                let wheel_translation = wheels.single().translation;
                let horz_difference = wheel_translation.x - motor_transform.translation.x;
                let vert_difference = wheel_translation.y - motor_transform.translation.y;
                
                let angle = vert_difference.atan2(horz_difference);
                
                // About 1 o'clock, to emulate air resistance keeping the whole thing upright.
                let ideal_angle = std::f32::consts::TAU / 4.0 - std::f32::consts::TAU / 16.0;
                
                let difference_to_ideal = ideal_angle - angle;
                
                // something to destabilize: goes from 1.0 when wheels are perfect to 0.0 when horizontal
                let straightening_factor = if difference_to_ideal > 0.0 {
                    // before reaching tipping point: flat to ideal_angle scaling
                    1.0 - difference_to_ideal / ideal_angle
                } else {
                    // ideal_angle to flipped upside down takes the complementary part of a half-turn
                    -1.0 + difference_to_ideal / (std::f32::consts::TAU / 2.0 - ideal_angle)
                };
                
                *antitorque = ExternalTorque::new(magnitude * 5.0 * straightening_factor)
                    .with_persistence(false);
            }
        }
    }
}

fn camera_follow(
    // the Target could be a Resource instead, but grabbing the Entity directly saves Resource setup and is just as easy.
    targets: Query<Ref<Transform>, With<BikeCameraTarget>>,
    mut cameras: Query<
        &mut Transform,
        (With<BikeCamera>, Without<BikeCameraTarget>),
    >,
) {
    let motor_x = targets.single().translation.x;
    let mut camera_transform = cameras.single_mut();
    camera_transform.translation.x = motor_x + 300.0; // TODO: base offset on viewport width
}