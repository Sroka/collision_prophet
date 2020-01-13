use nalgebra::Vector3;
use std::slice::from_raw_parts;
use crate::collision_prophet;

#[no_mangle]
pub unsafe extern fn collides(
    bounding_box: Vector3<f32>,
    position: Vector3<f32>,
    velocity: Vector3<f32>,
    obstacle_bounding_boxes_ptr: *const Vector3<f32>,
    obstacle_positions_ptr: *const Vector3<f32>,
    obstacle_velocities_ptr: *const Vector3<f32>,
    obstacles_count: i32,
    max_toi: f32,
    target_distance: f32,
) -> bool {
    let obstacle_bounding_boxes = from_raw_parts(obstacle_bounding_boxes_ptr, obstacles_count as usize);
    let obstacle_positions = from_raw_parts(obstacle_positions_ptr, obstacles_count as usize);
    let obstacle_velocities = from_raw_parts(obstacle_velocities_ptr, obstacles_count as usize);
    return collision_prophet::collides(
        &bounding_box,
        &position,
        &velocity,
        &obstacle_bounding_boxes,
        &obstacle_positions,
        &obstacle_velocities,
        max_toi,
        target_distance,
    );
}

