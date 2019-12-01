use nalgebra::Vector3;
use std::slice::from_raw_parts;

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

pub mod collision_prophet {
    use nalgebra::{Vector3, Isometry3};
    use ncollide3d::shape::Cuboid;
    use ncollide3d::query;

    pub fn collides(
        bounding_box: &Vector3<f32>,
        position: &Vector3<f32>,
        velocity: &Vector3<f32>,
        obstacle_bounding_boxes: &[Vector3<f32>],
        obstacle_positions: &[Vector3<f32>],
        obstacle_velocities: &[Vector3<f32>],
        max_toi: f32,
        target_distance: f32,
    ) -> bool {
//        print!("Bounding box {}", bounding_box);
//        print!("Position {}", position);
//        print!("Velocity {}", velocity);
//        println!("Obstacles bounding boxes {:?}", obstacle_bounding_boxes);
//        println!("Obstacles positions {:?}", obstacle_positions);
//        println!("Obstacles velocities {:?}", obstacle_velocities);
        for index in 0..obstacle_positions.len() {
            let bounding_cube = Cuboid::new(*bounding_box);
            let isometry_position = Isometry3::new(*position, nalgebra::zero());
            let obstacle_bounding_cube = Cuboid::new(obstacle_bounding_boxes[index]);
            let obstacle_isometry_position = Isometry3::new(obstacle_positions[index], nalgebra::zero());

            let time_of_impact = query::time_of_impact(&isometry_position,
                                                       velocity,
                                                       &bounding_cube,
                                                       &obstacle_isometry_position,
                                                       &obstacle_velocities[index],
                                                       &obstacle_bounding_cube,
                                                       max_toi,
                                                       target_distance);
            match time_of_impact {
                None => return false,
                Some(_) => return true,
            }
        }
        return false;
    }
}

#[cfg(test)]
mod test {
    use crate::collision_prophet::collides;
    use nalgebra::Vector3;

    #[test]
    fn test_straight_movement_collides() {
        assert_eq!(collides(
            &Vector3::new(1.0, 1.0, 1.0),
            &Vector3::new(0.0, 10.0, 0.0),
            &Vector3::new(1.0, -2.0, 1.0),
            &[Vector3::new(1.0, 1.0, 1.0)],
            &[Vector3::new(0.0, 0.0, 0.0)],
            &[Vector3::new(1.0, -1.0, 1.0)],
            100.0,
            100.0,
        ), true);
    }
}