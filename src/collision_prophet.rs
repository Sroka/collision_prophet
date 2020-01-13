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
//        dbg!(bounding_box);
//        dbg!(position);
//        dbg!(velocity);
//        dbg!(obstacle_bounding_boxes);
//        dbg!(obstacle_positions);
//        dbg!(obstacle_velocities);
    for index in 0..obstacle_positions.len() {
        let bounding_cube = Cuboid::new(*bounding_box * 0.5);
        let isometry_position = Isometry3::new(*position, nalgebra::zero());
        let obstacle_bounding_cube = Cuboid::new(obstacle_bounding_boxes[index] * 0.5);
        let obstacle_isometry_position = Isometry3::new(obstacle_positions[index], nalgebra::zero());

        let time_of_impact = query::time_of_impact(&isometry_position,
                                                   velocity,
                                                   &bounding_cube,
                                                   &obstacle_isometry_position,
                                                   &obstacle_velocities[index],
                                                   &obstacle_bounding_cube,
                                                   max_toi,
                                                   target_distance);
//        dbg!(&time_of_impact);
        match time_of_impact {
            None => {},
            Some(_) => return true,
        }
    }
    return false;
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

    #[test]
    fn test_straight_movement_doesnt_collide() {
        let bounding_box = Vector3::new(0.1, 0.1, 0.1);
        let position = Vector3::new(4.0, 49.61846, 0.0);
        let velocity = Vector3::new(-1.2935, -15.9474, 0.06603293);
        let obstacle_bounding_box = Vector3::new(2.5, 9.33, 1.0);
        let obstacle_position = Vector3::new(0.0, 42.293457, 0.0);
        let obstacle_velocity = Vector3::new(0.0, -1.0, 0.0);
        let collides = collides(
            &bounding_box,
            &position,
            &velocity,
            &[obstacle_bounding_box],
            &[obstacle_position],
            &[obstacle_velocity],
            100.0,
            0.0,
        );
        dbg!(position + velocity * 0.61042076);
        dbg!(obstacle_position + obstacle_velocity * 0.61042076);
        assert_eq!(collides, false);
    }
}
