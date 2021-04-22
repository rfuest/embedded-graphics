use crate::{
    geometry::{angle_consts::*, Angle, Point},
    primitives::common::{LineSide, OriginLinearEquation, PointType},
};

#[derive(Copy, Clone, PartialEq, PartialOrd, Debug)]
enum Operation {
    /// Return the intersection of both half planes.
    Intersection,
    /// Return the union of both half planes.
    Union,
    /// Return the entire plane.
    EntirePlane,
}

impl Operation {
    /// Executes the operation.
    fn execute(self, first: bool, second: bool) -> bool {
        match self {
            Operation::Intersection => first && second,
            Operation::Union => first || second,
            Operation::EntirePlane => true,
        }
    }
}

/// Sector shaped part of a plane.
///
/// The shape is described by two half-planes that divide the XY plane along the two
/// lines from the center point to the arc's end points. For sweep angles < 180° the
/// intersection of both half-planes is used and for angles >= 180° the union of both
/// half-planes.
#[derive(Copy, Clone, PartialEq, PartialOrd, Debug)]
pub struct PlaneSector {
    /// Half plane on the left side of a line.
    half_plane_left: OriginLinearEquation,

    /// Half plane on the right side of a line.
    half_plane_right: OriginLinearEquation,

    /// The operation used to combine the two half planes.
    operation: Operation,
}

impl PlaneSector {
    pub fn new(mut angle_start: Angle, angle_sweep: Angle) -> Self {
        let angle_sweep_abs = angle_sweep.abs();

        let operation = if angle_sweep_abs >= ANGLE_360DEG {
            // Skip calculation of half planes if the absolute value of the sweep angle is >= 360°.
            return Self {
                half_plane_left: OriginLinearEquation::new_horizontal(),
                half_plane_right: OriginLinearEquation::new_horizontal(),
                operation: Operation::EntirePlane,
            };
        } else if angle_sweep_abs >= ANGLE_180DEG {
            Operation::Union
        } else {
            Operation::Intersection
        };

        let mut angle_end = angle_start + angle_sweep;

        // Swap angles for negative sweeps to use the correct sides of the half planes.
        if angle_sweep < Angle::zero() {
            core::mem::swap(&mut angle_start, &mut angle_end)
        }

        Self {
            half_plane_left: OriginLinearEquation::with_angle(angle_start),
            half_plane_right: OriginLinearEquation::with_angle(angle_end),
            operation,
        }
    }

    pub fn contains(&self, point: Point) -> bool {
        let correct_side_1 = self.half_plane_left.check_side(point, LineSide::Left);
        let correct_side_2 = self.half_plane_right.check_side(point, LineSide::Right);

        self.operation.execute(correct_side_1, correct_side_2)
    }

    /// Checks if a point is inside the stroke or fill area.
    pub fn point_type(
        &self,
        point: Point,
        inside_threshold: i32,
        outside_threshold: i32,
    ) -> Option<PointType> {
        let distance_left = self.half_plane_left.distance(point);
        let distance_right = -self.half_plane_right.distance(point);

        if !self.operation.execute(
            distance_left >= outside_threshold,
            distance_right >= outside_threshold,
        ) {
            None
        } else if !self.operation.execute(
            distance_left >= inside_threshold,
            distance_right >= inside_threshold,
        ) {
            Some(PointType::Stroke)
        } else {
            Some(PointType::Fill)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        geometry::AngleUnit,
        primitives::{common::NORMAL_VECTOR_SCALE, Line, PointsIter},
    };

    use arrayvec::ArrayVec;

    /// Checks if the plane sector contains 8 different points.
    ///
    /// Four of the points lie on the boundary between two adjacent quadrants and should be
    /// contained in both quadrants. The remaining points are inside a single quadrant.
    fn contains(plane_sector: &PlaneSector) -> [bool; 8] {
        [
            plane_sector.contains(Point::new(10, 0)),
            plane_sector.contains(Point::new(10, -10)),
            plane_sector.contains(Point::new(0, -10)),
            plane_sector.contains(Point::new(-10, -10)),
            plane_sector.contains(Point::new(-10, 0)),
            plane_sector.contains(Point::new(-10, 10)),
            plane_sector.contains(Point::new(0, 10)),
            plane_sector.contains(Point::new(10, 10)),
        ]
    }

    #[test]
    fn plane_sector_quadrants_positive_sweep() {
        let plane_sector = PlaneSector::new(0.0.deg(), 90.0.deg());
        assert_eq!(
            contains(&plane_sector),
            [true, true, true, false, false, false, false, false]
        );

        let plane_sector = PlaneSector::new(90.0.deg(), 90.0.deg());
        assert_eq!(
            contains(&plane_sector),
            [false, false, true, true, true, false, false, false]
        );

        let plane_sector = PlaneSector::new(180.0.deg(), 90.0.deg());
        assert_eq!(
            contains(&plane_sector),
            [false, false, false, false, true, true, true, false]
        );

        let plane_sector = PlaneSector::new(270.0.deg(), 90.0.deg());
        assert_eq!(
            contains(&plane_sector),
            [true, false, false, false, false, false, true, true]
        );
    }

    #[test]
    fn plane_sector_quadrants_negative_sweep() {
        let plane_sector = PlaneSector::new(0.0.deg(), -90.0.deg());
        assert_eq!(
            contains(&plane_sector),
            [true, false, false, false, false, false, true, true]
        );

        let plane_sector = PlaneSector::new(90.0.deg(), -90.0.deg());
        assert_eq!(
            contains(&plane_sector),
            [true, true, true, false, false, false, false, false]
        );

        let plane_sector = PlaneSector::new(180.0.deg(), -90.0.deg());
        assert_eq!(
            contains(&plane_sector),
            [false, false, true, true, true, false, false, false]
        );

        let plane_sector = PlaneSector::new(270.0.deg(), -90.0.deg());
        assert_eq!(
            contains(&plane_sector),
            [false, false, false, false, true, true, true, false]
        );
    }

    fn test_point_type(
        stroke_width: i32,
        diameter: i32,
        sweep_step: f32,
        expected_types: &[Option<PointType>],
    ) {
        assert_eq!(diameter as usize, expected_types.len());

        let threshold = stroke_width * NORMAL_VECTOR_SCALE;

        let center = Point::new(0, diameter - 1);

        for sweep in &[sweep_step, sweep_step * 2.0, sweep_step * 3.0] {
            let plane_sector = PlaneSector::new(0.0.deg(), sweep.deg());

            let line = Line::new(Point::new(100, 0), Point::new(100, diameter - 1));

            let types = line
                .points()
                .map(|point| {
                    let delta = point * 2 - center;
                    plane_sector.point_type(delta, threshold, -threshold)
                })
                .collect::<ArrayVec<[_; 32]>>();

            assert_eq!(&types, expected_types, "sweep: {}", sweep,);
        }
    }

    #[test]
    fn point_type_1px_odd_diameter() {
        let mut expected = [
            Some(PointType::Fill),
            Some(PointType::Fill),
            Some(PointType::Stroke),
            None,
            None,
        ];

        test_point_type(1, 5, 90.0, &expected);

        expected.reverse();
        test_point_type(1, 5, -90.0, &expected);
    }

    #[test]
    fn point_type_1px_even_diameter() {
        let mut expected = [
            Some(PointType::Fill),
            Some(PointType::Fill),
            Some(PointType::Fill),
            Some(PointType::Stroke),
            None,
            None,
        ];

        test_point_type(1, 6, 90.0, &expected);

        expected.reverse();
        test_point_type(1, 6, -90.0, &expected);
    }

    #[test]
    fn point_type_2px_odd_diameter() {
        let mut expected = [
            Some(PointType::Fill),
            Some(PointType::Fill),
            Some(PointType::Stroke),
            Some(PointType::Stroke),
            None,
        ];

        test_point_type(2, 5, 90.0, &expected);

        expected.reverse();
        test_point_type(2, 5, -90.0, &expected);
    }

    #[test]
    fn point_type_2px_even_diameter() {
        let mut expected = [
            Some(PointType::Fill),
            Some(PointType::Fill),
            Some(PointType::Stroke),
            Some(PointType::Stroke),
            None,
            None,
        ];

        test_point_type(2, 6, 90.0, &expected);

        expected.reverse();
        test_point_type(2, 6, -90.0, &expected);
    }
}
