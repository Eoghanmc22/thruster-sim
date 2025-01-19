use motor_math::FloatType;
use thruster_sim::heuristic::ScoreSettings;

#[derive(Clone)]
pub struct ToggleableScoreSettings {
    pub mes_linear: (bool, FloatType),
    pub mes_x_off: (bool, FloatType),
    pub mes_y_off: (bool, FloatType),
    pub mes_z_off: (bool, FloatType),

    pub mes_torque: (bool, FloatType),
    pub mes_x_rot_off: (bool, FloatType),
    pub mes_y_rot_off: (bool, FloatType),
    pub mes_z_rot_off: (bool, FloatType),

    pub avg_linear: (bool, FloatType),
    pub avg_torque: (bool, FloatType),

    pub min_linear: (bool, FloatType),
    pub min_torque: (bool, FloatType),

    pub x: (bool, FloatType),
    pub y: (bool, FloatType),
    pub z: (bool, FloatType),

    pub x_rot: (bool, FloatType),
    pub y_rot: (bool, FloatType),
    pub z_rot: (bool, FloatType),

    pub center_of_mass_loss: (bool, FloatType),
    pub center_loss: (bool, FloatType),
    pub surface_area_loss: (bool, FloatType),
    pub dimension_loss: (bool, FloatType),
    pub tube_exclusion_radius: (bool, FloatType),
    pub tube_exclusion_loss: (bool, FloatType),
    pub thruster_exclusion_radius: (bool, FloatType),
    pub thruster_exclusion_loss: (bool, FloatType),
    pub cardinality_loss: (bool, FloatType),
    pub thruster_flow_exclusion_loss: (bool, FloatType),
}

impl ToggleableScoreSettings {
    pub fn flatten(&self) -> ScoreSettings {
        ScoreSettings {
            mes_linear: if self.mes_linear.0 {
                self.mes_linear.1
            } else {
                0.0
            },
            mes_x_off: if self.mes_x_off.0 {
                self.mes_x_off.1
            } else {
                -1.0
            },
            mes_y_off: if self.mes_y_off.0 {
                self.mes_y_off.1
            } else {
                -1.0
            },
            mes_z_off: if self.mes_z_off.0 {
                self.mes_z_off.1
            } else {
                -1.0
            },
            mes_torque: if self.mes_torque.0 {
                self.mes_torque.1
            } else {
                0.0
            },
            mes_x_rot_off: if self.mes_x_rot_off.0 {
                self.mes_x_rot_off.1
            } else {
                -1.0
            },
            mes_y_rot_off: if self.mes_y_rot_off.0 {
                self.mes_y_rot_off.1
            } else {
                -1.0
            },
            mes_z_rot_off: if self.mes_z_rot_off.0 {
                self.mes_z_rot_off.1
            } else {
                -1.0
            },
            avg_linear: if self.avg_linear.0 {
                self.avg_linear.1
            } else {
                0.0
            },
            avg_torque: if self.avg_torque.0 {
                self.avg_torque.1
            } else {
                0.0
            },
            min_linear: if self.min_linear.0 {
                self.min_linear.1
            } else {
                0.0
            },
            min_torque: if self.min_torque.0 {
                self.min_torque.1
            } else {
                0.0
            },
            x: if self.x.0 { self.x.1 } else { 0.0 },
            y: if self.y.0 { self.y.1 } else { 0.0 },
            z: if self.z.0 { self.z.1 } else { 0.0 },
            x_rot: if self.x_rot.0 { self.x_rot.1 } else { 0.0 },
            y_rot: if self.y_rot.0 { self.y_rot.1 } else { 0.0 },
            z_rot: if self.z_rot.0 { self.z_rot.1 } else { 0.0 },

            center_of_mass_loss: if self.center_of_mass_loss.0 {
                self.center_of_mass_loss.1
            } else {
                0.0
            },
            center_loss: if self.center_loss.0 {
                self.center_loss.1
            } else {
                0.0
            },
            surface_area_score: if self.surface_area_loss.0 {
                self.surface_area_loss.1
            } else {
                0.0
            },
            dimension_loss: if self.dimension_loss.0 {
                self.dimension_loss.1
            } else {
                0.0
            },
            tube_exclusion_radius: if self.tube_exclusion_radius.0 {
                self.tube_exclusion_radius.1
            } else {
                0.0
            },
            tube_exclusion_loss: if self.tube_exclusion_loss.0 {
                self.tube_exclusion_loss.1
            } else {
                0.0
            },
            thruster_exclusion_radius: if self.thruster_exclusion_radius.0 {
                self.thruster_exclusion_radius.1
            } else {
                0.0
            },
            thruster_exclusion_loss: if self.thruster_exclusion_loss.0 {
                self.thruster_exclusion_loss.1
            } else {
                0.0
            },
            cardinality_loss: if self.cardinality_loss.0 {
                self.cardinality_loss.1
            } else {
                0.0
            },
            thruster_flow_exclusion_loss: if self.thruster_flow_exclusion_loss.0 {
                self.thruster_flow_exclusion_loss.1
            } else {
                0.0
            },
        }
    }
}

impl Default for ToggleableScoreSettings {
    fn default() -> Self {
        let base = ScoreSettings::default();

        Self {
            mes_linear: (true, base.mes_linear),
            mes_x_off: (true, base.mes_x_off),
            mes_y_off: (true, base.mes_y_off),
            mes_z_off: (true, base.mes_z_off),
            mes_torque: (false, base.mes_torque),
            mes_x_rot_off: (true, base.mes_x_rot_off),
            mes_y_rot_off: (true, base.mes_y_rot_off),
            mes_z_rot_off: (true, base.mes_z_rot_off),
            avg_linear: (true, base.avg_linear),
            avg_torque: (true, base.avg_torque),
            min_linear: (true, base.min_linear),
            min_torque: (true, base.min_torque),
            x: (false, base.x),
            y: (false, base.y),
            z: (false, base.z),
            x_rot: (false, base.x_rot),
            y_rot: (false, base.y_rot),
            z_rot: (false, base.z_rot),
            center_of_mass_loss: (true, base.center_of_mass_loss),
            center_loss: (true, base.center_loss),
            surface_area_loss: (true, base.surface_area_score),
            dimension_loss: (true, base.dimension_loss),
            tube_exclusion_radius: (true, base.tube_exclusion_radius),
            tube_exclusion_loss: (true, base.tube_exclusion_loss),
            thruster_exclusion_radius: (true, base.thruster_exclusion_radius),
            thruster_exclusion_loss: (true, base.thruster_exclusion_loss),
            cardinality_loss: (true, base.cardinality_loss),
            thruster_flow_exclusion_loss: (true, base.thruster_flow_exclusion_loss),
        }
    }
}
