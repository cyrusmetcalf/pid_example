use crate::pid::Pid;
use std::time::{Duration, SystemTime};

#[derive(Debug, PartialEq)]
pub struct PidController {
    pid: Pid,
    last_runtime: Option<SystemTime>,
}

impl PidController {
    pub fn new(p_coefficient: f32, i_coefficient: f32, d_coefficient: f32) -> PidController {
        PidController {
            pid: Pid::new(p_coefficient, i_coefficient, d_coefficient),
            last_runtime: None,
        }
    }

    pub fn update(&mut self, setpoint: f32, measurement: f32) -> f32 {
        let dt = self.get_time_difference();
        self.pid.update(setpoint, measurement, dt)
    }

    fn get_time_difference(&mut self) -> Duration {
        let now = SystemTime::now();
        let dt = if let Some(last) = self.last_runtime {
            now.duration_since(last).unwrap()
        } else {
            Duration::new(0, 0)
        };
        self.last_runtime = Some(now);
        dt
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread;

    struct TestPid;
    impl TestPid {
        const P: f32 = 1.0;
        const I: f32 = 1.0;
        const D: f32 = 1.0;
        const ZERO: f32 = 0.0;
    }

    #[test]
    fn can_calculate_proportional_term_public() {
        let setpoint = 234.34_f32;
        let measurement = 0.0_f32;
        let expected = setpoint - measurement;
        let mut pid = PidController::new(TestPid::P, TestPid::ZERO, TestPid::ZERO);
        assert_eq!(pid.update(setpoint, measurement), expected);
    }

    #[test]
    fn can_calculate_integral_term_public() {
        let setpoint = 55.6_f32;
        let measurement = TestPid::ZERO;
        let error = setpoint - measurement;
        let sleep = Duration::from_millis(10);
        let mut pid = PidController::new(TestPid::ZERO, TestPid::I, TestPid::ZERO);
        let first_error = pid.update(setpoint, measurement);
        thread::sleep(sleep);
        let second_error = pid.update(setpoint, measurement);

        assert_eq!(first_error, TestPid::ZERO);
        assert!(second_error > first_error + error * sleep.as_secs_f32());
    }

    #[test]
    fn can_calculate_derivative_term_public() {
        let setpoint = 32.4_f32;
        let measurement = 14.8_f32;
        let error = setpoint - measurement;
        let mut pid = PidController::new(TestPid::ZERO, TestPid::ZERO, TestPid::D);
        let first_run = pid.update(setpoint, measurement);
        let sleep = Duration::from_millis(10);
        thread::sleep(sleep);
        let second_run = pid.update(setpoint, measurement);
        assert_eq!(first_run, TestPid::ZERO);
        assert!(second_run < (error - first_run) / sleep.as_secs_f32());
    }

    #[test]
    fn can_detect_differences_in_time() {
        let mut pid = PidController::new(TestPid::P, TestPid::I, TestPid::D);
        let _ = pid.update(1.0, 2.0);
        let sleep = Duration::from_millis(1000);
        thread::sleep(sleep);
        assert_eq!(pid.get_time_difference().as_secs(), sleep.as_secs()); //< Duration::new(0,2));
    }
}
