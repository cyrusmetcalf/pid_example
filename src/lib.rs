use std::time::{Duration, SystemTime};

#[derive(Debug, PartialEq)]
pub struct PidController {
    p_coefficient: f32,
    i_coefficient: f32,
    d_coefficient: f32,
    last_runtime: Option<SystemTime>,
    last_iterm: f32,
    last_error: f32,
}

impl PidController {
    pub fn new(p_coefficient: f32, i_coefficient: f32, d_coefficient: f32) -> PidController {
        PidController {
            p_coefficient,
            i_coefficient,
            d_coefficient,
            last_runtime: None,
            last_iterm: 0.0,
            last_error: 0.0,
        }
    }

    pub fn update(&mut self, setpoint: f32, measurement: f32) -> f32 {
        let dt = self.get_time_difference();
        let error = self.error(setpoint, measurement);
        self.p_term(error) * self.p_coefficient
            + self.i_term(error, dt) * self.i_coefficient
            + self.d_term(error, dt) * self.d_coefficient
    }

    fn set_last_runtime(&mut self) {
        self.last_runtime = Some(SystemTime::now());
    }

    fn get_time_difference(&mut self) -> f32 {
        let now = SystemTime::now();
        let dt = if let Some(last) = self.last_runtime {
            now.duration_since(last).unwrap()
        } else {
            Duration::new(0, 0)
        };
        self.set_last_runtime();
        dt.as_secs_f32()
    }

    fn error(&self, setpoint: f32, measurement: f32) -> f32 {
        setpoint - measurement
    }

    fn p_term(&self, error: f32) -> f32 {
        error
    }

    fn i_term(&mut self, error: f32, delta_time: f32) -> f32 {
        let i_term = self.last_iterm + error * delta_time;
        self.last_iterm = i_term;
        self.i_coefficient * i_term
    }

    fn d_term(&mut self, error: f32, delta_time: f32) -> f32 {
        if delta_time == 0.0_f32 {
            return 0.0_f32;
        }
            
        let d_term = (error - self.last_error) / delta_time;
        self.last_error = error;
        d_term
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
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }

    #[test]
    fn can_make_new_pid_controller() {
        assert_eq!(
            PidController::new(TestPid::P, TestPid::I, TestPid::D),
            PidController {
                p_coefficient: TestPid::P,
                i_coefficient: TestPid::I,
                d_coefficient: TestPid::D,
                last_runtime: None,
                last_iterm: TestPid::ZERO,
                last_error: TestPid::ZERO,
            }
        );
    }

    #[test]
    fn can_calculate_error_zero() {
        let setpoint = 24.7_f32;
        let measurement = 24.7_f32;
        let pid = PidController::new(TestPid::P, TestPid::I, TestPid::D);
        assert!(pid.error(setpoint, measurement) < f32::EPSILON);
    }

    #[test]
    fn can_calculate_proportional_term_private() {
        let setpoint = 234.34_f32;
        let measurement = 0.0_f32;
        let expected = setpoint - measurement;

        let mut pid = PidController::new(TestPid::P,TestPid::ZERO,TestPid::ZERO);
        assert_eq!(pid.p_term(expected) ,expected);
        assert_eq!(pid.update(setpoint,measurement),expected);
    }

    #[test]
    fn can_calculate_proportional_term_public() {
        let setpoint = 234.34_f32;
        let measurement = 0.0_f32;
        let expected = setpoint - measurement;
        let mut pid = PidController::new(TestPid::P,TestPid::ZERO,TestPid::ZERO );
        assert_eq!(pid.update(setpoint,measurement),expected);
    }

    #[test]
    fn can_calculate_integral_term_private() {
        let error = 55.6_f32;
        let dt = Duration::from_millis(1000).as_secs_f32();

        let mut pid = PidController::new(TestPid::ZERO, TestPid::I, TestPid::ZERO);
        let first_error = pid.i_term(error, dt);
        assert_eq!(first_error, error * dt);

        let second_error = pid.i_term(error, dt);
        assert_eq!(second_error, first_error + error * dt);
    }

    #[test]
    fn can_calculate_integral_term_public() {
        let setpoint = 55.6_f32;
        let measurement = TestPid::ZERO;
        let error = setpoint - measurement;

        let sleep = Duration::from_millis(10);

        let mut pid = PidController::new(TestPid::ZERO, TestPid::I, TestPid::ZERO);
        let first_error = pid.update(setpoint,measurement);
        thread::sleep(sleep);
        let second_error = pid.update(setpoint,measurement);

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
        let second_run = pid.update(setpoint,measurement);
        assert_eq!(first_run, TestPid::ZERO);
        assert!(second_run < (error - first_run)/ sleep.as_secs_f32());
    }

    #[test]
    fn can_calculate_derivative_term_private() {
        let error = 32.4_f32;
        let dt = Duration::from_millis(1000).as_secs_f32();

        let mut pid = PidController::new(TestPid::ZERO, TestPid::ZERO, TestPid::D);
        assert_eq!(pid.d_term(error, TestPid::ZERO), TestPid::ZERO);

        let first_d_term = pid.d_term(error, dt);
        assert_eq!(first_d_term, error / dt);

        let second_d_term = pid.d_term(error, dt);
        assert_eq!(second_d_term, (error - first_d_term) / dt);
    }

    #[test]
    fn can_set_last_runtime_to_now() {
        let mut pid = PidController::new(TestPid::P, TestPid::I, TestPid::D);
        pid.set_last_runtime();
        assert!(pid.last_runtime.is_some())
    }

    #[test]
    fn can_detect_differences_in_time() {
        let mut pid = PidController::new(TestPid::P, TestPid::I, TestPid::D);
        // start time zero following first call to this function
        assert_eq!(pid.get_time_difference(), 0.0_f32);
        // Sleep for some time to have something worth measuring
        let sleep = Duration::from_millis(1000);
        thread::sleep(sleep);
        // Make sure the measured time difference is larger than the time we slept.
        assert!(pid.get_time_difference() > sleep.as_secs_f32());
    }
}
