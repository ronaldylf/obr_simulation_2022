Motor motor = new Motor("motorTest");
Motor motorB = new Motor("motorB");
const double root_delay = 1;

async Task Main() {
    motor.attachInterrupt();
    motorB.attachInterrupt();
    IO.ClearPrint();
    IO.OpenConsole();
    motor.Lock(false); motorB.Lock(false);
    const double force = 100;
    const double speed = 200;
    while(true) {
        await Time.Delay(root_delay);
        await motor.walk(force, speed);
        await motorB.walk(force, speed);
    }
}

public class Motor {
    public Servomotor motor;
    public string name;
    //////////////////
    public double kp = 2; //5
    public double ki = 1; //1.1
    public double kd = 0; //0.1
    public double p, i, d, pid, error, last_error, pwm=10, rpm = 0;
    public double delta_time = 50;
    public double refresh_time = 100; // 100
    public ulong last_time = 0;
    public const double maxI = 250;
    //////////////////
    public double current_angle, last_angle = 0;
    public double delta_angle=0, delta_rotations=0;
    public double[] total_degrees = {0, 0}; //[0]: RPM,   [1]; rotations
    bool is_counting;

    public Motor(string motor_name="") { // constructor
        motor = Bot.GetComponent<Servomotor>(motor_name);
        name = motor_name;
        current_angle = getAngle(); last_angle = current_angle; delta_angle = 0;
        last_time = millis();
    }

    public ulong millis(){
        return (ulong)DateTimeOffset.Now.ToUnixTimeMilliseconds();
    }

    public void Lock(bool will_lock=true) {
        motor.Locked = will_lock;
    }

    public double getAngle() {
        return motor.Angle;
    }

    public void run(double force, double speed) {
        if (speed>500) speed = 500;
        if (speed<-500) speed = -500;
        motor.Apply(force, speed);
    }

    public void reset() {
        total_degrees[1] = 0;
    }


    public void setPIDconstants(double kp_, double ki_, double kd_) {
        kp = kp_;
        ki = ki_;
        kd = kd_;
    }

    public double computePID(double input, double sp, bool must_derivate=true) {
        error = sp - input;
        p = error * kp;
        i += error * ki;
        if (must_derivate) {
            d = (error - last_error) * kd;
        } else { d=0; }
        //applyIntegralLimit();
        pid = p + i + d;
        IO.Print($@"motor: {name}
            setpoint: {sp} || input: {input} || pid(pwm): {Convert.ToInt32(pid)}
            error:{Convert.ToInt32(error)}
            p: {Convert.ToInt32(p)}
            i: {Convert.ToInt32(i)}
            d: {Convert.ToInt32(d)}
            input(rpm): {Convert.ToInt32(input)} | setpoint: {sp}
        ");
        last_error = error;
        return pid;
    }

    public void applyIntegralLimit() {
        if(i > maxI) i=maxI;
        if(i < -maxI) i=-maxI;
    }

    public async Task computeDeltaAngle() {
        double a0, a;
        a0 = getAngle();
        await Time.Delay(1);
        a = getAngle();
        double normal = a - a0;
        double case1 = (180-a0) + (180+a);
        double case2 = -((180-a) + (180+a0));
        double[] all_cases = {normal, case1, case2};
        double[] abs_cases = {Math.Abs(normal), Math.Abs(case1), Math.Abs(case2)};
        double min_value = abs_cases[0];
        for (int index_current=0; index_current<3; index_current++) {
            if (abs_cases[index_current]<=min_value) delta_angle = all_cases[index_current];
        }
    }

    public bool count_interrupt;
    public async Task attachInterrupt() {
        count_interrupt = true;
        while(count_interrupt) {
            await computeDeltaAngle();
            total_degrees[0] += delta_angle;
            if (is_counting) {
                total_degrees[1] += delta_angle;
            }
        }
    }

    public void stopInterrupt() {
        count_interrupt = false;
        IO.PrintLine("stopped interrupt");
    }

    public void startCounting() {
        is_counting = true;
        reset();
        IO.PrintLine("started counting");
    }

    public void stopCounting() {
        is_counting = false;
        reset();
        IO.PrintLine("stopped counting");
    }

    public double getRotations() {
        return (total_degrees[1] / 360d);
    }

    public void computeRPM() {
        delta_rotations = total_degrees[0] / 360d;
        rpm = delta_rotations * 60000d / delta_time;
        total_degrees[0] = 0;
    }

    public double getRPM() {
        return rpm;
    }

    public async Task computeAll(double sp) {
        delta_time = millis() - last_time;
        if (delta_time >= refresh_time) {
            computeRPM();
            pwm = computePID(motor.Velocity, sp);
            last_time = millis();
        }
    }

    public async Task walk(double force, double sp) {
        if(sp==0){
            run(force, 0);
        } else{
            await computeAll(sp);
            run(force, pwm);
        }

        //IO.Print($@"RPM:{rpm}
        //    PWM: {pwm}
        //    error: {error}
        //    velocity: {motor.Velocity}
        //    last_angle: {last_angle}
        //    current_angle: {current_angle}
        //    delta_angle:{delta_angle}"
        //    );
    }

    public async Task debug() {
        Lock();
        while(true) await Time.Delay(10000);
    }

}

ulong millis(){
    return (ulong)DateTimeOffset.Now.ToUnixTimeMilliseconds();
}

