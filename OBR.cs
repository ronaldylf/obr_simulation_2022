ColorSensor RightColor = Bot.GetComponent<ColorSensor>("right_color");
ColorSensor MidColor = Bot.GetComponent<ColorSensor>("mid_color");
ColorSensor LeftColor = Bot.GetComponent<ColorSensor>("left_color");
ColorSensor ExtremeLeftColor = Bot.GetComponent<ColorSensor>("LLsensor");
ColorSensor ExtremeRightColor = Bot.GetComponent<ColorSensor>("RRsensor");

ColorSensor FrontRightColor = Bot.GetComponent<ColorSensor>("FrontRight");
ColorSensor FrontLeftColor = Bot.GetComponent<ColorSensor>("FrontLeft");


UltrasonicSensor ultraDown = Bot.GetComponent<UltrasonicSensor>("ultraDown");

Motor motorR = new Motor("rmotor");
Motor motorL = new Motor("lmotor");
Motor back_motorR = new Motor("backrightmotor");
Motor back_motorL = new Motor("backleftmotor");
MultiMotors both;

Motor bag = new Motor("bag");
Motor arm = new Motor("arm");
Motor handR = new Motor("handRight");
Motor handL = new Motor("handLeft");
MultiMotors two_hands;

TouchSensor touchR = Bot.GetComponent<TouchSensor>("touchRight");
TouchSensor touchM = Bot.GetComponent<TouchSensor>("touchMid");
TouchSensor touchL = Bot.GetComponent<TouchSensor>("touchLeft");


// editable variables ////////////////////////
double initial_basespeed = 190;
double initial_baseforce = 500; // 220
double initial_turnspeed = 200;
double superforce = 500;

// dynamic variables ////////////////////////
double leftspeed;
double rightspeed;
double basespeed;
double baseforce;
double turnspeed;
bool is_arm_up = true;
const double root_delay = 50;

// for PID line follower:
int error = 0, last_error = 0;
float     Kp = 100.0f, // 100
        Ki = 20f, // 20
        Kd = 15; // 15

float     P=0, I=0, D=0, PID=0;

//////////////////////////

public class Motor {
    public Servomotor motor;
    public string name;
    public bool Locked = false;
    public bool can_run = true;
    public double amount_rotations = 0;
    double current_rotations = 0;
    double start_angle = 0;
    double current_angle = 0;

    public Motor(string motor_name="") { // constructor
        motor = Bot.GetComponent<Servomotor>(motor_name);
        name = motor_name;
    }

    public async Task stop(double time=250) {
        Lock(true);
        await Time.Delay(1); // stop time
        Lock(false);
        motor.Apply(500, 0);
        await Time.Delay(time);
    }
    
    public void reset() {
        amount_rotations = 0;
        current_rotations = 0;
        start_angle = 0;
        current_angle = 0;
    }

    public async Task walk(double force, double speed, double rotations=0, bool block=false) {
        Lock(false);
        if (rotations==0) {
            motor.Apply(force, speed);
        } else {
            await stop();
            reset();
            start_angle = getAngle();
            can_run = true;
            while(can_run) {
                await Time.Delay(root_delay);
                gyrate(force, speed, rotations);
            }

            block = true;
            if (block) {
                Lock(true);
            } else {
                await stop();
            }
            reset();
        }
    }

    public void Lock(bool will_lock=true) {
        motor.Locked = will_lock;
    }

    public double getAngle() {
        return motor.Angle;
    }

    public void gyrate(double force, double speed, double rotations) {
        double side = rotations/Math.Abs(rotations);
        speed = Math.Abs(speed)*side;
        force = Math.Abs(force);
        motor.Apply(force, speed);
        rotations = Math.Abs(rotations);
        current_angle = getAngle();
        if (current_angle>=0) {
            if (start_angle<=0) { start_angle = current_angle; }
            if (speed>0) {
                current_rotations = Math.Abs((current_angle - start_angle)/360);
            } else {
                current_rotations = Math.Abs((start_angle - current_angle)/360);
            }
        } else {
            if (start_angle>0) { start_angle = current_angle; }
            if (speed>0) {
                current_rotations = Math.Abs((start_angle - current_angle)/360);
            } else {
                current_rotations = Math.Abs((current_angle - start_angle)/360);
            }
        }
        amount_rotations += current_rotations;
        can_run = amount_rotations < rotations;
        //IO.Print($"rotations:{amount_rotations} | can_run:{can_run}");
    }
}


public class MultiMotors {
    public Motor m1 = new Motor("");
    public Motor m2 = new Motor("");
    public Motor m3 = new Motor("");
    public Motor m4 = new Motor("");
    public bool exclusivity = false; /*
    se verdadeiro: cada motor gira até pelo menos um deles retornar falso
    se falso: todos os motores giram até todos não poderem mais */
    public double rotations_per_degree = 1;

    public MultiMotors(ref Motor motor1, ref Motor motor2, ref Motor motor3, ref Motor motor4) { // constructor
        m1 = motor1;
        m2 = motor2;
        m3 = motor3;
        m4 = motor4;
    }

    public void Lock(bool will_lock=true) {
        m1.Lock(will_lock);
        m2.Lock(will_lock);
        m3.Lock(will_lock);
        m4.Lock(will_lock);
    }

    public async Task stop(double time=250) {
        Lock(true);
        await Time.Delay(1); // stop time
        Lock(false);
        await together(500, 0);
        await Time.Delay(time);
    }

    void resetMotors() {
        m1.reset();
        m2.reset();
        m3.reset();
        m4.reset();
        m1.can_run = false;
        m2.can_run = false;
        m3.can_run = false;
        m4.can_run = false;
    }

    async Task resetForGyrate() {
        resetMotors();
        m1.can_run = true;
        m2.can_run = true;
        m3.can_run = true;
        m4.can_run = true;
        await stop();
    }


    public async Task together(double force, double speed1, double rot1=0, double speed2=0, double rot2=0) {
        if (rot1==0 && rot2==0 && speed2==0 && rot2==0) {
            //IO.Print("simple together");
            await m1.walk(force, speed1);
            await m2.walk(force, speed1);
            await m3.walk(force, speed1);
            await m4.walk(force, speed1);
            //IO.Print($"speed/force: {m1.motor.Velocity}/{m1.motor.Force}");
            //IO.PrintLine(m1.getAngle().ToString());
        } else {
            //IO.Print("rotations together");
            if (rot1!=0 && speed2==0 && rot2==0) {
                //IO.PrintLine("special case");
                speed2 = speed1;
                rot2 = rot1;
            }
            await resetForGyrate();
            //IO.Print($"force:{force} | speed1:{speed1} | rot1:{rot1} | speed2:{speed2} | rot2:{rot2}");
            while (m1.can_run || m2.can_run || m3.can_run || m4.can_run) {
                await Time.Delay(root_delay);
                if (exclusivity && (!m1.can_run || !m2.can_run || !m3.can_run || !m4.can_run)) {
                    break;
                } else {
                    m1.gyrate(force, speed1, rot1);
                    m3.gyrate(force, speed1, rot1);
                    m2.gyrate(force, speed2, rot2);
                    m4.gyrate(force, speed2, rot2);
                }
                //IO.Print($"{m1.getAngle()} {m2.getAngle()} {m3.getAngle()} {m4.getAngle()}");
                //IO.Print($"locked/speed: {m1.motor.Velocity}/{m1.motor.Locked}");
                //IO.Print($"{m1.motor.Locked} {m2.motor.Locked} {m3.motor.Locked} {m4.motor.Locked}");
            }
            resetMotors();
            await stop();
            if (exclusivity) Lock(true);
        }
    }

    public async Task turnDegree(double force, double speed, double degrees=0) {
        force = 500;
        double rotations = rotations_per_degree*degrees;
        rotations = rotations*(200/speed);
        await together(500, speed, rotations, speed, -rotations);
        await stop(); await Time.Delay(100);
    }
}

double getNearDirection(double value=-1, double margin=45) {
    double direction = 0;
    if (value<0) value = Bot.Compass;
    if (value >= (360-margin) || value <= (0+margin)) { direction = 0; } // norte
    if (value >= (180-margin) && value <= (180+margin)) { direction = 180; } // sul
    if (value >= (90-margin) && value <= (90+margin)) { direction = 90; } // leste
    if (value >= (270-margin) && value <= (270+margin)) { direction = 270; } // oeste
    return direction;
}

async Task alignDirection(double value=-1, double margin=45, double anti_inertia=0.5) {
    double direction = getNearDirection(value, margin);

    double smaller = direction - margin;
    double special = direction;
    double larger = direction + margin;

	if(direction==0){
        smaller = 360-margin; // 315
        special = 359; //359
        larger = margin; // 45
	}


    double reading_value = Bot.Compass;
    double speed = 150;
    IO.PrintLine($"direction: {direction} | reading_value:{reading_value}");
    if (reading_value>=smaller && reading_value<=special) { // da esquerda pra direita
        while (Bot.Compass<(special-anti_inertia)) {
            await Time.Delay(root_delay);
            await motorR.walk(superforce, -speed);
            await back_motorR.walk(superforce, -speed);
            await motorL.walk(superforce, speed);
            await back_motorL.walk(superforce, speed);
        }
    } else if (reading_value<=larger) { // da direita pra esquerda
        if (direction==0) special = 1;
        while (Bot.Compass>(special+anti_inertia)) {
            await Time.Delay(root_delay);
            await motorR.walk(superforce, speed);
            await back_motorR.walk(superforce, speed);
            await motorL.walk(superforce, -speed);
            await back_motorL.walk(superforce, -speed);
        }
    }
}

async Task Main() {
    leftspeed = basespeed;
    rightspeed = basespeed;
    basespeed = initial_basespeed;
    baseforce = initial_baseforce;
    turnspeed = initial_turnspeed;
    both = new MultiMotors(ref motorL, ref motorR, ref back_motorL, ref back_motorR);
    both.rotations_per_degree = 0.1; //0.1
    two_hands = new MultiMotors(ref handL, ref handR, ref handL, ref handR);
    two_hands.exclusivity = true;

    IO.ClearPrint();
    bool debug_mode = false; // debug mode
    if (debug_mode) {
        await both.turnDegree(500, 200, 90);
        await debug();
    }

    await Time.Delay(250);
    await armUp();
    await handOpen();
    both.Lock(false);

    while(!isRescue()) {
        await Time.Delay(root_delay);
        await MainProcess();
    }
    await RescueProcess();
    basespeed = initial_basespeed;
    baseforce = initial_baseforce;
    turnspeed = initial_turnspeed;
}

async Task MainProcess() {
    await followLine();
    bool has_left_green = isGreen(LeftColor) || isGreen(ExtremeLeftColor);
    bool has_right_green = isGreen(RightColor) || isGreen(ExtremeRightColor);
    bool has_obstacle = getDistance(ultraDown)<=0.3;
    bool has_cube = getDistance(ultraDown)<=0.3 && !has_obstacle; has_cube=false;
    bool possible_right_crossing = (readLine()=="0 1 1");
    bool possible_left_crossing = (readLine()=="1 1 0");
    bool absolute_crossing = (readLine()=="1 1 1" && !isRescue());

    if (has_cube) {
        IO.PrintLine("blue cube ahead");
        await both.together(superforce, basespeed, -0.2);
        await grabItem();
    }

    if (isUpRamp()) {
        //IO.Print($"up ramp: {Bot.Inclination}");
        baseforce = 500;
        basespeed = initial_basespeed*1.5;
        return;
    }
    
    if (isDownRamp()) {
        IO.Print($"down_ramp: {Bot.Inclination}");
        basespeed = initial_basespeed*0.6;
        return;
    }

    basespeed = initial_basespeed;
    baseforce = initial_baseforce;

    if (has_obstacle) {
        IO.PrintLine("obstacle ahead");
        await both.stop(500);
        await alignDirection();
        double c = 1;
        double turn_angle = 90;
        double back_rotations = 2;
        double side_rotations = 3.8;
        double front_rotations = 8.5;
        IO.Print("going back");
        await both.together(superforce, basespeed, -back_rotations);
        await both.stop(200);
        await both.turnDegree(superforce, turnspeed, turn_angle*c);
        await alignDirection();
        await both.together(superforce, basespeed, side_rotations);
        await both.turnDegree(superforce, turnspeed, turn_angle*-c);
        await alignDirection();

        await both.together(superforce, basespeed, front_rotations);

        await both.turnDegree(superforce, turnspeed, turn_angle*-c);
        await alignDirection();
        await both.together(superforce, basespeed, side_rotations);
        await both.turnDegree(superforce, turnspeed, turn_angle*c);
        await alignDirection();
        await both.together(superforce, -200);
        while(!touchL.Digital && !touchM.Digital && !touchR.Digital) await Time.Delay(root_delay);
        await both.stop();
        return;
    }

    if (has_left_green || has_right_green) {
        IO.PrintLine("possible green");
        double c = 0;
        if (has_left_green && has_right_green) {
            // 180 degrees
            IO.PrintLine("180 green");
            c = 0;
        } else if (has_left_green) {
            IO.PrintLine("left green");
            c = -1;
        } else if (has_right_green) {
            IO.PrintLine("right green");
            c = 1;
        }
        await both.stop();
        if (c!=0) {
            await both.together(superforce, basespeed, 1.3);
            await both.turnDegree(superforce, turnspeed, 3*c);
            await SharpCurve(basespeed, c);
        } else { // 180 green
            await both.turnDegree(superforce, 500, -150);
            await both.together(superforce, basespeed, -0.2);
        }
        while (hasSomeGreen()) {
            await both.together(500, 100);
            await Time.Delay(root_delay);
        }
        await both.stop();
        return;
    }

    if (absolute_crossing) {
        IO.PrintLine("absolute crossing");
        await both.together(superforce, basespeed, 0.33);
        return;
    }

    if (possible_right_crossing || possible_left_crossing) {
        IO.PrintLine("possible crossing");
        await both.together(superforce, basespeed, 0.25);
        await Time.Delay(100);
        bool crossing  = !isGap();
        if (crossing) {
        } else { // 90 degrees
            double c = 1;
            if (possible_right_crossing) { c = 1; }
            if (possible_left_crossing) { c = -1; }
            IO.PrintLine($"90 graus: {c}");
            await SharpCurve(basespeed, c);
        }
        while (hasSomeGreen()) {
            await both.together(500, 100);
            await Time.Delay(root_delay);
        }
        return;
    }
}


async Task followLine() {
    switch(readFullLine()) {
        case "0 0 1 0 0":
            error = 0;
            break;
        case "0 0 1 1 0":
            error = 1;
            break;
        case "0 0 0 1 0":
            error = 2;
            break;
        case "0 0 0 1 1":
            error = 3;
            break;
        case "0 0 0 0 1":
            error = 4;
            break;
        
        case "0 1 1 0 0":
            error = -1;
            break;
        
        case "0 1 0 0 0":
            error = -2;
            break;
        
        case "1 1 0 0 0":
            error = -3;
            break;
        
        case "1 0 0 0 0":
            error = -4;
            break;
    }

    const float delta_time = 50.0f/1000.0f;

    P = error * Kp;
    I += error * Ki * delta_time;
    D = (error - last_error) * Kd / delta_time;
    
    if(I > 250) I = 250;
    if(I < -250) I = -250;

    PID = P + I + D;
    //IO.Print(PID.ToString());
    IO.Print($"{motorL.motor.Locked} {motorR.motor.Locked} {back_motorL.motor.Locked} {back_motorR.motor.Locked}");

    both.Lock(false);
    await motorR.walk(baseforce, basespeed-PID);
    await back_motorR.walk(baseforce, basespeed-PID);
    await motorL.walk(baseforce, basespeed+PID);
    await back_motorL.walk(baseforce, basespeed+PID);
    
    last_error = error;
}

string readLine() {
    bool left_line = !LeftColor.Digital;
    bool mid_line = !MidColor.Digital;
    bool right_line = !RightColor.Digital;
    string line_status = $"{left_line} {mid_line} {right_line}"; // 1(black) 0(white)
    line_status = line_status.Replace("False", "0");
    line_status = line_status.Replace("True", "1");
    return line_status;
}

string readFullLine() {
    bool left_line = !LeftColor.Digital;
    bool mid_line = !MidColor.Digital;
    bool right_line = !RightColor.Digital;

    bool extreme_right_line = !ExtremeRightColor.Digital;
    bool extreme_left_line = !ExtremeLeftColor.Digital;

    string line_status = $"{extreme_left_line} {left_line} {mid_line} {right_line} {extreme_right_line}"; // 1(black) 0(white)
    line_status = line_status.Replace("False", "0");
    line_status = line_status.Replace("True", "1");
    return line_status;
}

async Task SharpCurve(double speed, double c=1) {
    await both.stop();
    await motorR.walk(superforce, speed*-c);
    await back_motorR.walk(superforce, speed*-c);
    await motorL.walk(superforce, speed*c);
    await back_motorL.walk(superforce, speed*c);
    while(true) {
        await Time.Delay(root_delay);
        if (c == -1 && readLine()=="1 0 0") {
            break;
        } else if (c == 1 && readLine()=="0 0 1") {
            break;
        }
        
    }

    while (MidColor.Digital) await Time.Delay(root_delay);
    await both.stop();
}

bool hasSomeGreen() {
    return isGreen(LeftColor) || isGreen(ExtremeLeftColor) || isGreen(MidColor) || isGreen(RightColor) || isGreen(ExtremeRightColor);
}

async Task debug(string text="debug here", bool put_text=true) {
    IO.OpenConsole();
    if (put_text) { IO.Print(text); }
    both.Lock(true);
    while(true) {
        await Time.Delay(1000);
    }
}

double getDistance(UltrasonicSensor ultra) {
    double distance = ultra.Analog;
    if (distance==-1) { distance = 9999999; }
    if (ultra.GetHashCode() == ultraDown.GetHashCode()) distance -= 0.3;
    return (distance);
}

bool isUpRamp() {
    double inclination = Bot.Inclination;
    return inclination>=30 && inclination<=350;
}

bool isDownRamp() {
    double inclination = Bot.Inclination;
    return inclination>=5 && inclination<=28;
}

bool isGap() {
    bool gap = true;
    for (int i=1; i<=10; i++) {
        gap = (readFullLine()=="0 0 0 0 0" && gap);
    }
    return gap;
}

bool isRescue() {
    Color reading = MidColor.Analog;
    return (reading.Blue>reading.Red && reading.Blue>reading.Green);
}

async Task openBag(double c=1, double time=1000) {
    double speed = 100;
    await bag.walk(500, speed*-c);
    await Time.Delay(time);
}

async Task closeBag() {
    await openBag(-1);
}

async Task grabItem() {
    await both.stop();
    await armDown();
    IO.Print("abaixou a garra");
    await Time.Delay(500);
    await handClose(1.2);
    IO.Print("fechou a mão");
    await armUp();
    await handOpen();
}

async Task armUp(double c=1, double speed=100) {
    both.Lock(true);
    await arm.walk(500, speed*c);
    await Time.Delay(2500); // 2000
    arm.Lock(true);
    await both.stop();
}

async Task armDown(double c=1) {
    await armUp(-Math.Abs(c));
}

async Task handOpen(double c=1, double speed=100) {
    double rotations = 3;
    both.Lock(true);
    await two_hands.together(500, speed, rotations*c, speed, -rotations*c);
    await both.stop();
}

async Task handClose(double c=1) {
    await handOpen(-Math.Abs(c));
}


bool isGreen(ColorSensor sensor, double margin=2) {
    Color reading = sensor.Analog;
    double real = reading.Green-margin;
    return ((real>reading.Red) && (real>reading.Blue));
}

bool isBlue(ColorSensor sensor, double margin=2) {
    Color reading = sensor.Analog;
    double real = reading.Blue-margin;
    return ((real>reading.Red) && (real>reading.Green));
}

bool isBox() {
    return !FrontRightColor.Digital && !FrontLeftColor.Digital;
}

async Task adjustFrontDistance(double force, double speed, double desired_distance, bool must_stop=true) {
    await both.together(force, speed);
    while (getDistance(ultraDown)>desired_distance) await Time.Delay(root_delay);
    if (must_stop) await both.stop();
}

async Task RescueProcess() {
    await alignDirection();
    await both.together(superforce, 180, 5.5);
    IO.Print("Inside Rescue Arena");

    baseforce = 500;
    basespeed = 150;
    turnspeed = 400;
    while(true) { // loop until find black box
        await Time.Delay(root_delay);
        await adjustFrontDistance(baseforce, basespeed, 5, false);
        if (isBox()) break;
        await adjustFrontDistance(baseforce, basespeed, 3);
        await both.turnDegree(baseforce, turnspeed, 90);
        await alignDirection();
        await debug();
    }
    await debug("rescue over");
}
