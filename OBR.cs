ColorSensor RightColor = Bot.GetComponent<ColorSensor>("right_color");
ColorSensor MidColor = Bot.GetComponent<ColorSensor>("mid_color");
ColorSensor LeftColor = Bot.GetComponent<ColorSensor>("left_color");
ColorSensor ExtremeLeftColor = Bot.GetComponent<ColorSensor>("LLsensor");
ColorSensor ExtremeRightColor = Bot.GetComponent<ColorSensor>("RRsensor");

ColorSensor FrontRightColor = Bot.GetComponent<ColorSensor>("FrontRight");
ColorSensor FrontLeftColor = Bot.GetComponent<ColorSensor>("FrontLeft");

UltrasonicSensor ultraDown = Bot.GetComponent<UltrasonicSensor>("ultraDown");
UltrasonicSensor ultraMid = Bot.GetComponent<UltrasonicSensor>("ultraMid");

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
double initial_turnspeed = 400;
double superforce = 500;

// dynamic variables ////////////////////////
double leftspeed;
double rightspeed;
double basespeed;
double baseforce;
double turnspeed;
bool is_arm_up = true;
const double root_delay = 50;
bool was_gap = false;

// for PID line follower:
int error = 0, last_error = 0;
float     Kp = 100f, // 100
        Ki = 17f, // 20
        Kd = 15f; // 15

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

    public async Task stop(double time=100) {
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
        //IO.Print($"angle: {current_angle}");
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

    public async Task stop(double time=100) {
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
            await moveRight(superforce, -speed);
            await moveLeft(superforce, speed);
            await Time.Delay(root_delay);
        }
    } else if (reading_value<=larger) { // da direita pra esquerda
        if (direction==0) special = 1;
        while (Bot.Compass>(special+anti_inertia)) {
            await moveRight(superforce, speed);
            await moveLeft(superforce, -speed);
            await Time.Delay(root_delay);
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
    both.rotations_per_degree = 0.15; //0.1 //0.06
    two_hands = new MultiMotors(ref handL, ref handR, ref handL, ref handR);
    two_hands.exclusivity = true;

    IO.ClearPrint();
    bool debug_mode = false; // debug mode
    if (debug_mode) {
        while(true) {
            await Time.Delay(root_delay);
            IO.PrintLine("abrindo...");
            await openBag();
            IO.PrintLine("ABRIU");
            await Time.Delay(1000);
            IO.PrintLine("fechando...");
            await closeBag();
            IO.PrintLine("FECHOU");
            await Time.Delay(1000);
        }
        
        await debug("to nao");
    }

    await armUp();
    await handOpen();
    await Time.Delay(300);
    both.Lock(false);
    while(isGap()) {
        await seekLine();
        if (!was_gap) { break; }
        await both.together(superforce, basespeed, -10);
        await Time.Delay(root_delay);
    }

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
    bool has_left_green = (isGreen(LeftColor) || isGreen(ExtremeLeftColor)) || (isGreen(RightColor) && isGreen(ExtremeRightColor) && isGreen(MidColor));
    bool has_right_green = (isGreen(RightColor) || isGreen(ExtremeRightColor)) || (isGreen(LeftColor) && isGreen(ExtremeLeftColor) && isGreen(MidColor));
    bool has_obstacle = getDistance(ultraMid) <= 0.3;
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
        IO.Print($"up ramp: {Bot.Inclination}");
        baseforce = 500;
        basespeed = initial_basespeed*1; // 1.5
        return;
    }
    
    if (isDownRamp()) {
        IO.Print($"down_ramp: {Bot.Inclination}");
        basespeed = initial_basespeed*0.6; //0.6
        return;
    }

    basespeed = initial_basespeed;
    baseforce = initial_baseforce;

    if (has_obstacle) {
        IO.PrintLine("obstacle ahead");
        await both.stop(500);
        await alignDirection();
        double c = 1;
        double turn_angle = 70;
        double back_rotations = 2;
        double side_rotations = 5.5;
        double front_rotations = 15;
        IO.Print("going back");
        await both.together(superforce, basespeed, -back_rotations);
        await both.stop(2001);
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
        await both.stop(300);
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
            await SharpCurve(turnspeed, c);
        } else { // 180 green
            IO.PrintLine("180 green");
            await both.together(superforce, basespeed, -0.8);
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
        await both.together(superforce, basespeed, 0.6);
        await seekLine(-1, false);
        if (was_gap) {
            IO.PrintLine("fake absolute crossing, going back...");

        }
        return;
    }

    if (possible_right_crossing || possible_left_crossing) {
        IO.PrintLine("possible crossing");
        double c=1;
        if (possible_right_crossing) { c = 1; }
        if (possible_left_crossing) { c = -1; }

        await both.together(superforce, basespeed, 0.45);
        await seekLine(-c, false);
        bool crossing  = !was_gap;
        if (crossing) {
            IO.PrintLine("false curve, continuing...");
        } else { // 90 degrees
            IO.PrintLine($"90 graus: {c}");
            await SharpCurve(turnspeed, c);
            await both.together(500, basespeed, -0.15);
        }
        while (hasSomeGreen()) {
            await both.together(500, basespeed);
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

    both.Lock(false);
    await moveRight(baseforce, basespeed-PID);
    await moveLeft(baseforce, basespeed+PID);
    
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
    while(true) {
        await moveRight(superforce, speed*-c);
        await moveLeft(superforce, speed*c);
        if (c == -1 && readLine()[0]=='1') { // readLine()=="1 0 0"
            break;
        } else if (c == 1 && readLine()[4]=='1') { // readLine()=="0 0 1"
            break;
        }
        await Time.Delay(root_delay);
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
    return inclination>35 && inclination<=350;
}

bool isDownRamp() {
    double inclination = Bot.Inclination;
    return inclination>=5 && inclination<=35;
}


bool isGap() {
    bool gap = readFullLine()=="0 0 0 0 0";
    was_gap = gap;
    return gap;
}

bool isRescue() {
    Color reading = MidColor.Analog;
    return (reading.Blue>reading.Red && reading.Blue>reading.Green);
}

async Task openBag(double c=1, double rotations=1) {
    bag.Lock(false); both.Lock(true);
    const double force = 500;
    const double speed = 300;
    await bag.walk(force, speed, rotations*c, true);
    bag.Lock(true); both.Lock(false);
}

async Task closeBag(double c=-1) {
    await openBag(-1);
}

async Task grabItem() {
    await armDown();
    IO.Print("abaixou a garra");
    await Time.Delay(500);
    await handClose();
    IO.Print("fechou a mão");
    await armUp();
    await handOpen();
}

async Task armUp(double c=1) {
    const double rotations = 8;
    arm.Lock(false); both.Lock(true);
    const double force = 500;
    const double speed = 100;
    await arm.walk(force, speed, rotations*c, true);
    arm.Lock(true); both.Lock(false);
}

async Task armDown() {
    await armUp(-0.9);
}

async Task handOpen(double c=1, double rotations=0.3) {
    // two_hands.Lock(false); both.Lock(true);
    // const double force = 500;
    // const double speed = 100;
    // await two_hands.together(force, speed, rotations*-c);
    // two_hands.Lock(true); both.Lock(false);


    //Motor hand = new Motor("hand");
    //hand.Lock(false); both.Lock(true);
    //const double force = 500;
    //const double speed = 150;
    //await hand.walk(force, speed, rotations*-c);
    //hand.Lock(true); both.Lock(false);
}

async Task handClose() {
    await handOpen(-1);
}


bool isGreen(ColorSensor sensor, double margin=0.9) {
    Color reading = sensor.Analog;
    double real = (reading.Green * margin);
    return ((real>reading.Red) && (real>reading.Blue));
}

bool isBlue(ColorSensor sensor, double margin=0.9) {
    Color reading = sensor.Analog;
    double real = (reading.Blue * margin);
    return ((real>reading.Red) && (real>reading.Green));
}

bool isBox() {
    return !FrontRightColor.Digital && !FrontLeftColor.Digital;
}

async Task adjustFrontDistance(double force, double speed, double desired_distance, bool must_stop=true) {
    await both.together(force, speed);
    // while (getDistance(ultraMid)>desired_distance) await Time.Delay(root_delay);
    while (getDistance(ultraMid)>desired_distance) {
        await Time.Delay(root_delay);
        IO.Print($"distance: {getDistance(ultraMid)}");
    }
    if (must_stop) await both.stop();
}

async Task alignBack(double force=500, double speed=310) {
    await both.together(force, -speed);
    await Time.Delay(1000);
    await both.stop();
}

ulong millis(){
	return (ulong)DateTimeOffset.Now.ToUnixTimeMilliseconds();
}

async Task seekLine(double init_action=-1, bool both_sides=true, double base_time=500, double speed=400) {
    IO.PrintLine("seeking line");
    was_gap = false;
    ulong init_time;
    init_time = millis();
    await both.stop();
    // goes for one side
    while ((millis()-init_time)<base_time) {
        await Time.Delay(root_delay);
        await moveRight(superforce, speed*-init_action);
        await moveLeft(superforce, speed*init_action);
        if (!isGap()) { await both.stop(); return; }
    }
    await both.stop();

    if (!both_sides) return;

    init_time = millis();
    // goes for the other side
    while ((millis()-init_time)<base_time*0.5d) {
        await Time.Delay(root_delay);
        await moveRight(superforce, speed*init_action);
        await moveLeft(superforce, speed*-init_action);
        if (!isGap()) { await both.stop(); return; }
    }
    await both.stop();

    // goes back to initial position
    init_time = millis();
    while ((millis()-init_time)<base_time*0.35d) {
        await Time.Delay(root_delay);
        await moveRight(superforce, speed*-init_action);
        await moveLeft(superforce, speed*init_action);
    }
    was_gap = true;
    await both.stop();
}

async Task moveLeft(double force, double speed) {
    await motorL.walk(force, speed);
    await back_motorL.walk(force, speed);
}

async Task moveRight(double force, double speed) {
    await motorR.walk(force, speed);
    await back_motorR.walk(force, speed);
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
        await adjustFrontDistance(baseforce, basespeed, 10, false);
        if (isBox()) break;
        await adjustFrontDistance(baseforce, basespeed, 3);
        await both.turnDegree(baseforce, turnspeed, 70);
        await alignDirection();
    }


    await both.turnDegree(500, basespeed, 45);
    await both.together(500, basespeed, 6); //await adjustFrontDistance(baseforce, basespeed, 10);
    await both.turnDegree(baseforce, basespeed, 95);
    await alignBack();
    await both.together(baseforce, basespeed, 0.3);
    await both.stop();
    both.Lock(true);
    IO.Print("descendo braço");
    await armDown();
    IO.Print("abrindo mochila");
    await openBag();
    IO.Print("abriu");
    await closeBag();
    IO.Print("fechou");

    await debug("rescue over");
}
