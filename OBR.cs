ColorSensor RightColor = Bot.GetComponent<ColorSensor>("right_color");
ColorSensor MidColor = Bot.GetComponent<ColorSensor>("mid_color");
ColorSensor LeftColor = Bot.GetComponent<ColorSensor>("left_color");
ColorSensor ExtremeLeftColor = Bot.GetComponent<ColorSensor>("LLsensor");
ColorSensor ExtremeRightColor = Bot.GetComponent<ColorSensor>("RRsensor");

ColorSensor FrontRightColor = Bot.GetComponent<ColorSensor>("FrontRight");
ColorSensor FrontLeftColor = Bot.GetComponent<ColorSensor>("FrontLeft");

ColorSensor BagRightColor = Bot.GetComponent<ColorSensor>("bagColorR");
ColorSensor BagLeftColor = Bot.GetComponent<ColorSensor>("bagColorL");

UltrasonicSensor ultraDown = Bot.GetComponent<UltrasonicSensor>("ultraDown");
UltrasonicSensor ultraMid = Bot.GetComponent<UltrasonicSensor>("ultraMid");

UltrasonicSensor ultraRight = Bot.GetComponent<UltrasonicSensor>("ultraRight");
UltrasonicSensor ultraLeft = Bot.GetComponent<UltrasonicSensor>("ultraLeft");

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
Motor hand = new Motor("hand");

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
float     Kp = 140f, // 120
        Ki = 5f, // 20
        Kd = 15f; // 15

float     P=0, I=0, D=0, PID=0;

//////////////////////////

public class Motor {
    public Servomotor motor;
    public string name;
    public bool Locked = false;
    public bool can_run = true;
    public double delta_angle = 0;
    public double amount_rotations = 0;
    public double amount_degrees = 0;
    public double timeout = 10000;
    bool is_counting = false;

    public Motor(string motor_name="") { // constructor
        motor = Bot.GetComponent<Servomotor>(motor_name);
        name = motor_name;
    }

    public async Task stop(double time=50) {
        Lock(true);
        await Time.Delay(time);
        await walk(500, 0);
        Lock(false);
    }
    
    public void reset() {
        delta_angle = 0;
        amount_rotations = 0;
        amount_degrees = 0;
    }

    public async Task walk(double force, double speed, double rotations=0, bool block=false) {
        Lock(false);
        if (rotations==0) {
            motor.Apply(force, speed);
        } else {
            stopCounting();
            can_run = true;
            startCounting();
            Lock(false);
            ulong init_time = millis();
            while(can_run) {
                await Time.Delay(root_delay);
                gyrate(force, speed, rotations);
                if ((millis() - init_time) >= timeout) {
                    IO.PrintLine($"estourou o timeout do motor {name}");
                    break;
                }
            }
            stopCounting();
            if (block) {
                Lock(true);
            } else {
                await stop();
            }
            reset();
        }
    }

    public async Task moveDegrees(double force, double speed, double degrees) {
        await walk(force, speed, (degrees/360d));
    }

    public void Lock(bool will_lock=true) {
        motor.Locked = will_lock;
    }

    public double getAngle() {
        return motor.Angle;
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

    public async Task startCounting() {
        //IO.PrintLine("started counting");
        is_counting = true;
        reset();
        while(is_counting) {
            await computeDeltaAngle();
            amount_degrees += delta_angle;
            amount_rotations = amount_degrees / 360;
        }
        
    }

    public void stopCounting() {
        is_counting = false;
        //IO.PrintLine("stopped counting");
    }

    public double getRotations() {
        return amount_rotations;
    }

    public void gyrate(double force, double speed, double rotations) {
        double side = rotations/Math.Abs(rotations);
        speed = Math.Abs(speed)*side;
        force = Math.Abs(force);
        Lock(false);
        motor.Apply(force, speed);
        double amount = getRotations();
        if (rotations>0) {
            can_run = (amount<rotations);
        } else {
            can_run = (amount>rotations);
        }
        //IO.Print($@" motor: {name}
        //rotations:{amount}
        //can_run:{can_run}
        //angle: {getAngle()}
        //");
    }

    ulong millis(){
        return (ulong)DateTimeOffset.Now.ToUnixTimeMilliseconds();
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

    public async Task stop(double time=50) {
        Lock(true);
        await Time.Delay(time);
        await together(500, 0);
        Lock(false);
    }

    void resetMotors() {
        m1.reset(); m1.stopCounting(); m1.can_run = false;
        m2.reset(); m2.stopCounting(); m2.can_run = false;
        m3.reset(); m3.stopCounting(); m3.can_run = false;
        m4.reset(); m4.stopCounting(); m4.can_run = false;
    }

    async Task resetForGyrate() {
        resetMotors();
        m1.can_run = true; m1.startCounting();
        m2.can_run = true; m2.startCounting();
        m3.can_run = true; m3.startCounting();
        m4.can_run = true; m4.startCounting();
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
                //IO.Print($@"
                //rot_desired: {rot1}
                //m1: {m1.getRotations()} | can_run: {m1.can_run} | angle: {m1.getAngle()}
                //m2: {m2.getRotations()} | can_run: {m2.can_run} | angle: {m2.getAngle()}
                //m3: {m3.getRotations()} | can_run: {m3.can_run} | angle: {m3.getAngle()}
                //m4: {m4.getRotations()} |can_run: {m4.can_run} | angle: {m4.getAngle()}
                //");
            }
            resetMotors();
            if (exclusivity) {
                Lock(true);
            } else {
                await stop();
            }
        }
    }

    public async Task turnDegree(double force, double speed, double degrees=0) {
        double rotations = rotations_per_degree*degrees;
        await together(force, speed, rotations, speed, -rotations);
        await stop();
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
    both.rotations_per_degree = (1d / 90d);
    two_hands = new MultiMotors(ref handL, ref handR, ref handL, ref handR);
    two_hands.exclusivity = true;

    IO.ClearPrint();
    bool debug_mode = false; // debug mode
    if (debug_mode) {
        await both.turnDegree(500, 200, -90);
        await Time.Delay(10000);
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
    bool has_obstacle = getDistance(ultraMid) <= 0.8 &&  getDistance(ultraDown) <= 2;
    bool has_cube = getDistance(ultraDown)<=0.8 && !has_obstacle && getDistance(ultraMid) >= 6;
    bool possible_right_crossing = (readLine()=="0 1 1");
    bool possible_left_crossing = (readLine()=="1 1 0");
    bool absolute_crossing = (readLine()=="1 1 1" && !isRescue() && !isOver());
    if (has_cube) {
        IO.PrintLine("blue cube ahead");
        await both.together(superforce, 50, -0.08);
        while(true){
            await Time.Delay(root_delay);
            await moveRight(superforce, -100); await moveLeft(superforce, 100);
            if (isBlue(FrontLeftColor)) break;
        }
        await both.stop();
        IO.PrintLine("found");
        await grabItem();
        return;
    }

     if (isOver()) {
        await both.together(500, 100, 0.2);
        await debug("fim do programa :)");
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
        double c = -1;
        double turn_angle = 90;
        double back_rotations = 0;
        double side_rotations = 0.8;
        double front_rotations = 1.8;
        IO.Print("going back");
        await both.together(superforce, basespeed, -back_rotations);
        await both.stop();
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
        await both.together(superforce,basespeed,0.01);
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
            await both.together(superforce, basespeed, 0.35);
            await both.turnDegree(superforce, turnspeed, 3*c);
            await SharpCurve(turnspeed, c);
        } else { // 180 green
            IO.PrintLine("180 green");
            await both.together(superforce, basespeed, -0.1);
            await both.turnDegree(superforce, 500, -180);
            await both.together(superforce, basespeed, -0.1);
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
        await both.together(superforce, basespeed, 0.4);
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

        await both.together(superforce, basespeed, 0.2);
        await seekLine(-c, false);
        bool crossing  = !was_gap;
        if (crossing) {
            IO.PrintLine("false curve, continuing...");
        } else { // 90 degrees
            IO.PrintLine($"90 graus: {c}");
            await SharpCurve(turnspeed, c);
            await both.together(500, basespeed, -0.05);
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
bool hasAllGreen() {
    return isGreen(LeftColor) && isGreen(ExtremeLeftColor) && isGreen(MidColor) && isGreen(RightColor) && isGreen(ExtremeRightColor);
}

async Task debug(string text="") {
    IO.OpenConsole();
    if (text!="") { IO.Print(text); }
    both.Lock(true);
    while(true) {
        await Time.Delay(1000);
    }
}

double getDistance(UltrasonicSensor ultra) {
    double distance = ultra.Analog;
    if (distance==-1) { distance = 999; }
    if (ultra.GetHashCode() == ultra.GetHashCode()) distance -= 0.3;
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
    return (isBlue(ExtremeLeftColor) || isBlue(LeftColor) || isBlue(MidColor) || isBlue(RightColor) || isBlue(ExtremeRightColor));
}

bool isOver() {
    return isRed(MidColor);
}

async Task openBag(double c=1, double rotations=1.5) {
    IO.PrintLine("abrindo/fechando mochila...");
    both.Lock(true); bag.Lock(false); 
    const double force = 400; //400
    const double speed = 250; //150
    await bag.walk(force, speed, rotations*c, true);
    bag.Lock(true); both.Lock(false);
    IO.PrintLine("mochila aberta/fechada");
}

async Task closeBag() {
    await openBag(-1);
}

async Task grabItem() {
    await armDown();
    await handClose();
    await armUp();
    await handOpen();
}

async Task armUp() {
    await moveArm(500, 200, 190);
}

async Task armDown() {
    await moveArm(500, 200, -190);
}

async Task handOpen() {
    await moveHand(500, 100, 50);
}

async Task handClose() {
    await moveHand(500, 100, -25);
}

async Task moveArm(double force, double speed, double angle, double timeout=10000) {
    IO.PrintLine("tentando mexer o braço...");
    both.Lock(true); arm.Lock(false); 
    arm.timeout = timeout;
    await arm.moveDegrees(force, speed, angle);
    arm.Lock(true); both.Lock(false);
    IO.PrintLine("moveu o braço");
}

async Task moveHand(double force, double speed, double angle, double timeout=1500) {
    IO.PrintLine("tentando mover a mão...");
    both.Lock(true); hand.Lock(false);
    hand.timeout = timeout;
    await hand.moveDegrees(force, speed, angle);
    hand.Lock(true); both.Lock(false);
    IO.PrintLine("moveu a mão");
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

bool isRed(ColorSensor sensor, double margin=0.9) {
    Color reading = sensor.Analog;
    double real = (reading.Red * margin);
    return ((real>reading.Blue) && (real>reading.Green));
}

bool isBox() {
    return !FrontRightColor.Digital || !FrontLeftColor.Digital;
}
bool cube_taken(){
    return (isBlue(BagRightColor) || isBlue(BagLeftColor));
}
async Task adjustFrontDistance(double force, double speed, double desired_distance, bool must_stop=true) {
    while (getDistance(ultraMid)>desired_distance) {
        await Time.Delay(root_delay);
        await both.together(force, speed);
        //IO.Print($"adjusting distance: {getDistance(ultraMid)}");
    }
    if (must_stop) await both.stop();
}

async Task alignBack(double force=500, double speed=300) {
    while(!hasTouched()) {
        await both.together(force, -speed);
        await Time.Delay(root_delay);
    }
    await both.stop();
}

bool hasTouched() {
    return (touchL.Digital || touchM.Digital || touchR.Digital);
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
    IO.Print("Inside Rescue Arena");
    baseforce = 500;
    basespeed = 200;
    turnspeed = 400;
    await alignDirection();
    await both.together(baseforce, basespeed, 1.5);
    double last_distance;
    double main_side = (getDistance(ultraRight) > getDistance(ultraLeft)) ? 1 : -1;
    IO.PrintLine($"main_side: {main_side}");
    while(true) { // loop until find black box
        await Time.Delay(root_delay);
        last_distance = getDistance(ultraMid);
        IO.PrintLine($"last_distance: {last_distance}");
        if (last_distance>900) {
            while(!hasSomeGreen()) {
                await both.together(baseforce, basespeed);
                await Time.Delay(root_delay);
            }
            await both.stop();
            await both.together(baseforce, basespeed, -0.2);
        } else {
            await adjustFrontDistance(baseforce, basespeed, 10, false);
            if(isBox()) break;
            await adjustFrontDistance(baseforce, basespeed, 4, true);
        }
        await both.turnDegree(baseforce, turnspeed, 90*main_side);
        await alignDirection();
    }

    await both.stop();
    await both.turnDegree(baseforce, basespeed, 45*main_side);
    await adjustFrontDistance(baseforce, basespeed, 12);
    await both.turnDegree(baseforce, basespeed, 90*main_side);
    await alignBack();
    await armDown();
    await openBag();
    await closeBag();
    IO.PrintLine("cubo talvez resgatado");

    await debug();
}

