ColorSensor RightColorSensor = Bot.GetComponent<ColorSensor>("right_color");
ColorSensor MidColorSensor = Bot.GetComponent<ColorSensor>("mid_color");
ColorSensor LeftColorSensor = Bot.GetComponent<ColorSensor>("left_color");
ColorSensor ExtremeLeftColorSensor = Bot.GetComponent<ColorSensor>("LLsensor");
ColorSensor ExtremeRightColorSensor = Bot.GetComponent<ColorSensor>("RRsensor");

UltrasonicSensor ultraUp = Bot.GetComponent<UltrasonicSensor>("ultraUp");
UltrasonicSensor ultraDown = Bot.GetComponent<UltrasonicSensor>("ultraDown");

Motor motorR = new Motor("rmotor");
Motor motorL = new Motor("lmotor");
Motor back_motorR = new Motor("backrightmotor");
Motor back_motorL = new Motor("backleftmotor");
MultiMotors both;

Motor armR = new Motor("armRight");
Motor armL = new Motor("armLeft");
Motor handR = new Motor("handRight");
Motor handL = new Motor("handLeft");
Motor door = new Motor("door");


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

    void stop() {
        Lock(true);
        Lock(false);
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
                stop();
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
        Lock(false);
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
    Motor m1 = new Motor("");
    Motor m2 = new Motor("");
    Motor m3 = new Motor("");
    Motor m4 = new Motor("");
    public double rotations_per_degree = 1;

    public MultiMotors(string name1="", string name2="", string name3="", string name4="") { // constructor
        m1 = new Motor(name1);
        m2 = new Motor(name2);
        m3 = new Motor(name3);
        m4 = new Motor(name4);
    }

    void Lock(bool will_lock=true) {
        m1.Lock(will_lock);
        m2.Lock(will_lock);
        m3.Lock(will_lock);
        m4.Lock(will_lock);
    }

    void stop() {
        Lock(true);
        Lock(false);
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

    void resetForGyrate() {
        resetMotors();
        m1.can_run = true;
        m2.can_run = true;
        m3.can_run = true;
        m4.can_run = true;
        stop();
    }

    
    public async Task together(double force, double speed1, double rot1=0, double speed2=0, double rot2=0) {
        if (rot1==0 && rot2==0 && speed2==0 && rot2==0) {
            //IO.Print("simple together");
            speed1 = 50;
            await m1.walk(force, speed1);
            await m2.walk(force, speed1);
            await m3.walk(force, speed1);
            await m4.walk(force, speed1);
            //IO.PrintLine(m1.getAngle().ToString());
        } else {
            IO.Print("rotations together");
            if (rot1!=0 && speed2==0 && rot2==0) {
                speed2 = speed1;
                rot2 = rot1;
            }

            //IO.Print($"force={force} | speed1={speed1} | rot1={rot1} | speed2={speed2} | rot2={rot2}");
            //IO.Print($"{m1.Locked} {m2.Locked} {m3.Locked} {m4.Locked}");
            //while(true) await Time.Delay(root_delay);
            resetForGyrate();
            while (m1.can_run || m2.can_run) {
                await Time.Delay(root_delay);
                m1.gyrate(force, speed1, rot1);
                m2.gyrate(force, speed2, rot2);
                m3.gyrate(force, speed1, rot1);
                m4.gyrate(force, speed2, rot2);
                //IO.Print($"{m1.can_run} {m2.can_run}");
                IO.Print($"{m1.amount_rotations} {m2.amount_rotations} {m3.amount_rotations} {m4.amount_rotations}");
            }

            resetMotors();
            stop();
        }
    }

    public async Task turnDegree(double force, double speed, double degrees=0) {
        double rotations = rotations_per_degree*degrees;
        await together(force, speed, rotations);
    }
}



// editable variables ////////////////////////
double initial_basespeed = 190;
double initial_baseforce = 220;
double initial_turnspeed = 200;


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
float     Kp = 100.0f, // 200
        Ki = 20f, // 40
        Kd = 15f; // 1.3

float     P=0, I=0, D=0, PID=0;

async Task Main() {
    leftspeed = basespeed;
    rightspeed = basespeed;
    basespeed = initial_basespeed;
    baseforce = initial_baseforce;
    turnspeed = initial_turnspeed;
    both = new MultiMotors(motorL.name, motorR.name, back_motorL.name, back_motorR.name);
    //both.rotations_per_degree = 0.8;
    IO.ClearPrint();

    bool debug_mode = true; // debug mode
    if (debug_mode) {
        IO.OpenConsole();
        /////
        motorR.Lock(false);
        motorL.Lock(false);
        back_motorL.Lock(false);
        back_motorR.Lock(false);

        //both.rotations_per_degree = 0.8;
        //await both.turnDegree(baseforce, basespeed, 90);
        while(true) {
            await both.together(baseforce, basespeed);
            await Time.Delay(root_delay);
        }
        await debug("", false);
        /////////////////
    }
    
    await armUp();
    await handOpen();
    await stop_();

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
    // add in green to read all 5 color sensors
    bool has_left_green = isGreen(LeftColorSensor);
    bool has_right_green = isGreen(RightColorSensor);
    bool has_obstacle = getDistance(ultraUp)<=0.3;
    bool has_cube = getDistance(ultraDown)<=0.3 && !has_obstacle;
    bool possible_right_crossing = (readLine()=="0 1 1");
    bool possible_left_crossing = (readLine()=="1 1 0");
    bool absolute_crossing = (readLine()=="1 1 1");
    

    if (has_cube) {
        IO.PrintLine("blue cube ahead");
        await moveFrontalRotations(-basespeed, -0.2f);
        await grabItem();
    }

    if (isUpRamp()) {
        IO.Print($"up_ramp: {Bot.Inclination}");
        baseforce = 500;
        basespeed = initial_basespeed*1.5;
        return;
    }
    
    if (isDownRamp()) {
        IO.Print($"down_ramp: {Bot.Inclination}");
        basespeed = initial_basespeed*0.5;
        return;
    }

    basespeed = initial_basespeed;
    baseforce = initial_baseforce;

    if (has_obstacle) {
        IO.PrintLine("obstacle ahead");
        await stop_();
        float c = -1F;
        float turn_angle = 90;
        float back_rotations = 0.4F;
        float side_rotations = 1;
        float front_rotations = 1.5F;
        await moveFrontalRotations(-basespeed, -back_rotations);
        await moveFrontalAngles(turnspeed, turn_angle*c);
        await moveFrontalRotations(basespeed, side_rotations);
        await moveFrontalAngles(turnspeed, turn_angle*-c);


        await moveFrontalRotations(basespeed, front_rotations);

        await moveFrontalAngles(turnspeed, turn_angle*-c);
        await debug();
        await moveFrontalRotations(basespeed, side_rotations);
        await moveFrontalAngles(turnspeed, turn_angle*c);

        applyLeft(baseforce, -200);
        applyRight(baseforce, -200);
        double start_time = Time.Timestamp;
        double last_time = 0;
        double delta_time = 0;
        while(readFullLine()=="0 0 0 0 0" || delta_time>2) {
            await Time.Delay(root_delay);
            last_time = Time.Timestamp;
            delta_time = last_time - start_time;
        }
        await stop_();
        return;
    }

    if (has_left_green || has_right_green) {
        IO.PrintLine("possible green");
        float c = 0F;
        if (has_left_green && has_right_green) {
            // 180 degrees
            IO.PrintLine("180 green");
            c = 0F;
        } else if (has_left_green) {
            IO.PrintLine("left green");
            c = -1F;
        } else if (has_right_green) {
            IO.PrintLine("right green");
            c = 1F;
        }
        await stop_();
        if (c!=0) {
            await moveFrontalRotations(basespeed, 0.4F);
            await moveFrontalAngles(basespeed, (float)(10*c));
            await SharpCurve(basespeed, (double)c);
            await moveFrontalRotations(basespeed, 0.2f);
        } else { // 180
            await moveFrontalAngles(-500, -170F);
            applyLeft(baseforce, -basespeed);
            applyRight(baseforce, -basespeed);
            while (isGap()) {await Time.Delay(50);}
        }
        await stop_();
        return;
    }

    if (absolute_crossing) {
        IO.PrintLine("absolute crossing");
        await moveFrontalRotations(basespeed, 0.33F);
        return;
    }

    if (possible_right_crossing || possible_left_crossing) {
        IO.PrintLine("possible crossing");
        await moveFrontalRotations(basespeed, 0.3F);
        await stop_();
        bool crossing  = !isGap() && readFullLine()!="0 0 0 0 0";
        if (crossing) {
        } else { // 90 degrees
            float c = 1;
            if (possible_right_crossing) { c = 1F; }
            if (possible_left_crossing) { c = -1F; }
            IO.PrintLine($"90 graus: {c}");
            await SharpCurve(basespeed, (double)c);
        }
        return;
    }
}

async Task moveFrontalRotations(double speed, float rotations, float read_side=1, float angle_mode=0) {
    await stop_(); // problem: robot away from ground and bug motors
    // IO.Print($"desired_rotations: {rotations}");
    //await debug($"desired_rotations: {rotations}");
    double stop_time = 350;
    Servomotor motor;
    if (read_side>=0) {
        motor = motorR.motor;
    } else {
        motor = motorL.motor;
    }

    if (angle_mode==0) {
        applyRight(baseforce, speed);
        applyLeft(baseforce, speed);
    } else if (angle_mode>0) {
        applyLeft(baseforce, Math.Abs(speed));
        applyRight(baseforce, -Math.Abs(speed));
        motor = motorL.motor;
        
    } else if (angle_mode<0) {
        applyRight(baseforce, Math.Abs(speed));
        applyLeft(baseforce, -Math.Abs(speed));
        motor = motorR.motor;
    }
    motor.Locked = false;
    

    rotations = (float)Math.Abs(rotations);
    float accumulated_rotations = 0;
    float current_rotations = 0;
    float start_angle = 0;
    float current_angle = 0;
    
    while(true) {
        current_angle = (float)motor.Angle;
        if (current_angle>=0) {
            start_angle = (float)current_angle;
            while(current_angle>0) {
                if (speed>0) {
                    current_rotations = (float)Math.Abs((current_angle - start_angle)/360F);
                } else {
                    current_rotations = (float)Math.Abs((start_angle - current_angle)/360F);
                }
                
                //IO.Print($"accumulated_rotations: {(accumulated_rotations+current_rotations)}");
                if ((accumulated_rotations+current_rotations)>=rotations) { await stop_(stop_time); return; }
                await Time.Delay(50);
                current_angle = (float)motor.Angle;
            }
        } else {
            start_angle = (float)current_angle;
            while(current_angle<0) {
                if (speed>0) {
                    current_rotations = (float)Math.Abs((start_angle - current_angle)/360F);
                } else {
                    current_rotations = (float)Math.Abs((current_angle - start_angle)/360F);
                }
                
                // IO.Print($"accumulated_rotations: {(accumulated_rotations+current_rotations)}");
                if ((accumulated_rotations+current_rotations)>=rotations) { await stop_(stop_time); return; }
                await Time.Delay(50);
                current_angle = (float)motor.Angle;
            }
        }
        accumulated_rotations += current_rotations;
        // IO.Print($"accumulated_rotations: {accumulated_rotations}");
        if (accumulated_rotations>=rotations) { await stop_(stop_time); return; }
        await Time.Delay(50);
    }
}

async Task moveFrontalAngles(double speed, float desired_degrees) {
    float rotations_per_degree = (float)(0.7F / 90F);
    float side = getNumberSignal(desired_degrees);
    float desired_rotations = Math.Abs(rotations_per_degree*desired_degrees);
    double desired_speed = Math.Abs(speed);
    desired_speed = 500;
    await moveFrontalRotations(desired_speed, desired_rotations, -side, side);
}

float getNumberSignal(float number) {
    if (number>0) {
        return 1;
    } else if (number<0) {
        return -1;
    }
    return 0;
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

    lockLeft(false);
    lockRight(false);

    const float delta_time = 50.0f/1000.0f;

    P = error * Kp;
    I += error * Ki * delta_time;
    if(I > 250) I = 250;

    D = (error - last_error) * Kd / delta_time;

    PID = P + I + D;
    IO.Print(PID.ToString());

    applyRight(baseforce, (double)basespeed-PID);
    applyLeft(baseforce, (double)basespeed+PID);
    
    last_error = error;
}

string readLine() {
    bool left_line = !LeftColorSensor.Digital;
    bool mid_line = !MidColorSensor.Digital;
    bool right_line = !RightColorSensor.Digital;
    string line_status = $"{left_line} {mid_line} {right_line}"; // 1(black) 0(white)
    line_status = line_status.Replace("False", "0");
    line_status = line_status.Replace("True", "1");
    return line_status;
}

string readFullLine() {
    bool left_line = !LeftColorSensor.Digital;
    bool mid_line = !MidColorSensor.Digital;
    bool right_line = !RightColorSensor.Digital;

    bool extreme_right_line = !ExtremeRightColorSensor.Digital;
    bool extreme_left_line = !ExtremeLeftColorSensor.Digital;

    string line_status = $"{extreme_left_line} {left_line} {mid_line} {right_line} {extreme_right_line}"; // 1(black) 0(white)
    line_status = line_status.Replace("False", "0");
    line_status = line_status.Replace("True", "1");
    return line_status;
}

async Task SharpCurve(double speed, double c=1) {
    await stop_();
    speed = 500;
    applyRight(baseforce, speed*-c);
    applyLeft(baseforce, speed*c);
    while(true) {
        await Time.Delay(root_delay);
        if (c == -1 && readLine()=="1 0 0") {
            break;
        } else if (c == 1 && readLine()=="0 0 1") {
            break;
        }
        
    }
    float new_c = (float)c;
}

async Task stop_(double stop_delay=250) {
    P = 0;
    I = 0;
    D = 0;
    last_error = 0;
    lockRight(true);
    lockLeft(true);
    await Time.Delay(stop_delay);
    lockRight(false);
    lockLeft(false);
    applyRight(0, 0);
    applyLeft(0, 0);
}

async Task debug(string text="debug here", bool put_text=true) {
    if (put_text) { IO.Print(text); }
    await stop_();
    lockLeft(true);
    lockRight(true);
    while(true) {
        await Time.Delay(1000);
    }
}

void applyRight(double force, double speed) {
    motorR.motor.Locked = false;
    back_motorR.motor.Locked = false;
    motorR.motor.Apply(force, speed);
    back_motorR.motor.Apply(force, speed);
}

void applyLeft(double force, double speed) {
    motorL.motor.Locked = false;
    back_motorL.motor.Locked = false;
    motorL.motor.Apply(force, speed);
    back_motorL.motor.Apply(force, speed);
}

void lockLeft(bool locked=true) {
    motorL.motor.Locked = locked;
    back_motorL.motor.Locked = locked;
}

void lockRight(bool locked=true) {
    motorR.motor.Locked = locked;
    back_motorR.motor.Locked = locked;
}

double getDistance(UltrasonicSensor ultra) {
    double distance = ultra.Analog;
    if (distance==-1) { distance = 9999999; }
    if (ultra.GetHashCode()==ultraUp.GetHashCode()) { distance -= 1.2; }
    return distance;
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
    return (readLine()=="0 0 0");
}

bool isRescue() {
    Color reading = MidColorSensor.Analog;
    return (reading.Blue>reading.Red && reading.Blue>reading.Green);
}

async Task openDoor(bool block=true, double speed=150, double rotations=1) {
    lockMotors();
    door.Lock(false);
    await door.walk(500, speed, rotations, block);
}

async Task closeDoor(bool block=true, double speed=150) {
    await openDoor(false, -speed);
}

async Task grabItem() {
    lockMotors();
    await armDown();
    IO.Print("abaixou a garra");

    await handClose();
    IO.Print("fechou a mão");

    // takes out item
    lockMotors();
    armR.Lock(true); armL.Lock(true); // blocks arm
    handR.Lock(true); handL.Lock(true); // blocks hand
    await Time.Delay(500);
    

    double speed = 100;
    double rotations = 1.9;
    double min_rate = 0.3;

    armR.walk(500, speed, rotations, true);
    armL.walk(500, speed, rotations, true);
    while(((armR.amount_rotations/rotations)<min_rate) && ((armL.amount_rotations/rotations)<min_rate)) { await Time.Delay(root_delay); }
    IO.Print("abrindo mão");
    await handOpen();
    IO.Print("pegou o item");
}

async Task armUp(double speed=100, double rotations=2) {
    lockMotors();
    armR.Lock(true); armL.Lock(true);
    await Time.Delay(500);
    armR.Lock(false); armL.Lock(false);

    armR.walk(500, speed, rotations, true);
    await armL.walk(500, speed, rotations, true);
}

async Task armDown(double speed=100, double rotations=1.6) {
    await armUp(-speed);
}

async Task handOpen(double speed=100, double rotations=0.7) {
    lockMotors();
    handR.Lock(true); handL.Lock(true);
    await Time.Delay(500);
    handR.Lock(false); handL.Lock(false);

    handR.walk(500, -speed, rotations, true);
    await handL.walk(500, speed, rotations, true);
}

async Task handClose(double speed=160, double rotations=1) {
    await handOpen(-speed, rotations);
}

void lockMotors() {
    motorR.Lock(true);
    motorL.Lock(true);
    back_motorL.Lock(true);
    back_motorR.Lock(true);
}

bool isGreen(ColorSensor sensor, double margin=10) {
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
    string color = Bot.GetComponent<ColorSensor>("verifyBox").Analog.ToString();
    return color=="Preto";
}

async Task adjustFrontDistance(double speed, double desired_distance) {
    await stop_();
    while (getDistance(ultraUp)>desired_distance) {
        await Time.Delay(root_delay);
        applyRight(baseforce, speed);
        applyLeft(baseforce, speed);
    }
    await stop_();
}

async Task RescueProcess() {
    float start_rotations = 1f;
    float clear_rotations = 0.3f;
    await moveFrontalRotations(180, start_rotations+clear_rotations);
    await moveFrontalRotations(-180, -clear_rotations);
    IO.Print("Inside Rescue Arena");

    while(true) { // loop until find black box
        await Time.Delay(root_delay);
        applyRight(baseforce, basespeed);
        applyLeft(baseforce, basespeed);
        ////
        if (getDistance(ultraUp)<=10) {
            await stop_();
            await moveFrontalAngles(basespeed, 45);
            bool first_confirmation = isBox();
            await moveFrontalRotations(basespeed, 0.75f);
            bool second_confirmation = isBox();

            if (first_confirmation && second_confirmation) {
                await moveFrontalAngles(basespeed, 90);
                applyLeft(baseforce, -200);
                applyRight(baseforce, -200);
                await Time.Delay(500);
                await stop_();
                IO.Print("found box");
                break;
            } else {
                await adjustFrontDistance(basespeed, 4);
                await moveFrontalAngles(basespeed, 45);
                IO.Print("box not found, continuing search");
            }
        }
    }
}
