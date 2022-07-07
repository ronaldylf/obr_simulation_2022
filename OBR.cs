const string left_color_name = "left_color";
const string mid_color_name = "mid_color";
const string right_color_name = "right_color";
const string right_motor_name = "rmotor";
const string left_motor_name = "lmotor";
const string back_right_motor_name = "backrightmotor";
const string back_left_motor_name = "backleftmotor";
const double root_delay = 50;



public class Motor {
    public Servomotor motor;
    public string motor_name = "";
    public bool Locked = false;
    public bool can_run = true;
    double accumulated_rotations = 0;
    double current_rotations = 0;
    double start_angle = 0;
    double current_angle = 0;


    public Motor(string name="") { // constructor
        motor = Bot.GetComponent<Servomotor>(name);
    }

    public async Task stop(double stop_delay=250) {
        Lock(true);
        await Time.Delay(stop_delay);
        Lock(false);
        walk(0, 0);
    }
    
    public void resetMotors() {
        can_run = true;
        accumulated_rotations = 0;
        current_rotations = 0;
        start_angle = 0;
        current_angle = 0;
    }

    public async Task walk(double force, double speed, double rotations=0) {
        if (rotations==0) {
            motor.Apply(force, speed);
        } else {
            Lock(false);
            start_angle = motor.Angle;
            while(can_run) {
                await Time.Delay(root_delay);
                gyrate(force, speed, rotations);
            }
            await stop(100);
            resetMotors();
        }
    }

    public void Lock(bool will_lock=false) {
        motor.Locked = will_lock;
    }

    public double getAngle() {
        return motor.Angle;
    }
    
    public void gyrate(double force, double speed, double rotations) {
        motor.Apply(force, speed);
        double possible_rotations = 0;
        rotations = Math.Abs(rotations);
        current_angle = motor.Angle;
        if (current_angle>=0) {
            if (start_angle<=0) { start_angle = current_angle; }
            if (speed>0) {
                current_rotations = Math.Abs((current_angle - start_angle)/360F);
            } else {
                current_rotations = Math.Abs((start_angle - current_angle)/360F);
            }
            possible_rotations = accumulated_rotations + current_rotations;
        } else {
            if (start_angle>0) { start_angle = current_angle; }
            if (speed>0) {
                current_rotations = Math.Abs((start_angle - current_angle)/360F);
            } else {
                current_rotations = Math.Abs((current_angle - start_angle)/360F);
            }
            possible_rotations = accumulated_rotations + current_rotations;
        }
        accumulated_rotations += current_rotations;
        can_run = accumulated_rotations < rotations;
        //IO.PrintLine($"accumulated_rotations: {accumulated_rotations}");
    }
}

Motor motorR = new Motor("rmotor");
Motor motorL = new Motor("lmotor");
Motor back_motorR = new Motor("backrightmotor");
Motor back_motorL = new Motor("backleftmotor");
Motor armR = new Motor("armRight");
Motor armL = new Motor("armLeft");
Motor handR = new Motor("handRight");
Motor handL = new Motor("handLeft");


// editable variables ////////////////////////
double initial_basespeed = 190;
double initial_baseforce = 220;
double initial_turnspeed = 200;

double green_margin = 10;
double blue_margin = 6;

// dynamic variables ////////////////////////
double leftspeed;
double rightspeed;
double basespeed;
double baseforce;
double turnspeed;
bool is_arm_up = true;

// for PID line follower:
int error = 0, last_error = 0;
float     Kp = 100.0f, // 200
        Ki = 20f, // 40
        Kd = 15f; // 1.3

float     P=0, I=0, D=0, PID=0;

async Task Main() {
    // IO.Print();
    // IO.PrintLine();
    // IO.ClearPrint();
    // IO.Write();
    // // IO.Print();
    // IO.ClearWrite();
    // await Time.Delay(root_delay);
    leftspeed = basespeed;
    rightspeed = basespeed;
    basespeed = initial_basespeed;
    baseforce = initial_baseforce;
    turnspeed = initial_turnspeed;
    IO.ClearPrint();
    await Time.Delay(1000);
    //await upArm();
    await stop_();

    bool debug_mode = true; // debug mode
    if (debug_mode) {
        motorR.Lock(true);
        motorL.Lock(true);
        back_motorL.Lock(true);
        back_motorR.Lock(true);

        await armUp();
        IO.Print("opened arm");

        await Time.Delay(1*1000);
        
        await handOpen();
        IO.Print("opened hand");
        

        await debug("", false);
        /////////////////
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

async Task armUp(double speed=100, double rotations=1.4) {
    armR.Lock(true); armL.Lock(true);
    await Time.Delay(500);
    armR.Lock(false); armL.Lock(false);

    armR.walk(500, speed, rotations);
    await armL.walk(500, speed, rotations);

    armR.Lock(true); armL.Lock(true);
}

async Task armDown(double speed=100, double rotations=3) {
    await armUp(-speed);
}

async Task handOpen(double speed=100, double rotations=1.5) {
    handR.Lock(true); handL.Lock(true);
    await Time.Delay(500);
    handR.Lock(false); handL.Lock(false);

    handR.walk(500, -speed, rotations);
    await handL.walk(500, speed, rotations);

    handR.Lock(true); handL.Lock(true);

}

async Task handClose(double speed=100, double rotations=3) {
    await handOpen(-speed);
}

async Task MainProcess() {
    await followLine();
    // add in green to read all 5 color sensors
    bool has_left_green = ((leftColor().Green-green_margin)>leftColor().Red) && ((leftColor().Green-green_margin)>leftColor().Blue);
    bool has_right_green = ((rightColor().Green-green_margin)>rightColor().Red) && ((rightColor().Green-green_margin)>rightColor().Blue);
    bool has_obstacle = frontDistance()<=0.3; //0.7
    bool possible_right_crossing = (readLine()=="0 1 1");
    bool possible_left_crossing = (readLine()=="1 1 0");
    bool absolute_crossing = (readLine()=="1 1 1");
    

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
        float turn_angle = 65F;
        float back_rotations = 0.7F;
        float side_rotations = 1.5F;
        float front_rotations = 0.9F;
        await moveFrontalRotations(-basespeed, -back_rotations);
        await moveFrontalAngles(turnspeed, turn_angle*c);
        await moveFrontalRotations(basespeed, side_rotations);
        await moveFrontalAngles(turnspeed, turn_angle*-c);

        await moveFrontalRotations(basespeed, front_rotations);

        await moveFrontalAngles(turnspeed, turn_angle*-c);
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
        motor = Bot.GetComponent<Servomotor>(right_motor_name);
    } else {
        motor = Bot.GetComponent<Servomotor>(left_motor_name);
    }

    if (angle_mode==0) {
        applyRight(baseforce, speed);
        applyLeft(baseforce, speed);
    } else if (angle_mode>0) {
        applyLeft(baseforce, Math.Abs(speed));
        applyRight(baseforce, -Math.Abs(speed));
        motor = Bot.GetComponent<Servomotor>(left_motor_name);
        //motor = Bot.GetComponent<Servomotor>(back_left_motor_name);
        
    } else if (angle_mode<0) {
        applyRight(baseforce, Math.Abs(speed));
        applyLeft(baseforce, -Math.Abs(speed));
        motor = Bot.GetComponent<Servomotor>(right_motor_name);
        //motor = Bot.GetComponent<Servomotor>(back_right_motor_name);
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
    bool left_line = !Bot.GetComponent<ColorSensor>(left_color_name).Digital;
    bool mid_line = !Bot.GetComponent<ColorSensor>(mid_color_name).Digital;
    bool right_line = !Bot.GetComponent<ColorSensor>(right_color_name).Digital;
    string line_status = $"{left_line} {mid_line} {right_line}"; // 1(black) 0(white)
    line_status = line_status.Replace("False", "0");
    line_status = line_status.Replace("True", "1");
    return line_status;
}

string readFullLine() {
    bool left_line = !Bot.GetComponent<ColorSensor>(left_color_name).Digital;
    bool mid_line = !Bot.GetComponent<ColorSensor>(mid_color_name).Digital;
    bool right_line = !Bot.GetComponent<ColorSensor>(right_color_name).Digital;

    bool extreme_right_line = !Bot.GetComponent<ColorSensor>("RRsensor").Digital;
    bool extreme_left_line = !Bot.GetComponent<ColorSensor>("LLsensor").Digital;

    string line_status = $"{extreme_left_line} {left_line} {mid_line} {right_line} {extreme_right_line}"; // 1(black) 0(white)
    line_status = line_status.Replace("False", "0");
    line_status = line_status.Replace("True", "1");
    return line_status;
}


Color leftColor() {
    return Bot.GetComponent<ColorSensor>(left_color_name).Analog;
}

Color midColor() {
    return Bot.GetComponent<ColorSensor>(mid_color_name).Analog;
}

Color rightColor() {
    return Bot.GetComponent<ColorSensor>(right_color_name).Analog;
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
    //await moveFrontalAngles(speed, 20*new_c);
    //await debug(readLine());
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
    motorR.motor.Apply(force, speed);
    back_motorR.motor.Apply(force, speed);
}

void applyLeft(double force, double speed) {
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

double upDistance() {
    double distance = Bot.GetComponent<UltrasonicSensor>("upUltraF").Analog;
    if (distance==-1) { distance = 9999999; }
    return distance;
}

double downDistance() {
    double distance = Bot.GetComponent<UltrasonicSensor>("downUltraF").Analog;
    if (distance==-1) { distance = 9999999; }
    return distance;
}

double frontDistance() {
    if (is_arm_up) { return downDistance(); } else { return upDistance(); }
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
    return (midColor().Blue>midColor().Red && midColor().Blue>midColor().Green);
}

// put functions of arm here

async Task openDoor() {
    Servomotor servo = Bot.GetComponent<Servomotor>("door");
    servo.Locked = false;
    servo.Apply(baseforce, 130);
    await Time.Delay(1*1000);
    servo.Locked = true;
    is_arm_up = true;
}

async Task closeDoor() {
    Servomotor servo = Bot.GetComponent<Servomotor>("door");
    servo.Locked = false;
    servo.Apply(baseforce, -130);
    await Time.Delay(2*1000);
    servo.Locked = true;
    is_arm_up = true;
}

bool isBox() {
    string color = Bot.GetComponent<ColorSensor>("verifyBox").Analog.ToString();
    return color=="Preto";
}

async Task adjustFrontDistance(double speed, double desired_distance) {
    await stop_();
    while (frontDistance()>desired_distance) {
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
        if (frontDistance()<=10) {
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