const string left_color_name = "left_color";
const string mid_color_name = "mid_color";
const string right_color_name = "right_color";
const string right_motor_name = "rmotor";
const string left_motor_name = "lmotor";
const string back_right_motor_name = "backrightmotor";
const string back_left_motor_name = "backleftmotor";
const string ultraFront = "ultraF";
const double root_delay = 50;

// editable variables ////////////////////////
double initial_basespeed = 200;
double initial_baseforce = 500;
double initial_turnspeed = 200;

double green_margin = 10;
double blue_margin = 6;

// dynamic variables ////////////////////////
double leftspeed;
double rightspeed;
double basespeed;
double baseforce;
double turnspeed;

async Task Main() {
    // IO.Print();
    // IO.PrintLine();
    // IO.ClearPrint();
    // IO.Write();
    // IO.WriteLine();
    // IO.ClearWrite();
    // await Time.Delay(root_delay);
    leftspeed = basespeed;
    rightspeed = basespeed;
    basespeed = initial_basespeed;
    baseforce = initial_baseforce;
    turnspeed = initial_turnspeed;
    //IO.ClearWrite();
    await stop();

    bool debug_mode = false; // debug mode
    if (debug_mode) {
        /////////////
        await moveFrontalAngles(-basespeed, -90);

        await debug("over");
    }
    

    while(!isRescue()) {
        await Time.Delay(root_delay);
        await MainProcess();
    }
    await RescueProcess();
}

async Task MainProcess() {
    await followLine();
    bool has_left_green = ((leftColor().Green-green_margin)>leftColor().Red) && ((leftColor().Green-green_margin)>leftColor().Blue);
    bool has_right_green = ((rightColor().Green-green_margin)>rightColor().Red) && ((rightColor().Green-green_margin)>rightColor().Blue);
    bool has_obstacle = frontDistance()<=0.7;
    bool possible_right_crossing = (readLine()=="0 1 1");
    bool possible_left_crossing = (readLine()=="1 1 0");
    bool absolute_crossing = (readLine()=="1 1 1");
    bool up_ramp = Bot.Inclination>=30 && Bot.Inclination<=350;

    if (up_ramp) {
        IO.PrintLine($"up_ramp: {Bot.Inclination}");
        basespeed = 400;
        return;
    }
    basespeed = initial_basespeed;

    if (has_obstacle) {
        IO.PrintLine("obstacle ahead");
        await stop();
        double c = -1;
        double back_rotations = 0.2;
        double side_rotations = 0.7;
        double front_rotations = 0.4;
        double last_rotations = 0.2;
        await moveFrontalRotations(-basespeed, -back_rotations);
        await moveFrontalAngles(basespeed, 85*c);
        await moveFrontalRotations(500, side_rotations);

        if (c>0) {
            await stop();
            lockLeft(true);
            lockRight(false);
            applyRight(baseforce, 500);
        } else {
            await stop();
            lockRight(true);
            lockLeft(false);
            applyLeft(baseforce, 500);
        }

        await Time.Delay(2.5*1000);
        await moveFrontalRotations(500, front_rotations);

        if (c>0) {
            await stop();
            lockLeft(true);
            lockRight(false);
            applyRight(baseforce, 500);
        } else {
            await stop();
            lockRight(true);
            lockLeft(false);
            applyLeft(baseforce, 500);
        }

        IO.Print("starting last phase");
        while(isGap()) { await Time.Delay(100); }

        await moveFrontalRotations(basespeed, last_rotations);
        await moveFrontalAngles(basespeed*c, 90*c);

        await debug("over");

        
        /*
        double c = -1;
        double turn_angle = 90;
        double back_rotations = 0.3;
        double side_rotations = 1.2;
        double front_rotations = 2;
        await moveFrontalRotations(-basespeed, -back_rotations);
        await moveFrontalAngles(turnspeed*c, turn_angle*c);
        await moveFrontalRotations(basespeed, side_rotations);
        await moveFrontalAngles(turnspeed*-c, turn_angle*-c);

        await moveFrontalRotations(basespeed, front_rotations);

        await moveFrontalAngles(turnspeed*-c, turn_angle*-c);
        await moveFrontalRotations(basespeed, side_rotations);
        await moveFrontalAngles(turnspeed*c, turn_angle*c);

        applyLeft(baseforce, -200);
        applyRight(baseforce, -200);
        double start_time = Time.Timestamp;
        double last_time = 0;
        double delta_time = 0;
        while(isGap() || delta_time>2) {
            await Time.Delay(root_delay);
            last_time = Time.Timestamp;
            delta_time = last_time - start_time;
        }
        */
        await stop();
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
        await stop();
        if (c!=0) {
            await moveFrontalRotations(basespeed, 0.7);
            await moveFrontalAngles(basespeed, 10*c);
            await SharpCurve(basespeed, c);
        } else { // 180
            await moveFrontalAngles(-500, -170);
            applyLeft(baseforce, -basespeed);
            applyRight(baseforce, -basespeed);
            while (isGap()) {await Time.Delay(50);}
        }
        await stop();
        return;
    }

    if (absolute_crossing) {
        IO.PrintLine("absolute crossing");
        await moveFrontalRotations(basespeed, 0.33);
        return;
    }

    if (possible_right_crossing || possible_left_crossing) {
        IO.PrintLine("possible crossing");
        await moveFrontalRotations(basespeed, 0.5);
        bool crossing  = !isGap();
        if (crossing) {
        } else { // 90 degrees
            double c = 1;
            if (possible_right_crossing) { c = 1; }
            if (possible_left_crossing) { c = -1; }
            IO.PrintLine($"90 graus: {c}");
            await SharpCurve(basespeed, c);
            await moveFrontalAngles(basespeed, 5*c);
        }
        return;
    }
}

async Task moveFrontalRotations(double speed, double rotations, double read_side=1, double angle_mode=0) {
    await stop();
    IO.PrintLine($"desired_rotations: {rotations}");
    Servomotor motor;
    if (read_side>=0) {
        motor = Bot.GetComponent<Servomotor>(right_motor_name);
    } else {
        motor = Bot.GetComponent<Servomotor>(left_motor_name);
    }

    lockRight(false);
    lockLeft(false);
    if (angle_mode==0) {
        applyRight(baseforce, speed);
        applyLeft(baseforce, speed);
    } else if (angle_mode>0) {
        applyLeft(baseforce, Math.Abs(speed));
        applyRight(baseforce, -Math.Abs(speed));
        //motor = Bot.GetComponent<Servomotor>(left_motor_name);
        motor = Bot.GetComponent<Servomotor>(back_left_motor_name);
        
    } else if (angle_mode<0) {
        applyRight(baseforce, Math.Abs(speed));
        applyLeft(baseforce, -Math.Abs(speed));
        //motor = Bot.GetComponent<Servomotor>(right_motor_name);
        motor = Bot.GetComponent<Servomotor>(back_right_motor_name);
    }
    motor.Locked = false;
    

    rotations = Math.Abs(rotations);
    double accumulated_rotations = 0;
    double current_rotations = 0;
    double start_angle = 0;
    double current_angle = 0;
    
    while(true) {
        current_angle = motor.Angle;
        if (current_angle>=0) {
            start_angle = current_angle;
            while(current_angle>0) {
                if (speed>0) {
                    current_rotations = Math.Abs((current_angle - start_angle)/360);
                } else {
                    current_rotations = Math.Abs((start_angle - current_angle)/360);
                }
                
                IO.Print($"accumulated_rotations: {(accumulated_rotations+current_rotations)}");
                if ((accumulated_rotations+current_rotations)>=rotations) { await stop(); return; }
                await Time.Delay(50);
                current_angle = motor.Angle;
            }
        } else {
            start_angle = current_angle;
            while(current_angle<0) {
                if (speed>0) {
                    current_rotations = Math.Abs((start_angle - current_angle)/360);
                } else {
                    current_rotations = Math.Abs((current_angle - start_angle)/360);
                }
                
                IO.Print($"accumulated_rotations: {(accumulated_rotations+current_rotations)}");
                if ((accumulated_rotations+current_rotations)>=rotations) { await stop(); return; }
                await Time.Delay(50);
                current_angle = motor.Angle;
            }
        }
        accumulated_rotations += current_rotations;
        IO.Print($"accumulated_rotations: {accumulated_rotations}");
        if (accumulated_rotations>=rotations) { await stop(); return; }
        await Time.Delay(50);
    }
}



async Task moveFrontalAngles(double speed, double desired_degrees) {
    double rotations_per_degree = 0.017f / 90f;
    double side = getNumberSignal(desired_degrees);
    double desired_rotations = Math.Abs(rotations_per_degree*desired_degrees);
    double desired_speed = Math.Abs(speed);
    desired_speed = 500;
    await moveFrontalRotations(desired_speed, desired_rotations, -side, side);
}

double getNumberSignal(double number) {
    if (number>0) {
        return 1;
    } else if (number<0) {
        return -1;
    }
    return 0;
}
async Task followLine() {
    string line_status = readLine();
    //IO.Print(line_status);
    switch(line_status) {
        case "0 1 0":
            leftspeed = basespeed;
            rightspeed = basespeed;
            break;

        case "0 0 0":
            leftspeed = basespeed;
            rightspeed = basespeed;
            break;

        case "0 0 1":
            leftspeed = basespeed;
            rightspeed = -basespeed;
            break;
        case "1 0 0":
            leftspeed = -basespeed;
            rightspeed = basespeed;
            break;
    }
    lockLeft(false);
    lockRight(false);
    // applyLeft(baseforce, leftspeed);
    // applyRight(baseforce, rightspeed);
    // if (!(basespeed==leftspeed && basespeed==rightspeed)) {
    //     await Time.Delay((200*470)/basespeed);
    // }
    if (!(basespeed==leftspeed && basespeed==rightspeed)) {
        double c = Math.Abs(leftspeed)/leftspeed;
        applyLeft(baseforce, 500*c);
        applyRight(baseforce, 500*-c);
    } else {
        applyLeft(baseforce, leftspeed);
        applyRight(baseforce, rightspeed);
    }
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

Color leftColor() {
    return Bot.GetComponent<ColorSensor>(left_color_name).Analog;
}

Color midColor() {
    return Bot.GetComponent<ColorSensor>(mid_color_name).Analog;
}

Color rightColor() {
    return Bot.GetComponent<ColorSensor>(right_color_name).Analog;
}

Servomotor motorR() {
    Servomotor motor = Bot.GetComponent<Servomotor>(right_motor_name);
    return motor;
}

Servomotor motorL() {
    Servomotor motor = Bot.GetComponent<Servomotor>(left_motor_name);
    return motor;
}

Servomotor backmotorR() {
    Servomotor motor = Bot.GetComponent<Servomotor>(back_right_motor_name);
    return motor;
}

Servomotor backmotorL() {
    Servomotor motor = Bot.GetComponent<Servomotor>(back_left_motor_name);
    return motor;
}


async Task SharpCurve(double speed, double c=1) {
    await stop();
    speed = 500;
    applyRight(baseforce, speed*(-c));
    applyLeft(baseforce, speed*(c));
    while(true) {
        await Time.Delay(root_delay);
        if (c == -1 && readLine()=="1 0 0") {
            break;
        } else if (c == 1 && readLine()=="0 0 1") {
            break;
        }
        
    }
    // 9;10
    await moveFrontalAngles(speed, 20*c); // as vezes não funciona
}

async Task stop(double stop_delay=250) {
    lockRight(true);
    lockLeft(true);
    await Time.Delay(stop_delay);
    applyRight(0, 0);
    applyLeft(0, 0);
    lockRight(false);
    lockLeft(false);
    applyRight(0, 0);
    applyLeft(0, 0);
}

async Task debug(string text="debug here") {
    IO.Print(text);
    await stop();
    lockLeft(true);
    lockRight(true);
    while(true) {
        await Time.Delay(1000);
    }
}

void applyRight(double force, double speed) {
    lockRight(false);
    motorR().Apply(force, speed);
    backmotorR().Apply(force, speed);
}

void applyLeft(double force, double speed) {
    lockLeft(false);
    motorL().Apply(force, speed);
    backmotorL().Apply(force, speed);
}

void lockLeft(bool locked=true) {
    motorL().Locked = locked;
    backmotorL().Locked = locked;
}

void lockRight(bool locked=true) {
    motorR().Locked = locked;
    backmotorR().Locked = locked;
}

double frontDistance() {
    double distance = Bot.GetComponent<UltrasonicSensor>(ultraFront).Analog;
    if (distance==-1) { distance = 9999999; }
    return distance;
}

bool isGap() {
    return (readLine()=="0 0 0");
}

bool isRescue() {
    return (midColor().Blue>midColor().Red && midColor().Blue>midColor().Green);
}

async Task RescueProcess() {
    await moveFrontalRotations(400, 1.9);
    await stop();
    IO.Print("Inside Rescue Arena");
    await debug("Inside Rescue Arena");
}