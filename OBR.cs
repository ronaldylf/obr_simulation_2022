const string left_color_name = "left_color";
const string mid_color_name = "mid_color";
const string right_color_name = "right_color";
const string right_motor_name = "rmotor";
const string left_motor_name = "lmotor";
const double root_delay = 100;

// editable variables ////////////////////////
double initial_basespeed = 250;
double baseforce = 500;


double green_margin = 10;

// dynamic variables ////////////////////////
double leftspeed;
double rightspeed;
double basespeed;

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
    await stop();
    while(true) {
        await Time.Delay(root_delay);
        await MainProcess();
    }
}

async Task MainProcess() {
    await followLine();
    bool has_left_green = ((leftColor().Green-green_margin)>leftColor().Red) && ((leftColor().Green-green_margin)>leftColor().Blue);
    bool has_right_green = ((rightColor().Green-green_margin)>rightColor().Red) && ((rightColor().Green-green_margin)>rightColor().Blue);
    bool has_obstacle = false;
    bool possible_right_crossing = (readLine()=="0 1 1");
    bool possible_left_crossing = (readLine()=="1 1 0");
    bool absolute_crossing = (readLine()=="1 1 1");

    if (has_left_green || has_right_green) {
        IO.Print("possible green");
        await stop();
        if (has_left_green && has_right_green) {
            // 180 degrees
            await moveFrontalAngles(-basespeed, -180);
        } else if (has_left_green) {
            await SharpCurve(basespeed, -1);
        } else if (has_right_green) {
            await SharpCurve(basespeed, 1);
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
        bool crossing  = false;
        if (crossing) {
        } else { // 90 degrees
            double c = 1;
            if (possible_right_crossing) { c = 1; }
            if (possible_left_crossing) { c = -1; }
            IO.PrintLine($"90 graus: {c}");
            await moveFrontalRotations(basespeed, 0.2);
            await SharpCurve(c);
        }
        return;
    }
}

async Task moveFrontalRotations(double speed, double rotations, double read_side=1, double angle_mode=0) {
    await stop();
    Servomotor motor;
    if (read_side>=0) {
        motor = Bot.GetComponent<Servomotor>(right_motor_name);
    } else {
        motor = Bot.GetComponent<Servomotor>(left_motor_name);
    }


    rotations = Math.Abs(rotations);
    double accumulated_rotations = 0;
    double current_rotations = 0;
    double start_angle = 0;
    double current_angle = 0;
    
    if (angle_mode==0) {
        motorR().Apply(baseforce, speed);
        motorL().Apply(baseforce, speed);
    } else if (angle_mode>0) {
        motorL().Apply(baseforce, Math.Abs(speed));
        motorR().Apply(baseforce, -Math.Abs(speed));
        
    } else if (angle_mode<0) {
        motorL().Apply(baseforce, -Math.Abs(speed));
        motorR().Apply(baseforce, Math.Abs(speed));
    }
    
    while(true) {
        await Time.Delay(root_delay);
        
        //IO.Print($"accumulated_rotations: {accumulated_rotations}");
        if (accumulated_rotations>=rotations) { await stop(); return; }

        current_angle = motor.Angle;
        if (current_angle>=0) {
            start_angle = current_angle;
            while(current_angle>0) {
                await Time.Delay(root_delay);
                if (speed>0) {
                    current_rotations = current_angle/360;
                } else {
                    current_rotations = (start_angle - current_angle)/360;
                }
                
                if ((accumulated_rotations+current_rotations)>=rotations) { await stop(); return; }
                current_angle = motor.Angle;
                //IO.Print($"accumulated_rotations: {(accumulated_rotations+current_rotations)}");
            }

            accumulated_rotations += current_rotations;
        } else {
            start_angle = current_angle;
            while(current_angle<0) {
                await Time.Delay(root_delay);
                if (speed>0) {
                    current_rotations = (Math.Abs(start_angle) - Math.Abs(current_angle))/360;
                } else {
                    current_rotations = (Math.Abs(current_angle) - Math.Abs(start_angle))/360;
                }
                
                if ((accumulated_rotations+current_rotations)>=rotations) { await stop(); return; }
                current_angle = motor.Angle;
                //IO.Print($"accumulated_rotations: {(accumulated_rotations+current_rotations)}");
            }
        }
        accumulated_rotations += current_rotations;
    }
}

async Task moveFrontalAngles(double speed, double desired_degrees) {
    double rotations_per_degree = 4.25f / 90f;
    double side = getNumberSignal(desired_degrees);
    double desired_rotations = Math.Abs(rotations_per_degree*desired_degrees);
    double desired_speed = Math.Abs(speed);
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
    IO.Print($"{leftspeed} {rightspeed}");
    motorL().Locked = false;
    motorR().Locked = false;
    motorL().Apply(baseforce, leftspeed);
    motorR().Apply(baseforce, rightspeed);
    if (!(basespeed==leftspeed && basespeed==rightspeed)) {
        await Time.Delay(500);
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

async Task SharpCurve(double speed, double c=1) {
    await stop();
    motorR().Apply(baseforce, speed*(-c));
    motorL().Apply(baseforce, speed*(c));
    while(true) {
        await Time.Delay(root_delay);
        if (c == -1 && readLine()=="1 0 0") {
            break;
        } else if (c == 1 && readLine()=="0 0 1") {
            break;
        }
    }
    await moveFrontalAngles(speed, 5);
    await stop();
}

async Task stop() {
    motorR().Apply(0, 0);
    motorL().Apply(0, 0);
    motorR().Locked = true;
    motorL().Locked = true;
    await Time.Delay(500);
    motorR().Locked = false;
    motorL().Locked = false;
}

async Task debug() {
    await stop();
    while(true) {
        await Time.Delay(1000);
    }
}