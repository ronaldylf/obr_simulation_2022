string left_color_name = "left_color";
string mid_color_name = "mid_color";
string right_color_name = "right_color";

string right_motor_name = "rmotor";
string left_motor_name = "lmotor";

double basespeed = 100;
double root_delay = 80;

async Task Main() {
    // IO.PrintLine();
    // await Time.Delay(root_delay);
    while(true) {
        await Time.Delay(root_delay);
        MainProcess();
    }

}

void MainProcess() {
    followLine();
    bool has_left_green = (leftColor().Green>leftColor().Red) && (leftColor().Green>leftColor().Blue);
    bool has_right_green = (rightColor().Green>rightColor().Red) && (rightColor().Green>rightColor().Blue);
    bool has_obstacle = false;
    bool possible_right_crossing = (readLine()=="0 1 1");
    bool possible_left_crossing = (readLine()=="1 1 0");
    if (possible_right_crossing || possible_left_crossing) {
        bool crossing  = false;
        if (crossing) {
        } else { // 90 degrees
            IO.PrintLine("90 graus");
            double c = 1;
            if (possible_right_crossing) { c = 1; }
            if (possible_left_crossing) { c = -1; }
            SharpCurve(c);
        }
    }
}

void followLine() {
    string line_status = readLine();
    switch(line_status) {
        case "0 1 0":
            motorL().Force = basespeed;
            motorR().Force = basespeed;
            break;
        case "0 0 1":
            stop();
            motorL().Force = basespeed;
            motorR().Force = -basespeed;
            break;
        case "1 0 0":
            stop();
            motorL().Force = -basespeed;
            motorR().Force = basespeed;
            break;
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
    motor.Locked = false;
    return motor;
}

Servomotor motorL() {
    Servomotor motor = Bot.GetComponent<Servomotor>(left_motor_name);
    motor.Locked = false;
    return motor;
}

async Task SharpCurve(double c=1) {
    stop();
    motorR().Force = basespeed*(-c);
    motorL().Force = basespeed*c;
    while(true) {
        await Time.Delay(root_delay);
        if (c == -1 && readLine()=="1 0 0") {
            break;
        } else if (c == 1 && readLine()=="0 0 1") {
            break;
        }
    }
}

void stop() {
    motorL().Force = 0;
    motorR().Force = 0;
    motorR().Locked = true;
    motorL().Locked = true; 
}

async Task debug() {
    stop();
    while(true) {
        await Time.Delay(1000);
    }
}