#include <Wire.h>

#include <FaBo9Axis_MPU9250.h>


typedef struct command {
    char command_char;
    int target_amount;
    float rotation_amount;
    command* next_command;
} command_t;


// Test program for wireless operation
String commands[] = {
    "d",
    "f20",
    "l90",
    "f20",
    "l90",
    "f20",
    "l90",
    "f20",
    "r180",
    "f20",
    "r90",
    "f20",
    "r90",
    "f20",
    "r90",
    "f20"
};


// Exponential filter weight
const float filter_weight = 0.1f;

// Initialise pins
const unsigned int right_motor_forwards_pin = 5;
const unsigned int left_motor_forwards_pin = 6;
const unsigned int right_motor_backwards_pin = 9;
const unsigned int left_motor_backwards_pin = 10;
const unsigned int right_sensor_pin = 2;
const unsigned int left_sensor_pin = 3;
const unsigned int pen_pin = 4;

// Motor setup

// Wheel encoder pulse count
const unsigned int sensor_pulse_count = 80;

// Motor PWM increments for gradual acceleration - counters wheel slippage
const unsigned int motor_pwm_inc = 10;
const unsigned int motor_inc_period = 50;
unsigned int prev_motor_inc_millis = 0;
const unsigned int min_motor_pwm = 45;
const unsigned int max_motor_pwm = 255;

unsigned int right_motor_pwm = min_motor_pwm;
unsigned int left_motor_pwm = min_motor_pwm;

unsigned int right_pulse_count = 0;
unsigned int left_pulse_count = 0;

// Previous RPMs for exponential filter
float prev_right_rpm = 0.0f;
float prev_left_rpm = 0.0f;

float prev_right_velocity = 0.0f;
float prev_left_velocity = 0.0f;

// Period in millis for checking sensors
const unsigned int rpm_period = 5;
const float rpm_period_minutes = 60000.0f / rpm_period;
const float space_count_minutes = rpm_period_minutes / sensor_pulse_count;

const unsigned int velocity_period = 5;
const float velocity_period_seconds = velocity_period / 1000.0f;
unsigned int prev_rpm_millis = 0;
unsigned int prev_velocity_millis = 0;
const unsigned int gyro_period = 5;
const float gyro_period_seconds = gyro_period / 1000.0f;
unsigned int prev_gyro_millis = 0;
const unsigned int accel_period = 5;
const float accel_period_seconds = accel_period / 1000.0f;
unsigned int prev_accel_millis = 0;

// Wheel parameters (measured in millimetres to preserve precision)
const float right_radius = 23.0f;
const float left_radius = 23.0f;
// Wheel base
const float wheel_base = 152.0f;
// Turning circle distance - rotating some angle means travelling
// a distance of angle * distance (more accurately angle / 360 *
// circumference), include division here to reduce calculations
const float turn_dist = PI * wheel_base / 360.0f;

// Gyro setup

// Gyro chip handle
FaBo9Axis mpu9250;

// Accelerometer data
float axs = 0.0f;
float ays = 0.0f;
float azs = 0.0f;
float prev_axs = 0.0f;
float prev_ays = 0.0f;
float prev_azs = 0.0f;

// Gyro data
float gx = 0.0f;
float gy = 0.0f;
float gz = 0.0f;
float gxs = 0.0f;
float gys = 0.0f;
float gzs = 0.0f;
float prev_gxs = 0.0f;
float prev_gys = 0.0f;
float prev_gzs = 0.0f;

// Commands
const unsigned int max_straight_amount = 200;
const unsigned int max_rotate_amount = 20;

bool synchronised = false;
const unsigned int synchronise_period = 500;
unsigned int prev_synchronise_millis = 0;

command_t* current_command = nullptr;
float current_amount = 0.0f;
float acceleration_distance = -1;
float mid_point = 0.0f;
const float rotation_threshold = 2.0f;
const float bump_threshold = 0.5f;
bool adjusting = false;
bool ouch = false;

int command_ind = 0;


// Interrupt callbacks

void right_pulse() {
    right_pulse_count++;
}


void left_pulse() {
    left_pulse_count++;
}


/**
 * Applies an exponential filter to the given measurement.
 */
float exponential_filter(float measurement, float* prev_measurement) {
    float new_measurement = (
        filter_weight * measurement
        + (1 - filter_weight) * (*prev_measurement));
    (*prev_measurement) = new_measurement;
    return new_measurement;
}


float get_right_velocity(float rpm) {
    return right_radius * rpm * 0.10472f;
}


float get_left_velocity(float rpm) {
    return left_radius * rpm * 0.10472f;
}


bool check_period(unsigned int period, unsigned int* prev_millis) {
    unsigned int now_millis = (unsigned int) millis();
    if (now_millis - *prev_millis >= period) {
        *prev_millis = now_millis;
        return true;
    } else {
        return false;
    }
}


void get_motor_data(float* right_velocity, float* left_velocity) {
    float right_data = 0.0f;
    float left_data = 0.0f;

    if (check_period(rpm_period, &prev_rpm_millis)) {
        unsigned int right_pulses = 0;
        unsigned int left_pulses = 0;
        noInterrupts();
        right_pulses = right_pulse_count;
        left_pulses = left_pulse_count;
        right_pulse_count = 0;
        left_pulse_count = 0;
        interrupts();

        // Equation is (pulses * 60,000 / period) / space_count, since period
        // and space count are constant we can remove repeated divisions
        float right_rpm = right_pulses * space_count_minutes;
        float left_rpm = left_pulses * space_count_minutes;

        right_rpm = exponential_filter(right_rpm, &prev_right_rpm);
        left_rpm = exponential_filter(left_rpm, &prev_left_rpm);

        right_data = get_right_velocity(right_rpm);
        left_data = get_left_velocity(left_rpm);
    }

    *right_velocity = right_data;
    *left_velocity = left_data;
}


bool synchronise() {
    synchronised = true;
    return true;

    if (check_period(synchronise_period, &prev_synchronise_millis)
            && !synchronised && !Serial.available()) {
        // Send '2' (as char - Pi expects to decode this)
        Serial.write('2');
    }

    if (!synchronised && Serial.available()) {
        char response = (char) Serial.read();

        if (response == '2') {
            synchronised = true;
        }
    }

    return synchronised;
}


bool decode_command() {
    char command_char;
    String command_data;

    if (!Serial.available()) {
        // Run UMBmark test
        if (command_ind < 16) {
            command_char = commands[command_ind][0];
            command_data = commands[command_ind].substring(1);
            command_ind++;
        } else {
            return false;
        }
    } else {
        command_char = (char) Serial.read();
        // readStringUntil required to avoid delays
        command_data = Serial.readStringUntil('\n');
    }

    current_command = new command_t;
    unsigned int remaining_amount;
    bool is_straight = false;

    if (command_data.length() != 0) {
        remaining_amount = command_data.toInt();
        if (command_char == 'f'
                || command_char == 'b') {
            // Multiply by 10 to get millimetres
            remaining_amount *= 10;
            is_straight = true;
        }
    } else {
        remaining_amount = 100;
        is_straight = true;
    }

    // Partition commands into a series of smaller ones to reduce the effects
    // of gyro drift
    const unsigned int max_amount = (is_straight
        ? max_straight_amount : max_rotate_amount);
    command_t* current_part = current_command;

    while (remaining_amount > max_amount) {
        current_part->command_char = command_char;
        current_part->target_amount = max_amount;
        current_part->rotation_amount = max_amount * turn_dist;

        command_t* next_part = new command_t;
        current_part->next_command = next_part;
        current_part = next_part;
        remaining_amount -= max_amount;
    }

    current_part->command_char = command_char;
    current_part->target_amount = remaining_amount;
    current_part->rotation_amount = remaining_amount * turn_dist;
    current_part->next_command = nullptr;

    // Serial.print("command: ");
    // Serial.println(current_command->command_char);
    // Serial.print("amount: ");
    // Serial.println(current_command->target_amount);
    // Serial.print("rotation amount: ");
    // Serial.println(current_command->rotation_amount);

    return true;
}


void insert_command(char command_char, int target_amount) {
    command_t* next_command = new command_t;
    next_command->command_char = command_char;
    next_command->target_amount = target_amount;
    next_command->rotation_amount = next_command->target_amount * turn_dist;
    next_command->next_command = current_command->next_command;
    current_command->next_command = next_command;
}


void process_gyro() {
    // Gyro data is in degrees per second - translate this to degrees
    if (check_period(gyro_period, &prev_gyro_millis)) {
        // Get gyro data
        mpu9250.readGyroXYZ(&gxs, &gys, &gzs);

        gxs = exponential_filter(gxs, &prev_gxs);
        gys = exponential_filter(gys, &prev_gys);
        gzs = exponential_filter(gzs, &prev_gzs);

        if (abs(gxs) < 0.5) {
            gxs = 0;
        }
        if (abs(gys) < 0.5) {
            gys = 0;
        }
        if (abs(gzs) < 0.5) {
            gzs = 0;
        }

        gx += gxs * gyro_period_seconds;
        gy += gys * gyro_period_seconds;
        gz += gzs * gyro_period_seconds;

        // Serial.print(gx, 2);
        // Serial.print("\t");
        // Serial.print(gy, 2);
        // Serial.print("\t");
        // Serial.println(gz, 2);
    }
}


void process_accelerometer() {
    if (check_period(accel_period, &prev_accel_millis)) {
        // Get accelerometer data
        mpu9250.readAccelXYZ(&axs, &ays, &azs);

        axs = exponential_filter(axs, &prev_axs);
        ays = exponential_filter(ays, &prev_ays);
        azs = exponential_filter(azs, &prev_azs);

        // Serial.print(axs, 2);
        // Serial.print("\t");
        // Serial.print(ays, 2);
        // Serial.print("\t");
        // Serial.println(azs, 2);
    }
}


bool move_straight(
        unsigned int right_pin,
        unsigned int left_pin,
        unsigned int right_ground_pin,
        unsigned int left_ground_pin) {
    // Serial.print(current_amount);
    // Serial.print(" / ");
    // Serial.println(current_command->target_amount);
    // Base case - we have moved the desired amount, return true
    if (current_amount >= current_command->target_amount - 1) {
        return true;
    }

    analogWrite(right_pin, right_motor_pwm);
    analogWrite(left_pin, left_motor_pwm);
    // Setting the PWM pin to 0 effectively allows it to act as a "ground"
    analogWrite(right_ground_pin, 0);
    analogWrite(left_ground_pin, 0);

    // Simply average the estimates for left/right motors for now - will need
    // to add comparison to accelerometer data
    if (check_period(velocity_period, &prev_velocity_millis)) {
        float right_velocity;
        float left_velocity;
        get_motor_data(&right_velocity, &left_velocity);

        if (!adjusting) {
            current_amount += right_velocity * velocity_period_seconds;
        }
        prev_right_velocity = right_velocity;
        prev_left_velocity = left_velocity;
    }

    return false;
}


bool rotate(
        unsigned int right_pin,
        unsigned int left_pin,
        unsigned int right_ground_pin,
        unsigned int left_ground_pin) {
    // Base case - we have turned the desired amount, return true
    if (current_amount >= current_command->rotation_amount - 1) {
        return true;
    }

    right_motor_pwm = max_motor_pwm;
    left_motor_pwm = max_motor_pwm;

    analogWrite(right_pin, right_motor_pwm);
    analogWrite(left_pin, left_motor_pwm);
    // Setting the PWM pin to 0 effectively allows it to act as a "ground"
    analogWrite(right_ground_pin, 0);
    analogWrite(left_ground_pin, 0);

    // Simply average the estimates for left/right motors for now - will need
    // to add comparison to accelerometer data
    if (check_period(velocity_period, &prev_velocity_millis)) {
        float right_velocity;
        float left_velocity;
        get_motor_data(&right_velocity, &left_velocity);

        if (!adjusting) {
            current_amount += right_velocity * velocity_period_seconds;
        }
        prev_right_velocity = right_velocity;
        prev_left_velocity = left_velocity;
    }

    return false;
}


bool forward() {
    // Gyro reports that we are off course, adjust motor speeds to compensate
    if (gz > rotation_threshold) {
        right_motor_pwm = min_motor_pwm;
        left_motor_pwm = max_motor_pwm;
        adjusting = true;
    } else if (gz < -rotation_threshold) {
        right_motor_pwm = max_motor_pwm;
        left_motor_pwm = min_motor_pwm;
        adjusting = true;
    } else {
        right_motor_pwm = max_motor_pwm;
        left_motor_pwm = max_motor_pwm;
        adjusting = false;
    }

    if (axs < -bump_threshold) {
        ouch = true;
        return true;
    }

    return move_straight(
        right_motor_forwards_pin,
        left_motor_forwards_pin,
        right_motor_backwards_pin,
        left_motor_backwards_pin);
}


bool backward() {
    // Gyro reports that we are off course, adjust motor speeds to compensate
    if (gz > rotation_threshold) {
        right_motor_pwm = max_motor_pwm;
        left_motor_pwm = min_motor_pwm;
        adjusting = true;
    } else if (gz < -rotation_threshold) {
        right_motor_pwm = min_motor_pwm;
        left_motor_pwm = max_motor_pwm;
        adjusting = true;
    } else {
        right_motor_pwm = max_motor_pwm;
        left_motor_pwm = max_motor_pwm;
        adjusting = false;
    }

    if (axs > bump_threshold) {
        ouch = true;
        return true;
    }

    return move_straight(
        right_motor_backwards_pin,
        left_motor_backwards_pin,
        right_motor_forwards_pin,
        left_motor_forwards_pin);
}


bool turn_left() {
    bool result = rotate(
        right_motor_forwards_pin,
        left_motor_backwards_pin,
        right_motor_backwards_pin,
        left_motor_forwards_pin);

    // Check wheel encoder rotation estimate with gyro rotation estimate -
    // adjust rotation if necessary
    if (result) {
        float actual_rotation = abs(gz);

        if (actual_rotation
                > current_command->target_amount + rotation_threshold) {
            insert_command('r', actual_rotation - current_command->target_amount);
        } else if (actual_rotation
                < current_command->target_amount - rotation_threshold) {
            insert_command('l', current_command->target_amount - actual_rotation);
        }
    }

    return result;
}


bool turn_right() {
    bool result = rotate(
        right_motor_backwards_pin,
        left_motor_forwards_pin,
        right_motor_forwards_pin,
        left_motor_backwards_pin);

    // Check wheel encoder rotation estimate with gyro rotation estimate -
    // adjust rotation if necessary
    if (result) {
        float actual_rotation = abs(gz);

        if (actual_rotation
                > current_command->target_amount + rotation_threshold) {
            insert_command('l', actual_rotation - current_command->target_amount);
        } else if (actual_rotation
                < current_command->target_amount - rotation_threshold) {
            insert_command('r', current_command->target_amount - actual_rotation);
        }
    }

    return result;
}


bool pen_up() {
    digitalWrite(pen_pin, 1);
    return true;
}


bool pen_down() {
    digitalWrite(pen_pin, 0);
    return true;
}


void reset_command() {
    // Reset amount
    current_amount = 0.0f;
    mid_point = 0.0f;
    acceleration_distance = -1;
    // Reset gyro
    gx = 0.0f;
    gy = 0.0f;
    gz = 0.0f;
    // Reset pwms
    right_motor_pwm = min_motor_pwm;
    left_motor_pwm = min_motor_pwm;
    // Reset pins
    analogWrite(right_motor_forwards_pin, 0);
    analogWrite(left_motor_forwards_pin, 0);
    analogWrite(right_motor_backwards_pin, 0);
    analogWrite(left_motor_backwards_pin, 0);
}


void setup() {
    Serial.begin(115200);

    // Set up pins
    pinMode(right_motor_forwards_pin, OUTPUT);
    pinMode(left_motor_forwards_pin, OUTPUT);
    pinMode(right_motor_backwards_pin, OUTPUT);
    pinMode(left_motor_backwards_pin, OUTPUT);
    pinMode(pen_pin, OUTPUT);

    // Set up interrupts for wheel encoders
    attachInterrupt(
        digitalPinToInterrupt(right_sensor_pin),
        right_pulse,
        CHANGE);
    attachInterrupt(
        digitalPinToInterrupt(left_sensor_pin),
        left_pulse,
        CHANGE);

    // Initialise gyro
    mpu9250.begin();
}


void loop() {
    // Synchronisation - is the Pi available?
    if (!synchronise()) {
        return;
    }

    // Fetch and decode the command
    if (current_command == nullptr) {
        if (!decode_command()) {
            // No command - don't do anything
            return;
        }
    }

    while (current_command != nullptr) {
        // Execute the command
        bool completed = false;

        process_gyro();
        process_accelerometer();

        switch (current_command->command_char) {
        case 'f':
            // Move forwards
            completed = forward();
            break;
        case 'b':
            // Move backwards
            completed = backward();
            break;
        case 'l':
            // Turn left
            completed = turn_left();
            break;
        case 'r':
            // Turn right
            completed = turn_right();
            break;
        case 'u':
            // Pen up
            completed = pen_up();
            break;
        case 'd':
            // Pen down
            completed = pen_down();
            break;
        default:
            // Unknown command - reset the command value
            completed = true;
            break;
        }

        if (completed) {
            reset_command();
            if (ouch) {
                // Hit something while moving - alert the Pi
                current_command = nullptr;
                Serial.write('1');
                ouch = false;
            } else {
                if (current_command->next_command == nullptr) {
                    delay(1000);
                    current_command = nullptr;
                    // Send response
                    Serial.write('0');
                } else {
                    current_command = current_command->next_command;

                    // Serial.print("command: ");
                    // Serial.println(current_command->command_char);
                    // Serial.print("amount: ");
                    // Serial.println(current_command->target_amount);
                    // Serial.print("rotation amount: ");
                    // Serial.println(current_command->rotation_amount);
                }
            }
        }
    }
}
