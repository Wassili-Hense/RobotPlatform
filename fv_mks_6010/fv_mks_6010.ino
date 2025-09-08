#include <Arduino.h>
#include <HardwareSerial.h>
#include <SimpleFOC.h>

// magnetic sensor instance - SPI
#define CS PA15 // PA15 // GPIO 7 ==> PA15 ==> SPI(AS5047)
#define SPI_MISO PC11 //
#define SPI_MOSI PC12 //
#define SPI0SCK PC10 // PC10

// SHUNT SENSING
#define M0_IA _NC // Seulement 2 mesures de courant B&C, A = pas dispo lui.
#define M0_IB PC0
#define M0_IC PC1

// Odrive M0 motor pinout
#define M0_INH_A PA8
#define M0_INH_B PA9
#define M0_INH_C PA10
#define M0_INL_A PB13
#define M0_INL_B PB14
#define M0_INL_C PB15

// M1 & M2 common enable pin
#define EN_GATE PB12

//Pole pair
#define PP 14

//Temp
#define M0_TEMP PC5

// Motor instance
BLDCMotor motor = BLDCMotor(PP);
BLDCDriver6PWM driver = BLDCDriver6PWM(M0_INH_A, M0_INL_A, M0_INH_B, M0_INL_B, M0_INH_C, M0_INL_C, EN_GATE);

LowsideCurrentSense currentSense = LowsideCurrentSense(0.0005f, 10.0f, M0_IA, M0_IB, M0_IC);

MagneticSensorSPI sensor = MagneticSensorSPI(CS, 14, 0x3FFF); // // alternative constructor (chipselsect, bit_resolution, angle_read_register, )
SPIClass SPI_1(SPI_MOSI, SPI_MISO, SPI0SCK);


//▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬Commander▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬
Commander command = Commander(SerialUSB);

void doMotor(char* cmd) {
  command.motor(&motor, cmd);
}

//▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬setup▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬
void setup() {
  //▬▬▬▬▬▬▬▬▬▬▬▬USB_OTG_Serial▬▬▬▬▬▬▬▬▬▬▬▬
  SerialUSB.begin(115200);

  //▬▬▬▬▬▬▬▬▬▬▬▬magnetic_sensor_AS5047P▬▬▬▬▬▬▬▬▬▬▬▬
  sensor.init(&SPI_1);
  motor.linkSensor(&sensor); // initialise magnetic sensor hardware
  _delay(1000);

  //▬▬▬▬▬▬▬▬▬▬▬▬driver▬▬▬▬▬▬▬▬▬▬▬▬
  driver.pwm_frequency = 20000; // 20000 max STM32  // pwm frequency to be used [Hz]
  driver.voltage_power_supply = 24.0f; // power supply voltage [V]
  driver.voltage_limit = 5.2f;  // Max DC voltage allowed - default voltage_power_supply
  /*
    https://docs.simplefoc.com/bldcdriver6pwm
     dead_time = 1/pwm_frequency*dead_zone
  */
  //driver.dead_zone = 0.05f; // dead_zone [0,1] - default 0.02f - 2% // 1/20000*dead_zone=?
  driver.init();// driver init
  _delay(1000);
  motor.linkDriver(&driver); // link the motor and the driver

  motor.torque_controller = TorqueControlType::foc_current; // foc_current || dc_current || voltage
  motor.controller = MotionControlType::velocity;
  motor.foc_modulation = FOCModulationType::SinePWM;  // pwm modulation settings
  motor.modulation_centered = 1; // 1

  motor.zero_electric_angle = 1.1275f; // zero_electric_angle
  motor.sensor_direction = Direction::CW; // Cw/CCW // direction
  //▬▬▬▬▬▬▬▬▬▬▬▬Limits_motor▬▬▬▬▬▬▬▬▬▬▬▬
  motor.phase_resistance = 0.214; // [Ohms] motor phase resistance // I_max = V_dc/R
  motor.phase_inductance = 0.000124; // [H]
  motor.KV_rating = 98; // [rpm/Volt] - default not set // motor KV rating [rpm/V]
  motor.velocity_limit = 100.0f; // [rad/s]
  //motor.voltage_limit = 0.5f * driver.voltage_limit; // [Volts] // Calcul ==> 5.57[Ohms]*1.0[Amps]=5,57[Volts] // [V] - if phase resistance not defined
  motor.current_limit = 25.0f; // Current limit [Amps] - if phase resistance defined
  //▬▬▬▬▬▬▬▬▬▬▬▬current q loop PID
  motor.PID_current_q.P = 2.0f; // 3
  motor.PID_current_q.I = 200.0f; // 300
  motor.PID_current_q.D = 0.0f;
  motor.PID_current_q.output_ramp = 0.0f;
  motor.LPF_current_q.Tf = 0.0015f;  // Low pass filtering time constant

  //▬▬▬▬▬▬▬▬▬▬▬▬current d loop PID
  motor.PID_current_d.P = 2.0f; // 3
  motor.PID_current_d.I = 200.0f; // 300
  motor.PID_current_d.D = 0.0f;
  motor.PID_current_d.output_ramp = 0.0f;
  motor.LPF_current_d.Tf = 0.0015f;  // Low pass filtering time constant
  //▬▬▬▬▬▬▬▬▬▬▬▬velocity loop PID
  motor.PID_velocity.P = 1.0f;
  motor.PID_velocity.I = 10.0f;
  motor.PID_velocity.D = 0.0f;
  motor.PID_velocity.output_ramp = 400.0f;  // [Volts per second]  how many volts can your controller raise the voltage in one time unit
  motor.LPF_velocity.Tf = 0.0001f; // Low pass filtering time constant
  //▬▬▬▬▬▬▬▬▬▬▬▬angle loop PID
  // https://docs.simplefoc.com/angle_loop
  motor.P_angle.P = 5.0f; // 14.0
  motor.P_angle.I = 0.0f; // usually only P controller is enough
  motor.P_angle.D = 0.0f; // usually only P controller is enough
  // this variable is in rad/s^2 and sets the limit of acceleration
  motor.P_angle.output_ramp = 1750.0f; // 10000.0
  motor.LPF_angle.Tf = 0.01f; // 0.01  // Low pass filtering time constant
  //▬▬▬▬▬▬▬▬▬▬▬▬monitoring▬▬▬▬▬▬▬▬▬▬▬▬
  motor.useMonitoring(SerialUSB);
  //motor.useMonitoring(rtt);
  motor.monitor_variables =  _MON_TARGET | _MON_VEL | _MON_ANGLE;
  motor.monitor_downsample = 0; // default 10 // 0 = disable monitor at first - optional

  //▬▬▬▬▬▬▬▬▬▬▬▬SimpleFOCDebug▬▬▬▬▬▬▬▬▬▬▬▬
  SimpleFOCDebug::enable(&SerialUSB); // ok pour simpleFOCStudio

  //▬▬▬▬▬▬▬▬▬▬▬▬init▬▬▬▬▬▬▬▬▬▬▬▬

  motor.init(); // initialise motor

  //▬▬▬▬▬▬▬▬▬▬▬▬Current▬▬▬▬▬▬▬▬▬▬▬▬
  // https://docs.simplefoc.com/low_side_current_sense
  currentSense.linkDriver(&driver); // link the driver

  //▬▬▬▬▬▬▬▬▬▬▬▬linkCurrentSense▬▬▬▬▬▬▬▬▬▬▬▬
  // https://docs.simplefoc.com/low_side_current_sense
  if (currentSense.init()) {
    SerialUSB.println("Current sense init success!");
  }
  else {
    SerialUSB.println("Current sense init failed!");
    return;
  }
  // motor.motor_status
  // If monitoring is enabled for the motor during the initFOC the monitor will display the alignment status:
  // 0 - fail
  // 1 - success and nothing changed
  // 2 - success but pins reconfigured
  // 3 - success but gains inverted
  // 4 - success but pins reconfigured and gains inverted
  // If you are sure in your configuration and if you wish to skip the alignment procedure you can specify set the skip_align flag before calling motor.initFOC():
  //currentSense.skip_align = true; // true false // skip alignment procedure
  motor.linkCurrentSense(&currentSense);

  //▬▬▬▬▬▬▬▬▬▬▬▬initFOC▬▬▬▬▬▬▬▬▬▬▬▬
  motor.initFOC(); // init FOC

  //▬▬▬▬▬▬▬▬▬▬▬▬command▬▬▬▬▬▬▬▬▬▬▬▬
  // https://docs.simplefoc.com/commander_interface
  // add the motor to the commander interface
  command.decimal_places = 4; // default 3
  command.add('M', doMotor, "motor"); // The letter (here 'M') you will provide to the SimpleFOCStudio

  //▬▬▬▬▬▬▬▬▬▬▬▬target▬▬▬▬▬▬▬▬▬▬▬▬
  motor.target = 0;

  //▬▬▬▬▬▬▬▬▬▬▬▬CHECK_IF_ERROR▬▬▬▬▬▬▬▬▬▬▬▬
  // https://docs.simplefoc.com/cheetsheet/options_reference
  if (motor.motor_status != 4) { // 0 - fail initFOC
    SerialUSB.println("ERROR:" + String(motor.motor_status));
    //return;
  }
  _delay(1000);
} // End setup
long timestamp = millis();

void loop() {
  motor.loopFOC(); // main FOC algorithm function
  long now = millis();
  if (now - 250 > timestamp) { // 250 ==> 4x per second
    if (motor.motor_status == 0) { // 0 - fail initFOC
      SerialUSB.println("ERROR:" + String(motor.motor_status));
    }
    //SerialUSB.println(String(sensor.getMechanicalAngle()) + ", " + String(sensor.getVelocity()));
    timestamp = now;
  }
  command.run(SerialUSB);
  motor.move();
  motor.monitor();
} // End loop
//▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬End_loop▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬▬
