#include <SimpleFOC.h>

// Stepper motor instance
StepperMotor motor1 = StepperMotor(50);
StepperMotor motor2 = StepperMotor(50);
// Stepper driver instance
StepperDriver4PWM driver1 = StepperDriver4PWM(13, 12, 14, 27);
StepperDriver4PWM driver2 = StepperDriver4PWM(13, 12, 14, 27);

// encoder instance
MagneticSensorAnalog sensor1 = MagneticSensorAnalog(15, 0, 4095);
MagneticSensorAnalog sensor2 = MagneticSensorAnalog(2, 0, 4095);

// commander interface
Commander command = Commander(Serial);
void onMotor1(char* cmd){ command.motor(&motor1, cmd); }
void onMotor2(char* cmd){ command.motor(&motor2, cmd); }

void setup() {

  // initialize encoder sensor hardware
  sensor1.init();
  sensor2.init();
  
  // link the motor to the sensor
  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);

  // choose FOC modulation
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor2.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // power supply voltage [V]
  driver1.voltage_power_supply = 12;
  driver1.init();
  driver2.voltage_power_supply = 12;
  driver2.init();
  // link the motor to the sensor
  motor1.linkDriver(&driver1);
  motor2.linkDriver(&driver2);

  //motor.torque_controller = TorqueControlType::voltage;

  // set control loop type to be used
  motor1.controller = MotionControlType::angle;
  motor2.controller = MotionControlType::angle;
  

  // controller configuration based on the control type 
  motor1.PID_velocity.P = 0.2;
  motor1.PID_velocity.I = 20;
  motor1.PID_velocity.D = 0.00;
  //motor1.PID_velocity.output_ramp = .5;
  motor2.PID_velocity.P = 0.2;
  motor2.PID_velocity.I = 20;
  motor2.PID_velocity.D = 0.00;
  //motor2.PID_velocity.output_ramp = .5;
  
  // default voltage_power_supply
  motor1.voltage_limit = 12;
  motor1.current_limit = 1.3;
  motor2.voltage_limit = 12;
  motor2.current_limit = 1.3;
  // velocity low pass filtering time constant
  motor1.LPF_velocity.Tf = 0.01;
  motor2.LPF_velocity.Tf = 0.01;

  // angle loop controller
  motor1.P_angle.P = 20;
  motor2.P_angle.P = 20;
  // angle loop velocity limit
  motor1.velocity_limit = 50;
  motor2.velocity_limit = 50;

  // use monitoring with serial for motor init
  // monitoring port
  Serial.begin(9600);
  // comment out if not needed
  motor1.useMonitoring(Serial);
  motor2.useMonitoring(Serial);

  // initialise motor
  motor1.init();
  motor2.init();
  // align encoder and start FOC
  motor1.initFOC();
  motor2.initFOC();

  // set the initial target value
  motor1.target = 2;
  motor2.target = 2;

  // define the motor id
  command.add('M', onMotor1, "motor1");
  command.add('N', onMotor2, "motor2");

  // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
  Serial.println(F("Motor commands sketch | Initial motion control > torque/voltage : target 2V."));
  
  _delay(1000);
}


void loop() {
  // iterative setting FOC phase voltage
  motor1.loopFOC();
  motor2.loopFOC();

  // iterative function setting the outter loop target
  // velocity, position or voltage
  // if tatget not set in parameter uses motor.target variable
  motor1.move();
  motor2.move();

  // user communication
  command.run();
}
