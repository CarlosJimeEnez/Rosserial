#include <Arduino.h>
// Servomotor:
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <ESP32Servo.h>
#include <cmath>

// ROS
#include <ros.h>
#include "centauri6dof_moveit/ArmJointState.h"
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <math.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>

// Encoder
#include <Wire.h>
#include <AS5600.h>
#include <Encoder.h>
#define tcaAddress 0x70

#include <vector>

using namespace std;

#define JOINT1_STEP_PIN 16 // CLK+
#define JOINT1_DIR_PIN 17  // CW+

//Este esta de mas despues borrar: 
#define JOINT2_STEP_PIN_M1 22 // CLK+
#define JOINT2_DIR_PIN_M1 24  // CW+

#define JOINT2_STEP_PIN_M2 26 // CLK+
#define JOINT2_DIR_PIN_M2 25  // CW+

#define JOINT3_STEP_PIN 33 // CLK+
#define JOINT3_DIR_PIN 32  // CW+

#define JOINT4_STEP_PIN 15 // CLK+
#define JOINT4_DIR_PIN 2   // CW+

#define JOINT5_STEP_PIN 18 // CLK+
#define JOINT5_DIR_PIN 19  // CW+

#define JOINT6_STEP_PIN 34 // CLK+
#define JOINT6_DIR_PIN 23  // CW+
AccelStepper joint1(1, JOINT1_STEP_PIN, JOINT1_DIR_PIN);
AccelStepper joint2_m1(1, JOINT2_STEP_PIN_M1, JOINT2_DIR_PIN_M1);
AccelStepper joint2_m2(1, JOINT2_STEP_PIN_M2, JOINT2_DIR_PIN_M2);
AccelStepper joint3(1, JOINT3_STEP_PIN, JOINT3_DIR_PIN);
AccelStepper joint4(1, JOINT4_STEP_PIN, JOINT4_DIR_PIN);
AccelStepper joint5(1, JOINT5_STEP_PIN, JOINT5_DIR_PIN);
AccelStepper joint6(1, JOINT6_STEP_PIN, JOINT6_DIR_PIN);

// Servo
Servo gripper;
int pos_gripper = 0;
int algo = 0; // Quesesto? :o

// Motores :
MultiStepper steppers;
int joint_step[7]; //[joint1,joint2,joint3,joint4,joint5,joint6,servo]
int joint_status = 0;
int pos = 0;
int eff0 = 0; // Efector final cerrado
int eff1 = 0; // Efector final abierto

// Encoders:
Encoder encoder1(0, 0, 0);
Encoder encoder2(0, 0, 1);
Encoder encoder3(0, 0, 2);

float Sensibility = 0.185;

// Declaración del NodeHandle con la instancia nh
ros::NodeHandle nh;
std_msgs::Int16 msg;
std_msgs::Float64 test;

// SUBSCRIPCIONES ENCODERS
std_msgs::Float32 msgCounter;
std_msgs::Float32 msgCounter2;
std_msgs::Float32 msgCounter3;
ros::Publisher pubCounter("/encoder", &msgCounter);
ros::Publisher pubCounter2("/encoder2", &msgCounter2);
ros::Publisher pubCounter3("/encoder3", &msgCounter3);

void arm_cb(const centauri6dof_moveit::ArmJointState &arm_steps)
{
  joint_status = 1;
  joint_step[0] = arm_steps.position1;
  joint_step[1] = arm_steps.position2;
  joint_step[2] = arm_steps.position3;
  joint_step[3] = arm_steps.position4;
  joint_step[4] = arm_steps.position5;
  joint_step[5] = arm_steps.position6;
  joint_step[6] = arm_steps.position7; // Posición del Gripper <0-89>
}

void gripper_cb(const std_msgs::UInt16 &cmd_msg)
{
  // gripper.write(msg_angulo.data);

  if (cmd_msg.data > 0)
  {
    for (pos = 0; pos < 200; pos += 1) // Va de 0 a 89° En pasos de 1 grado
    {
      gripper.write(pos); // Indicarle al servo que adquiera la variable pos
      delay(5);           // Esperar 5ms pra que el servo llegue a la posición
    }
  }

  if (cmd_msg.data == 0)
  {
    for (pos = 200; pos >= 1; pos -= 1) // Va de 89 a 0°
    {
      gripper.write(pos); // Indicarle al servo que adquiera la variable pos
      delay(5);           // Esperar 5ms pra que el servo llegue a la posición
    }
  }
}

/*------------------definición de los objetos subscriptores------------------*/
// la función arm_cb se ejecuta cuando hay un mensaje en el topic joint_steps
ros::Subscriber<centauri6dof_moveit::ArmJointState> arm_sub("joint_steps", arm_cb);

// la función arm_cb se ejecuta cuando hay un mensaje en el topic gripper_angle
ros::Subscriber<std_msgs::UInt16> gripper_sub("gripper_angle", gripper_cb);

void setup()
{
  Serial.begin(57600);
  Wire.begin();
  // Configura el 0 del encoder:
  encoder1.setupCero();
  encoder2.setupCero();
  encoder3.setupCero();

  joint_status = 1;
  // inicializacion del nodo para el uso de la comunicación por puerto serial
  nh.initNode();

  nh.advertise(pubCounter);
  nh.advertise(pubCounter2);
  nh.advertise(pubCounter3);
  // Inicializar subscriptores
  nh.subscribe(arm_sub);
  nh.subscribe(gripper_sub);

  // Asignación de valor de maxima velocidad para cada motor
  joint1.setMaxSpeed(1500);   // 1500, 3500
  joint2_m1.setMaxSpeed(400); // 400, 8000
  joint2_m2.setMaxSpeed(800); // 400
  joint3.setMaxSpeed(4000);   // 2000, 1550
  joint4.setMaxSpeed(1000);   // 200, 400
  joint5.setMaxSpeed(1000);   // 1000, 2500
  joint6.setMaxSpeed(800);    // 500, 700

  // Agregar motores a la libreria MultiStepper
  steppers.addStepper(joint1);
  steppers.addStepper(joint2_m1);
  steppers.addStepper(joint2_m2);
  steppers.addStepper(joint3);
  steppers.addStepper(joint4);
  steppers.addStepper(joint5);
  steppers.addStepper(joint6);

  // Asignar el puerto PWM 4 AL Gripper
  gripper.attach(4);
}

void loop()
{
  // Tomamos el valor de los encoders:
  encoder1.mapVal();
  encoder2.mapVal();
  encoder3.mapVal();
  float valEncoder1 = encoder1.SumDegTotal(2516.4, true)[0];
  float valEncoder2 = encoder2.SumDegTotal(2516.4, false)[0];
  float valEncoder3 = encoder3.SumDegTotal(2516.4, false)[0];

  //  Envío de los datos del Encoder
  msgCounter.data = (valEncoder1);
  msgCounter2.data = (valEncoder2);
  msgCounter3.data = (valEncoder3);
  pubCounter.publish(&msgCounter);
  pubCounter2.publish(&msgCounter2);
  pubCounter3.publish(&msgCounter3);

  if (joint_status == 1)
  { // Si arm_cb esta siendo llamado asigna el estado de joint_state a 1.

    long positions[7];

    positions[0] = joint_step[0];  // 8000  = 90°
    positions[1] = -joint_step[1]; //-4100  = 90°
    positions[2] = joint_step[1];  // 4100 = 90°
    positions[3] = joint_step[2];  // 18000 = 90°
    positions[4] = joint_step[3];  // 800   = 90°
    positions[5] = -joint_step[4]; // 3600  = 90°
    positions[6] = joint_step[5];  // 750   = 90°

    steppers.moveTo(positions);
    steppers.runSpeedToPosition();
    nh.spinOnce();

    if (joint_step[6] > 0)
    {
      if (eff1 == 0)
      {
        for (pos = 0; pos < 200; pos += 1)
        {                     // Va de 0 a 89° En pasos de 1 grado
          gripper.write(pos); // Indicarle al servo que adquiera la variable pos
          delay(5);           // Esperar 5ms para que el servo llegue a la posición
        }
      }
      eff1++;
      eff0 = 0;
    }

    if (joint_step[6] == 0)
    {
      if (eff0 == 0)
      {
        for (pos = 200; pos >= 1; pos -= 1)
        {                     // Va de 89 a 0°
          gripper.write(pos); // Indicarle al servo que adquiera la variable pos
          delay(5);           // Esperar 5ms para que el servo llegue a la posición
        }
      }
      eff0++;
      eff1 = 0;
    }
  }
  joint_status = 0;

  nh.spinOnce();
  delay(1);
}