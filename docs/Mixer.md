# Mixer and platform type

Mixing rules determine how servos and motors react to user and FC inputs. INAV supports various preset mixer configurations as well as custom mixing rules.

## Configuration

The mixer can be configured through the `Mixer` tab of the graphical user interface or using the CLI commands `mmix` and `smix`. `mmix` to define motor mixing rules and `smix` to define servo mixing rules.

To use a mixer preset first select the platform type then the mixer preset matching your aircraft and either press the `Load and apply` or `Load mixer` buttons. The `Load and apply` button will load the mixer, save it and ask to reboot the flight controller. The `Load mixer` button only loads the preset mixing rules, you can then edit them to suit your needs and when you are done you need to press the `Save and Reboot` button to save the rules.

Watch [this video](https://www.youtube.com/watch?v=0cLFu-5syi0) for a detailed description of the GUI and the documentation bellow for more details.

## Platform type

The platform type determines what features will be available to match the type of aircraft: available flight modes, flight modes behaviour, availability of flaps and displayed types of mixer presets. It can be set through the GUI's `Mixer tab` or through the CLI's `platform_type` setting.

Currently, following platform types are supported:

* MULTIROTOR
* AIRPLANE
* TRICOPTER

## Writing custom mixing rules

## Motor Mixing

A motor mixing rule is needed for each motor. Each rule defines weights that determine how the motor it applies to will change its speed relative to the requested throttle and flight dynamics: roll rate, pitch rate and yaw rate. The heigher a weight the more the input will have an impact on the speed of the motor. Refer to the following table for the meaning of each weight.

| Weight | Definition |
| ---------------------- | ---------- |
| THROTTLE    | Speed of the motor relative to throttle. Range [0.0, 1.0]. A motor with a weight of 0.5 will receive a command that will half of a motor with a 1.0 weight |
| ROLL    | Indicates how much roll authority this motor imparts to the roll rate of the aircraft. Range [-1.0, 1.0]. For fixed wing models this is usually set to 0. A positive value means that the motor needs to accelerate for a positive roll rate request (rolling right). A negative value means that the motor needs to decelerate. |
| PITCH    | Indicates how much pitch authority this motor imparts to the pitch rate of the aircraft. Range [-1.0, 1.0]. For fixed wing models this is usually set to 0. A positive value means that the motor needs to accelerate for a positive pitch rate request (pitching down). A negative value means that the motor needs to decelerate. |
| YAW    | Indicates how much yaw authority this motor imparts to the yaw rate of the aircraft. Range [-1.0, 1.0]. For fixed wing models with more than one motor this weight can be used to setup differential thrust. For fixed wing models with only one motor this is usually set to 0. A positive value means that the motor needs to accelerate for a positive yaw rate request (clockwise yaw seen from the top of the model). A negative value means that the motor needs to decelerate |

CLI commands to configure motor mixing rules:

The `mmix reset` command removes all the existing motor mixing rules.

The `mmix` command is used to list, create or modify rules. To list the currently defined rules run the `mmix` command without parameters.

To create or modify rules use the `mmix` command with the following syntax: `mmix <n> <throttle> <roll> <pitch> <yaw>`. `<n>` is representing the index of the motor output pin (integer). The other parameters are decimal weights for each of the inputs. To disable a mixing rule set the `throttle` weight to 0.

## Servo Mixing

At least one servo mixing rule is needed for each servo. Each rule defines how a servo will move relative to a specific input like a RC channel, or a requested flight dynamics rate or position from the flight controller.

Each servo mixing rule has the following parameters:
* Servo index: defines which servo the rule will apply to. The absolute value of the index is not important, what matters is only the relative difference between the used indexes. The rule with the smaller servo index will apply to the first servo, the next higher servo index to the second servo, etc. More than one rule can use the same servo index. The output of the rules with the same servo index are added together to give the final output for the specified servo.
* Input: the input for the mixing rule, see a summary of the input types table bellow.
* Weight: percentage of the input to forward to the servo. Range [-1000, 1000]. Mixing rule output = input * weight. If the output of a set of mixing rules is lower/higher than the defined servo min/max the output is clipped (the servo will never travel farther than the set min/max).
* Speed: maximum rate of change of the mixing rule output. Used to limit the servo speed. 1 corresponds to maximum 10µs/s output rate of change. Set to 0 for no speed limit. For example: 10 = full sweep (1000 to 2000) in 10s, 100 = full sweep in 1s.

| CLI input ID | Mixer input | Description |
|----|--------------------------|------------------------------------------------------------------------------|
| 0  | Stabilised ROLL          | Roll command from the flight controller. Depends on the selected flight mode(s) |
| 1  | Stabilised PITCH         | Pitch command from the flight controller. Depends on the selected flight mode(s) |
| 2  | Stabilised YAW           | Yaw command from the flight controller. Depends on the selected flight mode(s) |
| 3  | Stabilised THROTTLE      | Throttle command from the flight controller. Depends on the selected flight mode(s) |
| 4  | RC ROLL                  | Raw roll RC channel |
| 5  | RC PITCH                 | Raw pitch RC channel |
| 6  | RC YAW                   | Raw yaw RC channel |
| 7  | RC THROTTLE              | Raw throttle RC channel |
| 8  | RC channel 5             | Raw RC channel 5 |
| 9  | RC channel 6             | Raw RC channel 6 |
| 10 | RC channel 7             | Raw RC channel 7 |
| 11 | RC channel 8             | Raw RC channel 8 |
| 12 | GIMBAL PITCH             | Scaled pitch attitude of the aircraft [-90°, 90°] => [-500, 500] |
| 13 | GIMBAL ROLL              | Scaled roll attitude of the aircraft [-180°, 180°] => [-500, 500] |
| 14 | FEATURE FLAPS            | This input value is equal to the `flaperon_throw_offset` setting when the `FLAPERON` flight mode is enabled, 0 otherwise |
| 15 | RC channel 9             | Raw RC channel 9 |
| 16 | RC channel 10            | Raw RC channel 10 |
| 17 | RC channel 11            | Raw RC channel 11 |
| 18 | RC channel 12            | Raw RC channel 12 |
| 19 | RC channel 13            | Raw RC channel 13 |
| 20 | RC channel 14            | Raw RC channel 14 |
| 21 | RC channel 15            | Raw RC channel 15 |
| 22 | RC channel 16            | Raw RC channel 16 |
| 23 | Stabilized ROLL+         | Clipped between 0 and 1000 |       
| 24 | Stabilized ROLL-         | Clipped between -1000 and 0 |
| 25 | Stabilized PITCH+        | Clipped between 0 and 1000 |
| 26 | Stabilized PITCH-        | Clipped between -1000 and 0 |
| 27 | Stabilized YAW+          | Clipped between 0 and 1000 |
| 28 | Stabilized YAW-          | Clipped between -1000 and 0 |
| 29 | Logic one                | Constant equals 500 used in combination with LC (logic conditions) |


The `smix reset` command removes all the existing motor mixing rules.

The `smix` command is used to list, create or modify rules. To list the currently defined rules run the `smix` command without parameters.

To create or modify rules use the `smix` command with the following syntax: `smix <n> <servo_index> <input_id> <weight> <speed> <condition>`.

`<n>` is representing the index of the servo mixing rule to create or modify (integer). To disable a mixing rule set the weight to 0.

`<condition>` is using for logic condition. Default value 0 mean that no logic condition is used and `smix` is always enabled.

`<condition>` are stored in binary moving:

CLI logic 0 is first condition and means 00000001, so put 1 `<condition>`.

CLI logic 1 is second condition and means 00000010, so put 2 `<condition>`.

CLI logic 2 is third condition and means 00000100, so put 4 `<condition>`.

...

CLI logic 7 is eighth condition and means 10000000, so put 128 `<condition>`.

It is passible to use multiple condition together. For example: if You want use CLI `logic 1` and `logic 3` together put 10 (2 (00000010) + 8 (00001000)).

When `<condition>` is positive, conditions are combined using the AND operator, so all conditions must be true to enable `smix`.

When `<condition>` is negative, conditions are combined using the OR operator, so is enough only one conditions must be true to enable `smix`.

## Logic Conditions (LC)

### Operation type

| CLI input ID | Operation type | Operator used | Description |
|----|----------|---------------|------------------------------------------------------------------------------|
| 0  | True     | True | Always return true value |
| 1  | Equal    | A = B | Return true if operand A equals operand B, othervise return false |
| 2  | Greater  | A > B | Return true if operand A is greater then operand B, othervise return false |
| 3  | Lower    | A < B | Return true if operand A is lower then operand B, othervise return false |
| 4  | Between  | A>=B AND A<=C | Return true if operand A is between operand B and operand C, othervise return false |

### Operand type

| CLI input ID | Operand type | Operand value | Description |
|----|------------|-------------|------------------------------------------------------------------------------|
| 0  | Value      | A simply number |  |
| 1  | RX channel | Number of channel |  |
| 2  | Flight     | Number of flight operand value |  |

### Flight operand value

| CLI input ID | Unit | Flight operand value | Description |
|----|----------------|-----------|----------------------------------------------------------------------------|
| 0  | ARM_TIMER      | s         |  |
| 1  | HOME_DISTANCE  | m         |  |
| 2  | TRIP_DISTANCE  | m         |  |
| 3  | RSSI           | % (0-99)  |  |
| 4  | VBAT           | Volt / 10 |  |
| 5  | CELL_VOLTAGE   | Volt / 10 |  |
| 6  | CURRENT        | Amp / 100 |  |
| 7  | MAH_DRAWN      | mAh       |  |
| 8  | GPS_SATS       | count     |  |
| 9  | GROUD_SPEED    | cm/s      |  |
| 10 | 3D_SPEED       | cm/s      |  |
| 11 | AIR_SPEED      | cm/s      |  |
| 12 | ALTITUDE       | cm        |  |
| 13 | VERTICAL_SPEED | cm/s      |  |
| 14 | TROTTLE_POS    | %         |  |
| 15 | ATTITUDE_ROLL  | deg       |  |
| 16 | ATTITUDE_PITCH | deg       |  |

### Example

Flaps and spoilers
```
logic 0 1 4 1 10 0 1000 0 1200 0 //This is condition for spoilers
```
//First logic condition (0), is enabled (1), operator between (4), rx channel (1), ten (10) 
//must be between value (0) 1000 and value (0) 1200, flag (0) is not used. 
//From 1201 to 1400 no flap or spoilers, so no condition
```
logic 1 1 4 1 10 0 1401 0 1600 0 //Small flaps, 1401 because between operator use <= and >=
logic 2 1 4 1 10 0 1601 0 1800 0 //Full flaps

smix 0 2 1 100 0 0 //elewator
smix 1 3 2 100 0 0 //rudder
smix 2 4 0 100 0 0 //aileron 1
smix 3 5 0 -100 0 0 //aileron 2
smix 4 4 29 -40 0 -3 //spoilers - use logic one input source (29)
smix 5 5 29 40 0 -3 //spoilers - enabled when logic 0 or logic 1 is true
smix 6 4 29 60 0 -6 //flaps - enabled when logic 0 or logic 1 is true
smix 7 5 29 -60 0 -6 //flaps
```
