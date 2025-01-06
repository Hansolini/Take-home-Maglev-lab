# Maggy 4.0  

## Hardware Description  

### Table of Contents  

- [Purpose](#purpose)  
- [Teensy](#teensy)  
- [Sensors](#sensors)  
- [Motor Drivers](#motor-drivers)  
- [Solenoids](#solenoids)  
- [Power Supply](#power-supply)  
- [Current Measurement](#current-measurement)  
- [Connectors](#connectors)  
- [Things to Do for V4.1](#things-to-do-for-maggy-4_1)  



## Purpose <a name="purpose"></a>  

The **Maggy 4.0** represents the latest and most advanced version of the Maggy series, offering substantial improvements over earlier iterations. The new **rectangular design** replaces the previous cylindrical shape to resolve challenges related to design complexity, IPC standards compliance, production risks, and repairability.  

This redesign ensures optimal component arrangement, simplifying both manufacturing and maintenance. It also allows greater flexibility for positioning solenoids and magnets on either side of the board.  

### Key Features Introduced Since Version 2.6:  

- **USB-C power supply**  
- **8 sensors with I2C multiplexer for management**  
- **Terminal blocks for solenoid connections**  
- **Current measurement on solenoids**  
- **Analog I/O signal connector**  
- **External solenoid power connector**  
- **I2C communication for the Teensy**  

These features and improvements will be detailed in the sections below.  

You can join the project on EasyEDA to explore the board design:  
<https://u.easyeda.com/join?type=project&key=e1d90298173405562d58ddb29952aff6&inviter=5f96ec8310644096bd58132cfb3f9c97>  


<p align="center">  
  <img src="maggy40_3D_top.png" alt="Maggy4.0 Top Side" />  
  <br><em>Figure 1: Maggy 4.0 Top Side</em>  
</p>  

<p align="center">  
  <img src="maggy40_3D_bot.png" alt="Maggy4.0 Bottom Side" />  
  <br><em>Figure 2: Maggy 4.0 Bottom Side</em>  
</p> 



## Teensy

The **Teensy 4.1** microcontroller powers Maggy 4.0, offering high performance and Arduino compatibility. It enables precise magnet levitation control via **I2C communication** with sensors and drives motor controllers for the solenoids.  

With numerous I/O pins, the Teensy can manage current measurements, process analog signals, and integrate external devices, making it highly versatile for advanced projects.  

<p align="center">  
  <img src="Teensy_pinout.png" alt="Teensy Pinout Diagram" />  
  <br><em>Figure 3: Teensy 4.1 Pinout Diagram</em>  
</p>  

<p align="center">  
  <img src="maggy40_teensy.png" alt="Teensy on Maggy4.0" />  
  <br><em>Figure 4: Teensy 4.1 Mounted on Maggy 4.0</em>  
</p>  


## Sensors

Previous versions used TLE493D Hall effect sensors. However, due to I2C addressing limitations, we 
chose to equip the Maggy 4.0 with eight **TLV493D Hall effect sensors** from Infineon. Although these 
sensors all share the same I2C address, they are individually managed through an **I2C TCA9548 
multiplexer**, which allows each sensor to be selected independently. 

The TLV493D sensors measure the position and strength of a magnetic field by providing values for the 
X, Y, and Z axes. The current code already enables the levitation of a magnet using a central sensor that 
provides the magnet's position. However, the addition of these extra sensors expands the possibilities 
for magnetic field detection on the board, paving the way for advanced measurement and control 
applications.   

| ![Sensors on Maggy4.0](maggy40_sensors.png) | ![I2C Multiplexer](maggy40_mux_sensors.png) |  
|:-------------------------------------------:|:------------------------------------------:|  
| _Figure 5: Sensors on Maggy 4.0_            | _Figure 6: I2C Multiplexer_                |  



## Motor Drivers 

The Maggy 4.0 is equipped with four **A950 motor drivers**, each capable of independently powering the 
solenoids based on a PWM signal sent by the microcontroller. It is important to note that on the LLS 
pin, there is a resistor whose value determines the current limitation through the solenoids. You can 
adjust this value depending on the desired current. In our case, we limited the current to 1.1 A per 
solenoid. The formula for calculating the appropriate resistor value can be found in the component's 
datasheet. 

<p align="center">  
  <img src="maggy40_pwm.png" alt="Motor Drivers Schematic" />  
  <br><em>Figure 7: Motor Drivers on Maggy 4.0</em>  
</p>  



## Solenoids

As in previous versions, we use 18.5 mm diameter solenoids with 480 turns and a resistance of 4 ohms. 
When powered, these solenoids generate a magnetic field directed along the Z or -Z axis, depending 
on the current direction. Due to their low resistance, these coils can heat up significantly when used 
for extended periods. Future work could involve resizing the coils by finding larger ones or those with 
higher resistance, while ensuring a balance to avoid compromising the levitation quality. 
The solenoids can be placed on both sides of the board and are connected to the board via terminal 
blocks located on the top side. 

<p align="center">  
  <img src="maggy40_terminals_blocks.png" alt="Terminal Blocks" />  
  <br><em>Figure 8: Solenoid Terminal Blocks on Maggy 4.0</em>  
</p>  



## Power Supply   

In its previous versions, the Maggy was powered by a specific 12V power supply. To make the Maggy 
more flexible and reduce costs by avoiding the need for a dedicated charger, we replaced the Jack 
connector with a **USB-C connector**. However, a simple USB-C connector powers the board with 5V by 
default, which is not sufficient for our use. Therefore, we opted for the Power Delivery (PD) protocol, 
which allows communication with all fast chargers. 

To achieve this, we use the **CH224K integrated circuit**, which manages the Power Delivery 
communication. Its use is simplified by the ability to select the voltage in hardware in several ways (for 
more details, refer to the datasheet). In our case, we chose the simplest solution: a 24kΩ resistor is 
placed on the CFG1 pin, which fixes the voltage to 12V. If the 12V is successfully negotiated, the 12V 
LED will light up. 

Please note that to make this work, you need to power the board with a Power Delivery charger, which 
corresponds to most modern fast chargers and laptop chargers. If your charger cannot provide 12V, the 
integrated circuit will negotiate the highest possible voltage lower than 12V that the charger can supply. 
In many cases, this will be 9V, which is a common voltage for smartphone charging. 

<p align="center">  
  <img src="maggy40_power_supply.png" alt="Maggy Power Supply Schematic" />  
  <br><em>Figure 9: Maggy 4.0 Power Supply Schematic</em>  
</p>  


<p align="center">  
  <img src="maggy40_regulators.png" alt="Maggy Regulators Schematic" />  
  <br><em>Figure 10: Maggy 4.0 Voltage Regulators</em>  
</p>  


## Current Measurement

One of the new features in version 4.0 is the current measurement for each solenoid. To achieve this, 
we are using the **INA213**, a sensor specifically designed for this purpose, which allows us to obtain very 
precise current measurements. The INA213 works by measuring the voltage difference across a shunt 
resistor placed in the load circuit, which enables the calculation of the current flowing through the 
circuit. It is capable of measuring bidirectional current (both positive and negative) using a low
resistance shunt and a reference voltage. 

The reference voltage of 1.65V plays a crucial role in the operation of the INA213, as it serves as a 
reference level for its internal analog measurements. This voltage ensures that the internal analog-to
digital converter (ADC) operates optimally, thus improving the precision of measurements, even for 
very low currents. 

This new feature will provide us with real-time data on the current flowing through the solenoids. This 
will help us better understand their charging and discharging process while monitoring the card's 
power consumption and heating. 

<p align="center">  
  <img src="maggy40_current_measurement.png" alt="Current Measurement Schematic" />  
  <br><em>Figure 11: Maggy 4.0 Current Measurement Schematic</em>  
</p>  

The reference voltage of the INA is 1.65V, which sets the zero point for the sensor at 1.65V, allowing it 
to operate in bidirectional mode. To generate this reference voltage, we use a voltage divider from the 
3.3V supply. An operational amplifier is then used to buffer the voltage, as the voltage divider creates 
a high impedance on the VREF pin, which could affect the measurement stability.

<p align="center">  
  <img src="maggy40_vref_ina.png" alt="VREF INA213 Schematic" />  
  <br><em>Figure 12: VREF Circuit for INA213</em>  
</p>  



## Connectors

Maggy 4.0 introduces three new connectors:  

1. **CN1**: 8 analog inputs/outputs for signals or oscilloscope visualization.  
2. **CN2**: Connects motor driver outputs to power solenoids externally.  
3. **U6**: Enables I2C communication between the Teensy and external devices.  

| ![CN1 and CN2](maggy40_connectors1.png) | ![U6 Connector](maggy40_connectors2.png) |  
|:---------------------------------------:|:---------------------------------------:|  
| _Figure 13: CN1 and CN2 Connectors_     | _Figure 14: U6 Connector_               |  



## Things to Do for Maggy 4_1 

1. **Routing Optimization** 

    Refine the routing to improve manufacturability, repairability, and overall robustness. While these adjustments are not critical to the card's functionality, they will streamline production and maintenance.
    Well-organized tracks minimize manufacturing defects and simplify repairs. Below are some recommendations to optimize the design:


    - **Package Replacement**: Use 0805 packages or larger whenever possible to facilitate troubleshooting.  
    *Explanation: Larger components are easier to handle and replace.*  

    - **Track Entry Alignment**: Ensure that tracks connect directly to the footprint from the front, not the sides.  
   *Explanation: This improves connection reliability and reduces soldering issues.*  

    - **Component Spacing and Orientation**: Leave enough space around components for soldering, even if it requires rotating them 90°.  
   *Explanation: Proper spacing supports manual work and reduces errors.*  

    - **Straight Tracks**: Remove unnecessary zigzags and aim for direct paths whenever possible.  
   *Explanation: Straight tracks are shorter and improve routing clarity.*  

    - **Track Alignment**: Keep parallel tracks grouped together to maintain alignment.  
   *Explanation: Organized routing improves reliability and reduces interference.*  

    - **Ground Plane Management**: Assign a dedicated plane to the bottom and another to the top. Avoid splitting the planes with tracks and consolidate areas with vias when necessary.  
   *Explanation: Continuous ground planes enhance electrical and thermal performance.* 

