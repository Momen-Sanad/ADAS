An **H-Bridge** typically uses a combination of **enable** and **control pins** to drive a motor or load. To control an H-Bridge, you generally need the following:

1. **Enable Pins** (like **H_en1**, **H_en2**) to control whether the H-Bridge is active or not.
    
2. **Control Pins** (like **H_A1**, **H_A2**, **H_A3**, **H_A4**) to control the direction of current flow through the H-Bridge and thus the rotation of the motor.
    

If you have a **PWM signal** coming from **PB3**, and you want to use it to control the H-Bridge, you can achieve this with the following logic:

- **PWM for Enable Pins**: You can use the **PWM signal** from **PB3** to control the **enable pins** (**H_en1**, **H_en2**). The **duty cycle** of the PWM signal will control how much time the H-Bridge is active.
    
- **Control Pins for Direction**: The control pins (**H_A1**, **H_A2**, **H_A3**, **H_A4**) will determine the direction of the motor. These pins typically don't need to be PWM signals; they are used to set the direction of current flow through the H-Bridge.
    

### **Control Strategy:**

1. **Enable Pins (H_en1, H_en2)**:
    
    - You can directly connect **PB3** to **H_en1** and **H_en2** or use the PWM signal to control them.
        
    - You can adjust the **PWM duty cycle** to vary the motor's speed (e.g., by controlling the **H_en1** and **H_en2** pins).
        
2. **Control Pins (H_A1, H_A2, H_A3, H_A4)**:
    
    - You will need to manually set these pins to control the direction of the motor.
        
    - For example, setting **H_A1** and **H_A2** high will rotate the motor in one direction, while **H_A3** and **H_A4** can be set high for the opposite direction.
        

### **Copying PB3 to Another Pin** (without a jumper):

If you want to **duplicate the signal from PB3** to another pin, like **PD4**, you can **set PD4's output value** to the same as **PB3**. This is done by reading the **PB3** pin value and writing it to **PD4**.

#### Example Code:

```c
u8 pb3_value = DIO_readPin(PB3); // Read PB3
DIO_setPinValue(PD4, pb3_value);  // Set PD4 to the same value
```

This method would allow you to **duplicate the signal** from **PB3** to **PD4** (or any other pin), without using a jumper. You can use this approach to ensure that both **H_en1** and **H_en2** get the same PWM signal.