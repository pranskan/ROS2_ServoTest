# Manual FK/IK Calibration Guide

## Current Status

**Working:** ✅
- Servo control (angles → movement)
- Manual teleop control  
- Safe startup

**Broken:** ❌
- Forward kinematics (angles → XYZ display) - Shows wrong position
- Inverse kinematics (XYZ → angles) - Can't plan paths

---

## Step-by-Step Calibration

### Step 1: Measure Position A (All at 90°)

```bash
# Terminal 1
cd ~/ROS2_ServoTest
source /opt/ros/jazzy/setup.bash
source ~/ros2_servo_venv/bin/activate
python3 servo_control.py

# Wait for "READY!"

# Terminal 2
cd ~/ROS2_ServoTest
source /opt/ros/jazzy/setup.bash
source ~/ros2_servo_venv/bin/activate
python3 teleop_keyboard.py

# In teleop:
# Press 'c' to center all servos to 90°
# Press 's' to see status
```

**What it displays:**
```
Position: (X, Y, Z) cm
```

**Now measure with ruler:**
1. Horizontal distance from base center to gripper tip: _____ cm
2. Vertical height from ground to gripper tip: _____ cm

**Tell me these 4 numbers:**
- Displayed X: _____ cm
- Displayed Z: _____ cm
- Measured horizontal: _____ cm
- Measured height: _____ cm

---

### Step 2: I Calculate the Fix

Once you give me the 4 numbers above, I'll:
1. Calculate the exact offset angle
2. Give you updated code for `kinematics.py`
3. Show you exactly what to change

---

### Step 3: Test the Fix

After applying my code changes:

```bash
# Restart both terminals

# Terminal 1
python3 servo_control.py

# Terminal 2
python3 teleop_keyboard.py
# Press 'c', then 's'
```

**Check:** Displayed XYZ should now match your ruler measurements!

---

## Example

**Good measurement format:**
```
Displayed X: 28.17 cm
Displayed Z: 3.52 cm
Measured horizontal: 25 cm
Measured height: 40 cm
```

Then I'll calculate the fix and tell you exactly what to update!

---

**Ready? Do Step 1 and tell me the 4 numbers!** 📏
