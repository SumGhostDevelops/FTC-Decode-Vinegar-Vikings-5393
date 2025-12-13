# Code Changes to Make - Copy/Paste Ready

## ✅ OPTION 1: Quick Switch (Recommended)

### Change Line 229 in Macros.java

**BEFORE:**
```java
public void aimToAprilTag(int id, double manualAngleOffset)
{
    robot.telemetry.log().add("-aimToAprilTag---------");

    ElapsedTime searchTimer = new ElapsedTime();
    Optional<AprilTagDetection> tag = Optional.empty();
    double timeLimit = 2; // seconds max to search

    searchTimer.reset();

    while (tag.isEmpty() && searchTimer.seconds() < timeLimit)
    {
        robot.localization.webcam.updateDetections();
        tag = robot.localization.webcam.getSingleDetection(id);

        robot.telemetry.addData("Status", "Searching for Tag " + id);
        robot.telemetry.update();
    }

    if (!tag.isPresent())
    {
        robot.telemetry.log().add("Cancelling auto-aim command: Could not find Tag " + id);
        return;
    }

    double offset = tag.get().ftcPose.bearing;
    double goalOffset = Base.getGoalOffset(tag.get().ftcPose.range, offset, id, robot.telemetry);
    double currentAngle = robot.localization.getHeading();
    double targetAngle = currentAngle + offset + goalOffset + manualAngleOffset;

    aimToAngle(targetAngle);  // ← OLD: Uses original function
}
```

**AFTER (for Fast version):**
```java
public void aimToAprilTag(int id, double manualAngleOffset)
{
    robot.telemetry.log().add("-aimToAprilTag---------");

    ElapsedTime searchTimer = new ElapsedTime();
    Optional<AprilTagDetection> tag = Optional.empty();
    double timeLimit = 2; // seconds max to search

    searchTimer.reset();

    while (tag.isEmpty() && searchTimer.seconds() < timeLimit)
    {
        robot.localization.webcam.updateDetections();
        tag = robot.localization.webcam.getSingleDetection(id);

        robot.telemetry.addData("Status", "Searching for Tag " + id);
        robot.telemetry.update();
    }

    if (!tag.isPresent())
    {
        robot.telemetry.log().add("Cancelling auto-aim command: Could not find Tag " + id);
        return;
    }

    double offset = tag.get().ftcPose.bearing;
    double goalOffset = Base.getGoalOffset(tag.get().ftcPose.range, offset, id, robot.telemetry);
    double currentAngle = robot.localization.getHeading();
    double targetAngle = currentAngle + offset + goalOffset + manualAngleOffset;

    aimToAngleFast(targetAngle);  // ← NEW: Uses fast two-stage version ⭐
}
```

**AFTER (for Improved version):**
```java
    // ... same code as above ...
    aimToAngleImproved(targetAngle);  // ← NEW: Uses improved anti-jitter version
}
```

---

## ✅ OPTION 2: Create Separate Methods (Safe Testing)

### Add these new methods to Macros.java (anywhere after the existing aimToAprilTag methods)

**For testing the FAST version:**
```java
/**
 * Test version using the fast two-stage aim function.
 * Call this from your OpMode to test the fast version without modifying existing code.
 */
public void aimToAprilTagFast(int id, double manualAngleOffset)
{
    robot.telemetry.log().add("-aimToAprilTag (FAST)---------");

    ElapsedTime searchTimer = new ElapsedTime();
    Optional<AprilTagDetection> tag = Optional.empty();
    double timeLimit = 2;

    searchTimer.reset();

    while (tag.isEmpty() && searchTimer.seconds() < timeLimit)
    {
        robot.localization.webcam.updateDetections();
        tag = robot.localization.webcam.getSingleDetection(id);
        robot.telemetry.addData("Status", "Searching for Tag " + id);
        robot.telemetry.update();
    }

    if (!tag.isPresent())
    {
        robot.telemetry.log().add("Cancelling auto-aim command: Could not find Tag " + id);
        return;
    }

    double offset = tag.get().ftcPose.bearing;
    double goalOffset = Base.getGoalOffset(tag.get().ftcPose.range, offset, id, robot.telemetry);
    double currentAngle = robot.localization.getHeading();
    double targetAngle = currentAngle + offset + goalOffset + manualAngleOffset;

    aimToAngleFast(targetAngle);  // Uses the fast version
}

/**
 * Convenience method for team AprilTag with fast aiming
 */
public void aimToTeamAprilTagFast()
{
    robot.localization.webcam.updateDetections();
    
    double maxDistance = 4.5;
    Optional<Double> distanceToTag = robot.localization.webcam.getRangeToTag(robot.team.goal.id);
    
    if (distanceToTag.isEmpty())
    {
        robot.telemetry.log().add("Tag not found.");
        return;
    }
    
    double angleOffset;
    switch (robot.team)
    {
        case BLUE:
            angleOffset = RobotConstants.FORCED_ANGLE_OFFSET;
            break;
        case RED:
            angleOffset = -RobotConstants.FORCED_ANGLE_OFFSET;
            break;
        default:
            angleOffset = 0.0;
    }
    
    angleOffset *= distanceToTag.get() / maxDistance;
    
    aimToAprilTagFast(robot.team.goal.id, angleOffset);
    
    // Adjust outtake RPM based on distance
    robot.outtake.modifyTargetRPMBasedOnDistance(distanceToTag.get());
}
```

**For testing the IMPROVED version:**
```java
/**
 * Test version using the improved anti-jitter aim function.
 */
public void aimToAprilTagImproved(int id, double manualAngleOffset)
{
    robot.telemetry.log().add("-aimToAprilTag (IMPROVED)---------");

    ElapsedTime searchTimer = new ElapsedTime();
    Optional<AprilTagDetection> tag = Optional.empty();
    double timeLimit = 2;

    searchTimer.reset();

    while (tag.isEmpty() && searchTimer.seconds() < timeLimit)
    {
        robot.localization.webcam.updateDetections();
        tag = robot.localization.webcam.getSingleDetection(id);
        robot.telemetry.addData("Status", "Searching for Tag " + id);
        robot.telemetry.update();
    }

    if (!tag.isPresent())
    {
        robot.telemetry.log().add("Cancelling auto-aim command: Could not find Tag " + id);
        return;
    }

    double offset = tag.get().ftcPose.bearing;
    double goalOffset = Base.getGoalOffset(tag.get().ftcPose.range, offset, id, robot.telemetry);
    double currentAngle = robot.localization.getHeading();
    double targetAngle = currentAngle + offset + goalOffset + manualAngleOffset;

    aimToAngleImproved(targetAngle);  // Uses the improved version
}
```

---

## ✅ OPTION 3: Direct Testing in OpMode

### In your autonomous or teleop OpMode:

**Instead of:**
```java
robot.macros.aimToAprilTag(tagId, offset);
```

**Use one of these test methods (if you created them via Option 2):**
```java
robot.macros.aimToAprilTagFast(tagId, offset);       // Test fast version
robot.macros.aimToAprilTagImproved(tagId, offset);   // Test improved version
```

---

## 🎯 Quick Tuning Adjustments

### If aimToAngleFast() needs tuning, modify these lines in the function:

**Make it faster (less settling time):**
```java
// Line ~486 in aimToAngleFast()
int requiredSettledLoops = 2;          // Was 3
double angularRateThreshold = 4.0;     // Was 3.0
```

**Make it more accurate (prevent overshoot):**
```java
// Line ~479 in aimToAngleFast()
double coarseThreshold = 15.0;         // Was 12.0 (switch to fine stage sooner)
double maxPower_fine = 0.2;            // Was 0.25 (gentler in fine stage)
```

**Make approach more aggressive:**
```java
// Line ~489 in aimToAngleFast()
double maxPower_coarse = 0.9;          // Was 0.85
double kP_coarse = 0.03;               // Was 0.025
```

**Make settling more stable:**
```java
// Line ~481 in aimToAngleFast()
double fineTolerance = 0.3;            // Was 0.5 (tighter position requirement)
double angularRateThreshold = 2.0;     // Was 3.0 (stricter rotation requirement)
```

---

## 🔄 How to Revert Back

If you need to go back to the original function:

**Change line 229 back to:**
```java
aimToAngle(targetAngle);  // Back to original
```

All three functions exist in your code:
- `aimToAngle()` - Original (line 232)
- `aimToAngleImproved()` - Improved (line 325)
- `aimToAngleFast()` - Fast (line 474)

You can switch between them anytime by changing which one gets called!

---

## 📋 Testing Checklist

After making the change:

- [ ] Code compiles without errors
- [ ] Robot turns toward AprilTag
- [ ] Watch telemetry output for stage transitions (COARSE → FINE)
- [ ] Check final error is < 0.5°
- [ ] Verify no jittering near target
- [ ] Time the alignment (should be ~30-40% faster)
- [ ] Test at different starting angles (10°, 45°, 90°, 180°)
- [ ] Confirm it works consistently over multiple runs

---

## 🎉 That's It!

The functions are already in your code. Just change which one gets called on line 229, or create new test methods to try them out safely.

**Recommended:** Start with `aimToAngleFast(targetAngle)` for the best performance!

