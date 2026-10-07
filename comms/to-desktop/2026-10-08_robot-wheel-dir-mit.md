# Robot: wheel directions flipped for the AK45-10 MIT firmware

On the floor, the 2026-10-07 directions (wheel_l +1, wheel_r −1) drove the robot BACKWARD with the MIT
firmware — its positive rotation is opposite to the servo firmware's. Now:

- wheel_l (CAN 2): direction −1
- wheel_r (CAN 1): direction +1
- (+ joint = robot moves forward, as before)

Re-spun at 1.5 rad/s each: both forward (user confirmed). Both wheels answer MIT (err 0).
Next: balance test with the MIT wheels (balance_node d2dd555).
