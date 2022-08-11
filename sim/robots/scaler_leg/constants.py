HARDWAREOFFSET = {
        'installation_trans': np.array(
            [[1,    # Shoulder Pair (Leg 1)
              -1,   # Top Leg Serve Pair
              1,    # Bottom Leg Servo Pair
              1,    # First Wrist Servo
              1,    # Second Wrist Servo
              1],   # Third Wrist Servo

             [1,    # Shoulder Pair (Leg 2)
              1,    # Top Leg Serve Pair
              -1,   # Bottom Leg Servo Pair
              1,   # First Wrist Servo
              -1,   # Second Wrist Servo
              1],   # Third Wrist Servo

             [1,    # Shoulder Pair (Leg 3)
              -1,   # Top Leg Serve Pair
              1,    # Bottom Leg Servo Pair
              1,    # First Wrist Servo
              1,    # Second Wrist Servo
              1],   # Third Wrist Servo

             [1,    # Shoulder Pair (Leg 4)
              1,    # Top Leg Serve Pair
              -1,   # Bottom Leg Servo Pair
              1,   # First Wrist Servo
              -1,   # Second Wrist Servo
              1]]), # Third Wrist Servo

        'installation_0': np.array(
            [[-90.0,  # Identical for a pair of motors
              90.0,
              -90.0,
              0.0,
              0.0,
              -180.0],

             [90.0,
              90.0,
              -90.0,
              0.0,
              0.0,
              0.0],

             [90.0,
              90.0,
              -90.0,
              0.0,
              0.0,
              -180.0],

             [-90.0,
              90.0,
              -90.0,
              0.0,
              0.0,
              0.0]]) / 180.0 * np.pi}
