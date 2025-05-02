printf("LED turned on\n");

    calibrate_PID(12.5f, 0.8f, 0.02f);

    printf("PID gains sent: Kp=12.5, Ki=0.8, Kd=0.02\n");

    set_degrees_of_yaw(45.0f);

    printf("Desired yaw set to 45°\n");

    set_depth_cm(150.0f);

    printf("Desired depth set to 150 cm\n");

    set_complementary_filters(0.02f, 0.02f, 0.02f, 0.5f);

    printf("Complementary filter coefficients sent\n");

    // pwm_motors_for_drive_one_by_one(
    //     1200,1200,1200,1200,
    //     1200,1200,1200,1200
    // );
    // printf("PWM values for 8 motors sent\n");

    for_arm(0);
    printf("System armed\n");